// Edge Impulse
#include <lapor-project-1_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"

#include "camera_config.h"
#include "web_server.h"
#include "servo_control.h"

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h> // Include FreeRTOS semaphore header

static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

// Credenciales Wi-Fi
const char *ssid = "alumnosInfo"; //"alumnosInfo";
const char *password = "InformaticaUNLP"; //"InformaticaUNLP";

/* Private variables ------------------------------------------------------- */
static bool debug_nn = false;
static bool is_initialised = false;

// Pointer to buffer that holds captured image data
uint8_t *snapshot_buf;

// Calculate the scaling factors based on the new dimensions
float scaleX = (float)EI_CAMERA_RAW_FRAME_BUFFER_COLS / 80;
float scaleY = (float)EI_CAMERA_RAW_FRAME_BUFFER_ROWS / 80;

/* Function definitions ------------------------------------------------------- */
bool ei_camera_init(void);
void ei_camera_deinit(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf);

/* Web server functions prototypes -------------------------------------------- */
void startCameraServer(void);
static esp_err_t index_handler(httpd_req_t *req);
static esp_err_t stream_handler(httpd_req_t *req);

//WEB SERVER
httpd_handle_t stream_httpd = NULL;
camera_fb_t * fb = NULL;

// Servos
uint8_t MAX_X = 80; //Posicion maxima en X 
uint8_t MAX_Y = 80; //Posicion maxima en Y
uint8_t FOV_X = 40; //Angulo FOV en X (Grados)
uint8_t FOV_Y = 60; //Angulo FOV en Y (Grados)
int DIST_X; //Distancia X del objeto al centro de la imagen
int DIST_Y; //Distancia Y del objeto al centro de la imagen
int ANG_X; //Angulo de corrimiento del objeto al centro de la imagen
int ANG_Y; //Angulo de corrimiento del objeto al centro de la imagen
int lastPosS1;
int lastPosS2;
int positionS1; //Posicion del Servo 1
int positionS2; //Posicion del Servo 2
Servo s1;
Servo s2;

int x, y, w, h;
float max_value = 0.6;
#define COLOR_GREEN  0x0000FF00
uint32_t green_color = COLOR_GREEN;

// Add these for task handling
TaskHandle_t inferenceTask;
TaskHandle_t streamTask;

// Function declarations for tasks
void inferenceLoop(void * parameter);
void streamLoop(void * parameter);

// Add this global variable at the top with other globals
bool serverStarted = false;

// Add this line to declare a mutex
SemaphoreHandle_t fbMutex;

/**
* @brief      Arduino setup function
*/
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Disable brownout detector
 
  Serial.begin(115200);
  Serial.setDebugOutput(false);

  // Initialize EI camera
  if (ei_camera_init() == false) {
      ei_printf("Failed to initialize Camera!\r\n");
  }
  else {
      ei_printf("Camera initialized\r\n");
  }
  
  // Wi-Fi connection
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("WiFi connected");
  
  Serial.print("Camera Stream Ready! Go to: http://");
  Serial.println(WiFi.localIP());

  // Create the mutex
  fbMutex = xSemaphoreCreateMutex();
  
  // Start streaming web server
  startCameraServer();
  ei_printf("\r\nStarting continious inference in 2 seconds...\r\n");
  ei_sleep(2000);

  // Create tasks for different cores
  xTaskCreatePinnedToCore(
      inferenceLoop,    // Task function
      "InferenceTask",  // Name
      10000,           // Stack size
      NULL,            // Parameters
      1,              // Priority
      &inferenceTask, // Task handle
      0              // Core ID (0)
  );

  xTaskCreatePinnedToCore(
      streamLoop,     // Task function
      "StreamTask",   // Name
      10000,         // Stack size
      NULL,          // Parameters
      1,             // Priority
      &streamTask,   // Task handle
      1             // Core ID (1)
  );

  // Initialize servos
  s1.attach(S1_PIN);
  s2.attach(S2_PIN);
}

void loop() {
    // Empty - tasks handle the work
    vTaskDelay(1000);
}

// New task function for inference
void inferenceLoop(void * parameter) {
    Serial.print("Inference task running on core: ");
    Serial.println(xPortGetCoreID());
    
    while(true) {

        if (xSemaphoreTake(fbMutex, portMAX_DELAY) == pdTRUE)  {

          max_value = 0.85;
            // instead of wait_ms, we'll wait on the signal, this allows threads to cancel us...
          if (ei_sleep(5) != EI_IMPULSE_OK) {            
              vTaskDelay(500);
              xSemaphoreGive(fbMutex);       
              return;
          }
        
          // Allocate snapshot buffer
          snapshot_buf = (uint8_t*)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE);

          // Check if allocation was successful
          if(snapshot_buf == nullptr) {
              ei_printf("ERR: Failed to allocate snapshot buffer!\n");
              free(snapshot_buf);
              xSemaphoreGive(fbMutex);
              vTaskDelay(500);
              return;
          }

          // Save frame buffer (fb) on the snapshot
          if (ei_camera_capture((size_t)EI_CLASSIFIER_INPUT_WIDTH, (size_t)EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf) == false) {
              ei_printf("Failed to capture image on inference\r\n");
              free(snapshot_buf);
              xSemaphoreGive(fbMutex);
              vTaskDelay(500);
              return;
          }

          ei::signal_t signal;
          signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
          signal.get_data = &ei_camera_get_data;

          ei_impulse_result_t result = { 0 };

          EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
          if (err != EI_IMPULSE_OK) {
              ei_printf("ERR: Failed to run classifier (%d)\n", err);
              xSemaphoreGive(fbMutex);
              free(snapshot_buf);
              vTaskDelay(500);
              return;
          }

          free(snapshot_buf);
          if (fb)
          {
              esp_camera_fb_return(fb);
          }

          xSemaphoreGive(fbMutex);



          //ei_printf("Object detection bounding boxes:\r\n");
          for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
              ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
              if (bb.value <= 0.85) {
                  continue;
              }

              ei_printf("  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n",
                      bb.label,
                      bb.value,
                      bb.x,
                      bb.y,
                      bb.width,
                      bb.height);

              if (bb.value > max_value) {              
                
                max_value = bb.value;
                // Convert bounding box coordinates to the new dimensions
                x = (int)(bb.x * scaleX);
                y = (int)(bb.y * scaleY);
                w = (int)(bb.width * scaleX);
                h = (int)(bb.height * scaleY);
              }
          }

          
          vTaskDelay(500);

          // Servo Movement
          DIST_X = x - (MAX_X*0.5);
          DIST_Y = y - (MAX_X*0.5);
          ANG_X = DIST_X * FOV_X / MAX_X;
          ANG_Y = DIST_Y * FOV_Y / MAX_Y;

          lastPosS1 = positionS1;
          lastPosS2 = positionS1;

          positionS1 = ANG_X + 90;
          positionS2 = ANG_Y + 90;

          Serial.print("angX: ");
          Serial.print(ANG_X);
          Serial.print(",angY: ");
          Serial.println(ANG_Y);

          s1.write(positionS1);
          s2.write(positionS2);
          s1.write((positionS1+lastPosS1)/2);
          s2.write((positionS2+lastPosS2)/2);
        }
    }
}

// New task function for streaming
void streamLoop(void * parameter) {
    Serial.print("Stream task running on core: ");
    Serial.println(xPortGetCoreID());
    
    while(true) {
        if(!serverStarted) {
            Serial.println("Starting camera server...");
            startCameraServer();
            if(stream_httpd != NULL) {
                serverStarted = true;
                Serial.println("Camera server started successfully");
            } else {
                Serial.println("Failed to start camera server");
                vTaskDelay(5000); // Wait 5 seconds before trying again
            }
        }
        vTaskDelay(1000); // Longer delay since we only need to check occasionally
    }
}

//EDGE IMPULSE
/**
 * @brief   Setup image sensor & start streaming
 *
 * @retval  false if initialisation failed
 */
bool ei_camera_init(void) {

    if (is_initialised) 
        return true;

    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    config.ledc_timer = LEDC_TIMER_0;
    config.ledc_channel = LEDC_CHANNEL_0;

    config.frame_size = FRAMESIZE_QVGA;//FRAMESIZE_UXGA;
    config.jpeg_quality = 12;
    config.fb_count = 2;

    //initialize the camera
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
      Serial.printf("Camera init failed with error 0x%x\n", err);
      return false;
    }

    sensor_t * s = esp_camera_sensor_get();
    // initial sensors are flipped vertically and colors are a bit saturated
    //s->set_vflip(s, 1); // flip it back

    is_initialised = true;
    return true;
}

/**
 * @brief Stop streaming of sensor data
 */
void ei_camera_deinit(void) {

    //deinitialize the camera
    esp_err_t err = esp_camera_deinit();

    if (err != ESP_OK)
    {
        ei_printf("Camera deinit failed\n");
        return;
    }

    is_initialised = false;
    return;
}


/**
 * @brief      Gets image from the frame buffer and update the snapshot buffer
 *
 * @param[in]  img_width   Width of the target image
 * @param[in]  img_height  Height of the target image  
 * @param[out] out_buf     Output buffer for the processed image
 *
 * @retval     true        Success
 * @retval     false       Failure
 *
 */
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
    if (!is_initialised) {
        ei_printf("ERR: Camera is not initialized\r\n");
        return false;
    }

    fb = esp_camera_fb_get();

    if (!fb) {
        ei_printf("Camera capture failed on ei_camera_capture\n");
        return false;
    }

    bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, out_buf);

    if(!converted){
        ei_printf("Conversion failed\n");
        return false;
    }

    ei::image::processing::crop_and_interpolate_rgb888(
      out_buf,
      EI_CAMERA_RAW_FRAME_BUFFER_COLS,
      EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
      out_buf,
      img_width,
      img_height);

    return true;
}

/**
 * @brief      Get snapshot buffer of the image and convert to RGB888 format
 *
 * @param[in]  offset    Position in the image buffer to start reading from
 * @param[in]  length    Number of pixels to process
 * @param[out] out_ptr   Pointer to store the processed pixel data
 *
 * @retval     0         Success
 * 
 * @details    This function reads RGB888 data from the snapshot buffer starting at the 
 *             specified offset. It processes 'length' number of pixels, converting each 
 *             pixel from BGR to RGB format (due to ESP32-CAM hardware quirk) and stores 
 *             them as 32-bit values in the output buffer. Each pixel is packed as:
 *             (R << 16) + (G << 8) + B
 */
static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr)
{
    // we already have a RGB888 buffer, so recalculate offset into pixel index
    size_t pixel_ix = offset * 3;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;

    while (pixels_left != 0) {
        // Swap BGR to RGB here
        // due to https://github.com/espressif/esp32-camera/issues/379
        out_ptr[out_ptr_ix] = (snapshot_buf[pixel_ix + 2] << 16) + (snapshot_buf[pixel_ix + 1] << 8) + snapshot_buf[pixel_ix];

        // go to the next pixel
        out_ptr_ix++;
        pixel_ix+=3;
        pixels_left--;
    }

    return 0;
}

/**
 * @brief Initializes and starts the camera server for streaming
 *
 * This function sets up an HTTP server to handle camera streaming requests.
 * It configures the server, defines URI handlers for the index page and
 * the video stream, and starts the server if initialization is successful.
 *
 * @return This function does not return a value.
 */
void startCameraServer() {
    if(stream_httpd != NULL) {
        Serial.println("Server is already running");
        return;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.ctrl_port = 32768;  // Add this line
    config.max_resp_headers = 16;  // Add this line
    
    Serial.println("Starting web server on port: 80");

    httpd_uri_t index_uri = {
        .uri       = "/",
        .method    = HTTP_GET,
        .handler   = index_handler,
        .user_ctx  = NULL
    };

    httpd_uri_t stream_uri = {
        .uri       = "/stream",
        .method    = HTTP_GET,
        .handler   = stream_handler,
        .user_ctx  = NULL
    };

    if (httpd_start(&stream_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(stream_httpd, &index_uri);
        httpd_register_uri_handler(stream_httpd, &stream_uri);
        Serial.println("HTTP server started");
    } else {
        Serial.println("Error starting HTTP server");
        stream_httpd = NULL;
    }
}

static esp_err_t index_handler(httpd_req_t *req) {
    const char* html = "<!DOCTYPE html>"
        "<html><head>"
        "<meta name='viewport' content='width=device-width, initial-scale=1'>"
        "<title>Camera Tracker</title>"
        "<style>"
        "body { font-family: Georgia, serif; margin: 0; padding: 20px; text-align: center; }"
        "h1 { color: #333; }"
        "img { max-width: 100%; height: auto; margin: 20px 0; }"
        ".controls { margin: 20px 0; }"
        "button { "
        "   background-color: #4CAF50; color: white; border: none; "
        "   padding: 10px 20px; margin: 5px; cursor: pointer; "
        "   border-radius: 4px; font-family: Georgia, serif; "
        "}"
        "button:hover { background-color: #45a049; }"
        "#stream { display: none; }"
        "</style>"
        "</head><body>"
        "<h1>Camera Tracker</h1>"
        "<div class='controls'>"
        "<button onclick='toggleStream()'>Start/Stop Stream</button>"
        "</div>"
        "<img id='stream' src='/stream' alt='Camera Stream'>"
        "<script>"
        "function toggleStream() {"
        "   const stream = document.getElementById('stream');"
        "   if (stream.style.display === 'none') {"
        "       stream.style.display = 'block';"
        "   } else {"
        "       stream.style.display = 'none';"
        "       stream.src = '';"
        "       setTimeout(() => { stream.src = '/stream'; }, 100);"
        "   }"
        "}"
        "</script>"
        "</body></html>";

    httpd_resp_send(req, html, strlen(html));
    return ESP_OK;
}

static void draw_face_boxes(fb_data_t *fb, int x, int y, int w, int h)
{
    if(fb->bytes_per_pixel == 2){
        green_color = ((green_color >> 16) & 0x001F) | ((green_color >> 3) & 0x07E0) | ((green_color << 8) & 0xF800);
    }

    fb_gfx_drawFastHLine(fb, x, y, w, green_color);
    fb_gfx_drawFastHLine(fb, x, y + h - 1, w, green_color);
    fb_gfx_drawFastVLine(fb, x, y, h, green_color);
    fb_gfx_drawFastVLine(fb, x + w - 1, y, h, green_color);
}

static esp_err_t stream_handler(httpd_req_t *req) {
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len = 0;
    uint8_t *_jpg_buf = NULL;
    char * part_buf[64];

    size_t out_len = 0, out_width = 0, out_height = 0;
    uint8_t *out_buf = NULL;
    bool s = false;

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if(res != ESP_OK) {
        return res;
    }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "X-Framerate", "60");

    while(true) {

        if (xSemaphoreTake(fbMutex, portMAX_DELAY) == pdTRUE) {

          fb = esp_camera_fb_get();

          if (!fb) {
              Serial.println("Camera capture failed on stream handler");
              res = ESP_FAIL;
              xSemaphoreGive(fbMutex);
              break;
          }
 
          // Check if the buffer is valid
          if (fb->buf == NULL || fb->len == 0) {
              Serial.println("Invalid frame buffer");
              res = ESP_FAIL;
              xSemaphoreGive(fbMutex);
              break;
          }

          Serial.print("Format: ");
          switch(fb->format) {
              case PIXFORMAT_RGB888:
                  Serial.println("PIXFORMAT_RGB888");
                  break;
              case PIXFORMAT_GRAYSCALE:
                  Serial.println("PIXFORMAT_GRAYSCALE");
                  break;
              case PIXFORMAT_RGB565:
                  Serial.println("PIXFORMAT_RGB565");
                  break;
              case PIXFORMAT_JPEG:
                  Serial.println("PIXFORMAT_JPEG");
                  break;
              case PIXFORMAT_YUV422:
                  Serial.println("PIXFORMAT_YUV422");
                  break;
              case PIXFORMAT_RAW:
                  Serial.println("PIXFORMAT_RAW");
                  break;
              default:
                  Serial.println("Unknown format");
          }

          out_len = fb->width * fb->height * 3;
          out_width = fb->width;
          out_height = fb->height;
          out_buf = (uint8_t*)malloc(out_len);
          if (!out_buf) {
              Serial.println("out_buf malloc failed");
              res = ESP_FAIL;
          } else {
              s = fmt2rgb888(fb->buf, fb->len, fb->format, out_buf);                        
              if (!s) {
                  free(out_buf);
                  Serial.println("to rgb888 failed");
                  res = ESP_FAIL;
              } else {
                  fb_data_t rfb;
                  rfb.width = out_width;
                  rfb.height = out_height;
                  rfb.data = out_buf;
                  rfb.bytes_per_pixel = 3;
                  rfb.format = FB_BGR888;

                  if (max_value > 0.85) {
                      draw_face_boxes(&rfb, x, y, w, h);
                  }
                  
                  s = fmt2jpg(out_buf, out_len, out_width, out_height, PIXFORMAT_RGB888, 90, &_jpg_buf, &_jpg_buf_len);
                  free(out_buf);

                  if (!s) {
                      Serial.println("fmt2jpg failed");
                      res = ESP_FAIL;
                  }
               }
          }

        //_jpg_buf_len = fb->len;
        //_jpg_buf = fb->buf;

        if (fb)
        {
            esp_camera_fb_return(fb);
        }
        xSemaphoreGive(fbMutex);
        vTaskDelay(1);

        if(res == ESP_OK){
            size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
            res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
            if(res == ESP_OK){
                res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
            }
            if(res == ESP_OK){
                res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
            }
        }

        

        if (_jpg_buf)
        {
            free(_jpg_buf);
            _jpg_buf = NULL;
        }
        if (res != ESP_OK)
        {
            Serial.println("Send frame failed failed");
            break;
        }        
        }
    }

    return res;
}
