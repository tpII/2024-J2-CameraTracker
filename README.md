# 2024-J2-CameraTracker

---

<div align="center">
  <table>
    <tr>
      <td><img src="https://github.com/tpII/2024-J2-CameraTracker/blob/master/images/Camera-tracker-presentation-gif.gif" alt="GIF del Robot en Acción" width="400"></td>
      <td><img src="https://github.com/tpII/2024-J2-CameraTracker/blob/master/images/Camera_tracker_image.jpg" alt="Hardware del Proyecto" width="400"></td>
    </tr>
  </table>
</div>

---

## Descripción

El proyecto **Camera Tracker** es un sistema de seguimiento de objetos que utiliza una cámara ESP32-CAM montada en una torreta. El sistema es capaz de reconocer y seguir objetos utilizando un modelo de aprendizaje automático optimizado, implementado en la plataforma Edge Impulse.

---

## Documentación

Consulta la [documentación del proyecto](https://github.com/tpII/2024-J2-CameraTracker/blob/master/docs/Camera%20Tracker%20-%20Project%20Overview%20%5BSP%5D.pdf) para obtener una visión más detallada de los objetivos, implementación y resultados del sistema.

---

## Contenido

- [Características](#características)
- [Hardware](#hardware)
- [Reconocimiento de Objetos](#reconocimiento-de-objetos)
- [Configuración del Entorno](#configuración-del-entorno)
- [Licencia](#licencia)

---

## Características

- Seguimiento de objetos.
- Integración de hardware utilizando ESP32-CAM y servomotores.
- Modelo de reconocimiento de objetos entrenado con Edge Impulse.
- Interfaz web para monitoreo y configuración.

---

## Hardware

El hardware del proyecto se basa en las siguientes etapas:

1. **Medición de Componentes**: Se realizan mediciones detalladas de los componentes.
2. **Diseño Inicial**: Se elabora un boceto preliminar a mano alzada.
3. **Diseño 3D**: Se utiliza Autodesk Fusion 360 para diseñar las piezas y exportarlas en formato `.stl` para impresión 3D.
4. **Prototipado**: Se desarrollan varias versiones del prototipo para optimizar el equilibrio y funcionalidad del sistema.

---

## Reconocimiento de Objetos

El reconocimiento de objetos se realiza mediante el uso de **Edge Impulse**, donde se entrena un modelo para identificar diferentes elementos. El proceso incluye:

1. **Captura de Imágenes**: Se toman aproximadamente 700 fotos de cada objeto con buena iluminación y fondo uniforme.
2. **Etiquetado**: Las imágenes se etiquetan manualmente en la plataforma Edge Impulse.
3. **Generación de Features**: Se especifica el "color depth" (escala de grises o RGB).
4. **Entrenamiento del Modelo**: Se ajusta el modelo y se realiza el entrenamiento.
5. **Evaluación**: Se utiliza la matriz de confusión para analizar el rendimiento del modelo y determinar posibles mejoras.

---

## Configuración del Entorno

### Instalación de Manejadores de Placa

1. Abre el IDE de Arduino.
2. Ve a **Archivo > Preferencias** y añade la URL para el ESP32 en "Gestor de URLs Adicionales":
   ```
   https://dl.espressif.com/dl/package_esp32_index.json
   ```
3. Dirígete a **Herramientas > Placa > Gestor de Placas** y busca e instala las siguientes versiones:
   - **Arduino AVR Boards**: Versión 1.8.6.
   - **ESP32 by Espressif Systems**: Versión 2.0.5.

### Configuración de la Placa

1. Selecciona **AI-Thinker ESP32-CAM** en el menú de placas.
2. Configura el puerto correspondiente al dispositivo conectado.

### Instalación de Bibliotecas

1. Abre el **Library Manager** desde el IDE de Arduino.
2. Instala las siguientes bibliotecas:
   - **ESP32Servo**: Versión 3.0.5.
   - **FreeRTOS** (incluida por defecto):
     ```cpp
     #include <freertos/FreeRTOS.h>
     #include <freertos/semphr.h>
     ```
   - **WiFi** y **esp_http_server** (incluidas por defecto):
     ```cpp
     #include <WiFi.h>
     #include "esp_http_server.h"
     ```
   - **Arduino Core** (incluida por defecto):
     ```cpp
     #include "Arduino.h"
     ```
3. Importa manualmente la biblioteca de inferencia generada por Edge Impulse:
   - Descarga el archivo `.zip` desde Edge Impulse.
   - Usa la opción **Sketch > Incluir Biblioteca > Añadir biblioteca .ZIP**.
   - Incluye los siguientes archivos en tu proyecto:
     ```cpp
     #include <lapor-project-1_inferencing.h>
     #include "edge-impulse-sdk/dsp/image/image.hpp"
     ```
4. Asegúrate de tener las bibliotecas para dibujado de bounding boxes:
   ```cpp
   #include "img_converters.h"
   #include "fb_gfx.h"
   ```

### Clonación del Repositorio

Clona el repositorio del proyecto para acceder a todo el código fuente y configuración:
```bash
git clone https://github.com/tpII/2024-J2-CameraTracker.git
```

---

## Licencia

Este proyecto está bajo la Licencia GPL 3.0. Consulta el archivo `LICENSE` para más detalles.

