# 2024-J2-CameraTracker

## Descripción

El proyecto **Camera Tracker** es un sistema de seguimiento de objetos que utiliza una cámara ESP32-CAM montada en una torreta. El sistema es capaz de reconocer y seguir objetos utilizando un modelo de aprendizaje automático optimizado, implementado en la plataforma Edge Impulse.

## Contenido

- [Características](#características)
- [Hardware](#hardware)
- [Reconocimiento de Objetos](#reconocimiento-de-objetos)
- [Configuración del Entorno](#configuración-del-entorno)
- [Licencia](#licencia)

## Características

- Seguimiento de objetos.
- Integración de hardware utilizando ESP32-CAM y servos.
- Modelo de reconocimiento de objetos entrenado con Edge Impulse.

## Hardware

El hardware del proyecto se basa en la iteración de prototipos, que incluye las siguientes etapas:

1. **Medición de Componentes**: Se realizan mediciones de todos los componentes.
2. **Diseño Inicial**: Se elabora un boceto a mano alzada.
3. **Diseño 3D**: Se utiliza Autodesk Fusion 360 para el diseño, exportándolo en formato .stl para impresión 3D con filamento PLA.
4. **Versiones de Prototipos**: Se han desarrollado varias versiones de la estructura para optimizar el equilibrio y la funcionalidad.

## Reconocimiento de Objetos

Para el reconocimiento de objetos, se utiliza **Edge Impulse**, donde se entrena un modelo para identificar al menos dos objetos: una estatuilla de Super Mario y un mouse. El proceso incluye:

1. **Captura de Imágenes**: Se toman aproximadamente 700 fotos de cada objeto con buena iluminación y fondo liso.
2. **Etiquetado**: Se etiquetan las imágenes manualmente en la plataforma Edge Impulse.
3. **Generación de Features**: Se especifica el color depth (Grayscale o RGB).
4. **Entrenamiento**: Se configura el modelo y se ejecuta el entrenamiento, (obtuvimos un F1 SCORE del 100%!).
5. **Confusion Matrix**: Con ella puedes evaluar los resultados del modelo y se identifican áreas de mejora.

## Configuración del Entorno

Para configurar el entorno de desarrollo y ejecutar el proyecto, sigue estos pasos:

1. Clona el repositorio:
   ```bash
   git clone https://github.com/tpII/2024-J2-CameraTracker.git

2. Asegúrate de tener instaladas las bibliotecas necesarias de Arduino: [PONER LIBRERIAS]

## Licencia

Este proyecto está bajo la Licencia GPL 3.0. Consulta el archivo `LICENSE` para más detalles.
