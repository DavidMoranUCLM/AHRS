# AHRS (Attitude and Heading Reference System)

Este proyecto implementa un sistema de referencia de actitud y rumbo (AHRS) en un microcontrolador ESP32, utilizando el framework ESP-IDF. El sistema utiliza una unidad de medición inercial (IMU) y un sensor GNSS.

## Características

- **Fusión de sensores:** Utiliza un Filtro de Kalman Extendido (EKF) para combinar los datos de los sensores y obtener una estimación precisa de la actitud (orientación) en forma de cuaternión.
- **Calibración de sensores:** Incluye un mecanismo para leer y aplicar datos de calibración para el acelerómetro, el giroscopio y el magnetómetro.
- **Comunicación I2C:** Se comunica con la IMU MPU-9250 a través del bus I2C.
- **Registro de datos:** Almacena los datos de la estimación de actitud en una partición de la memoria flash.
- **Arquitectura multitarea:** Utiliza FreeRTOS para gestionar las tareas de lectura de sensores, procesamiento de datos y registro.

## Componentes

El proyecto está estructurado en varios componentes:

- **`components/`**:
  - **`ahrs`**: Lógica principal del AHRS. Encargado de organizar el resto de librerías.
  - **`EKF_C`**: Implementación del Filtro de Kalman Extendido. Encargado de procesar las medidas de la IMU
  - **`mpu9250`**: Driver para la IMU MPU-9250 derivado del repo https://github.com/UncleRus/esp-idf-lib. Encargado de generar las medidas de la IMU
  - **`partitionLog`**: Utilidades para el registro de datos en particiones de la memoria flash.
  - **`wgs84`**: Librería para procesado de fases NMEA del GNSS y cálculos asociados al estandar WGS84.
- **`esp-idf-lib`**: Submódulo que proporciona una colección de drivers y librerías para el ESP-IDF, derivado del repo https://github.com/UncleRus/esp-idf-lib.

## Puesta en marcha

1.  **Configurar el entorno:** Es necesario tener instalado y configurado el [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html).
2.  **Clonar el repositorio:**
    ```bash
    git clone --recursive https://github.com/DavidMoranUCLM/AHRS.git
    ```
3.  **Compilar y flashear:**
    ```bash
    idf.py build
    idf.py -p /dev/ttyUSB0 flash
    ```

## Uso

Al iniciar, el sistema espera a que se presione un botón conectado al pin `GPIO_NUM_22`. Una vez presionado, el sistema comienza a leer los datos de la IMU, a procesarlos con el EKF y a registrar los resultados.
