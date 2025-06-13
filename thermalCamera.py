import time
import board
import busio
import numpy as np
import adafruit_mlx90640
import cv2  # Usaremos OpenCV para manejar la webcam y mostrar imágenes
import smbus

# Inicializa el bus I2C para el multiplexor PCA9548A
# Inicializa el sensor térmico MLX90640
def initialize_sensor():
    i2c = busio.I2C(board.SCL, board.SDA)
    mlx = adafruit_mlx90640.MLX90640(i2c)
    mlx.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_16_HZ
    return mlx

# Obtiene un frame térmico del sensor MLX90640
def get_thermal_frame(mlx):
    frame = np.zeros((24 * 32,))
    try:
        mlx.getFrame(frame)
    except ValueError:
        return None
    return np.reshape(frame, (24, 32))

# Normaliza los datos térmicos para mostrarlos en una imagen
def normalize_thermal_data(data_array):
    normalized = np.clip((data_array - 5) / (50 - 5), 0, 1) * 255
    return normalized.astype(np.uint8)

# Aplica un zoom virtual a la imagen térmica
def apply_virtual_zoom(frame, zoom_factor):
    if zoom_factor <= 1:
        return frame

    height, width = frame.shape[:2]
    new_width = int(width / zoom_factor)
    new_height = int(height / zoom_factor)

    # Coordenadas del centro
    x1 = (width - new_width) // 2
    y1 = (height - new_height) // 2
    x2 = x1 + new_width
    y2 = y1 + new_height

    cropped_frame = frame[y1:y2, x1:x2]
    return cv2.resize(cropped_frame, (width, height))

# Función principal que ejecuta la lógica de captura
def main():
    # Inicializa el multiplexo

    # Inicializa el sensor térmico
    mlx = initialize_sensor()

    # Inicializa la webcam
    cap = cv2.VideoCapture(4)
    if not cap.isOpened():
        print("Error: No se pudo acceder a la webcam.")
        return

    # Configura el tamaño de la salida
    output_size = (640, 480)
    zoom_factor = 1.5  # Ajustar este valor para el nivel de zoom deseado

    try:
        while True:
            # Captura de la webcam
            ret, webcam_frame = cap.read()
            if not ret:
                print("Error al leer la imagen de la webcam.")
                break

            # Redimensionar y reflejar la imagen de la webcam
            webcam_frame = cv2.resize(webcam_frame, output_size)

            # Captura de la cámara térmica
            thermal_frame = get_thermal_frame(mlx)
            if thermal_frame is not None:
                # Normaliza la imagen térmica
                thermal_image = normalize_thermal_data(thermal_frame)

                # Aplica zoom virtual a la imagen térmica
                thermal_image = apply_virtual_zoom(thermal_image, zoom_factor)

                # Redimensiona y aplica un colormap
                thermal_image = cv2.applyColorMap(cv2.resize(thermal_image, output_size), cv2.COLORMAP_JET)
                thermal_image = cv2.flip(thermal_image, 1)  # Reflejo horizontal

                # Fusiona las imágenes (50% de opacidad cada una)
                combined_image = cv2.addWeighted(webcam_frame, 0.5, thermal_image, 0.8, 0)

                # Muestra la imagen combinada
                cv2.imshow('Thermal + Webcam (Overlay)', combined_image)
            else:
                # Si no se obtiene el frame térmico, muestra solo la webcam
                cv2.imshow('Thermal + Webcam (Overlay)', webcam_frame)

            # Salir si se presiona 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        print("Programa terminado por el usuario.")
    finally:
        # Liberar recursos
        cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
