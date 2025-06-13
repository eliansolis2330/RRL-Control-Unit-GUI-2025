import cv2
import numpy as np

# Inicializamos la captura de video
cap = cv2.VideoCapture(0)  # Usa la cámara por defecto

# Comprobamos si la cámara está abierta
if not cap.isOpened():
    print("Error: No se puede acceder a la cámara.")
    exit()

# Establecemos la resolución de captura (por ejemplo, 1280x720)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 720)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)

# Inicializamos el primer frame (None al principio)
previous_frame = None

# Creamos una única ventana antes de entrar en el bucle
cv2.namedWindow("Detección de Movimiento", cv2.WINDOW_NORMAL)

# Establecemos un tamaño específico para la ventana (por ejemplo, 1600x900)
cv2.resizeWindow("Detección de Movimiento", 600, 600)

while True:
    ret, frame = cap.read()  # Captura un frame
    if not ret:
        print("Error al capturar el frame.")
        break

    # Convertimos el frame a escala de grises
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Suavizamos el frame para reducir el ruido
    gray_frame = cv2.GaussianBlur(gray_frame, (21, 21), 0)

    # Si no hay frame anterior, lo inicializamos
    if previous_frame is None:
        previous_frame = gray_frame
        continue

    # Calculamos la diferencia entre el frame actual y el anterior
    frame_delta = cv2.absdiff(previous_frame, gray_frame)

    # Umbralizamos la diferencia para detectar cambios significativos
    _, threshold = cv2.threshold(frame_delta, 25, 255, cv2.THRESH_BINARY)

    # Encontramos los contornos en la imagen umbralizada
    contours, _ = cv2.findContours(threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Dibujamos los contornos en el frame original
    for contour in contours:
        if cv2.contourArea(contour) > 100:  # Ignoramos contornos pequeños
            # Dibujamos puntos verdes en las áreas de movimiento
            for point in contour:
                x, y = point[0]
                cv2.circle(frame, (x, y), 2, (0, 255, 0), -1)

            # Dibujamos un rectángulo azul alrededor del área de movimiento
            (x, y, w, h) = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)  # Rectángulo azul

    # Mostramos el frame con los puntos de movimiento y los rectángulos azules en la misma ventana
    cv2.imshow("Detección de Movimiento", frame)

    # Actualizamos el frame anterior para la próxima iteración
    previous_frame = gray_frame

    # Salir del bucle si presionamos la tecla 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Liberamos la cámara y cerramos las ventanas
cap.release()
cv2.destroyAllWindows()
