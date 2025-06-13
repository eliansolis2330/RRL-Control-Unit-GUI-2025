import cv2
import numpy as np

# Inicializamos la captura de video
cap = cv2.VideoCapture(0)  # Usa la cámara por defecto

# Comprobamos si la cámara está abierta
if not cap.isOpened():
    print("Error: No se puede acceder a la cámara.")
    exit()

# Inicializamos el detector de códigos QR
qr_detector = cv2.QRCodeDetector()

# Creamos una única ventana antes de entrar en el bucle
cv2.namedWindow("Detección de QR", cv2.WINDOW_NORMAL)

# Establecemos un tamaño específico para la ventana (por ejemplo, 1600x900)
cv2.resizeWindow("Detección de QR", 600, 600)

while True:
    ret, frame = cap.read()  # Captura un frame
    if not ret:
        print("Error al capturar el frame.")
        break

    # Detectamos y decodificamos el código QR
    value, pts, _ = qr_detector.detectAndDecode(frame)

    # Si se detecta un código QR
    if pts is not None:
        pts = np.int32(pts).reshape(-1, 2)  # Convertimos los puntos a enteros y reformateamos

        for i in range(4):
            # Dibujamos los puntos de los vértices del QR
            cv2.line(frame, tuple(pts[i]), tuple(pts[(i + 1) % 4]), (0, 255, 0), 3)

        # Opcionalmente, podemos mostrar el valor del QR en la ventana
        cv2.putText(frame, f"QR Detectado: {value}", (pts[0][0], pts[0][1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)

    # Mostramos el frame con los códigos QR enmarcados
    cv2.imshow("Detección de QR", frame)

    # Salir del bucle si presionamos la tecla 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Liberamos la cámara y cerramos las ventanas
cap.release()
cv2.destroyAllWindows()
