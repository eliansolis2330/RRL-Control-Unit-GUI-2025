import cv2
import supervision as sv
from ultralytics import YOLOv10
import os
import math

# Carga el modelo previamente entrenado
model = YOLOv10('NixitoS.pt.pt')

# Inicializa los anotadores para las cajas de detección y las etiquetas
boundingBoxAnnotator = sv.BoundingBoxAnnotator()
labelAnnotator = sv.LabelAnnotator()

# Abre la webcam
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print('No se pudo abrir la cámara')
    exit()

img_counter = 0

while True:
    ret, frame = cap.read()

    if not ret:
        break

    # Realiza las detecciones usando el modelo
    results = model(frame)[0]
    detections = sv.Detections.from_ultralytics(results)

    # Anota las imágenes con las cajas y las etiquetas
    annotatedImage = boundingBoxAnnotator.annotate(scene=frame, detections=detections)
    annotatedImage = labelAnnotator.annotate(scene=annotatedImage, detections=detections)

    # Muestra la imagen anotada
    cv2.imshow('WebCam', annotatedImage)

    # Sale del loop si se presiona la tecla ESC
    if cv2.waitKey(1) & 0xFF == ord('q'):
        print('Cerrando...')
        break

# Libera la cámara y cierra las ventanas de OpenCV
cap.release()
cv2.destroyAllWindows()
