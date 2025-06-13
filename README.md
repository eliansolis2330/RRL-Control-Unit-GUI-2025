# RRL-Control-Unit-GUI-2025
This is a Robot Rescue League user interface from our mexican team 'Nixito', this consists in the brain and smart part of the robot for this 2025 Robocup
>Para ejecutar la interfaz es necesario crear preferentemente un entorno virtuaal para no tener conflicto con las librerías y su administrador (pip)

>La interfaz para mostrar un video en la parte principal necesita al menos 2 webcams conectadas
>En la parte superior derecha de encuentra un input para los indices de cámras de opencv
>Para el correcto funcionamiento de los multiplicadores que requieren una cámara, EVITAR configurar e iniciar el Indice 0,
pues este está reservado en cada script para su respectivo algoritmo y uso en un hilo distinto

>Todos los recursos fueron subidos a esta carpeta, es importante que todos estén en el mismo nivel del directorio, como la imagen del logo
y el archivo .pt perteneciente al modelo

>Para hacer funcionar el script runyolov10.py que despliega el modelo entrenado tipo S de 30 objetos, es necesario seguir los siguiente pasos:

  * regresar una carpeta hacia atrás del entorno (salir de .venv)*
  * ejecutar las siguientes líneas en la terminal:
    - git clone https://github.com/THU-MIG/yolov10.git
    - cd yolov10
    - pip install .
  * acceder de nuevo a la carpeta donde se encuentra el main y ejecutar la siguiente linea en la terminal:
    - pip install huggingface-hub==0.24.7
De esta manera, y con el hipnoss.pt en la carpeta de main, el modelo puede ser ejecutado

>>Instalar bibliotecas en el entorno<<
(en caso de no detectar pip en el entorno, ir a la terminal del sistema y ejecutar
    sudo apt-get install python3-pip y pip3 install virtualenv para poder crear el entorno y usar pip desde ahí)

Ejecutar en la terminal del entorno:

    - pip install supervision
    - pip install ultralytics
    - pip install opencv-python
    - pip install customtkinter
    - pip install SpeechRecognition
    - pip install pillow
    - pip install pyserial
    - pip install rplidar
    - pip install matplotlib


Una vez instaladas todas las librerías, y con todos los archivos en el mismo directorio, el proyecto puede ser ejecutado
En caso de obtener una advertencia de tipo 'no module named ...', instalar esa librería.

Archivo principal: main.py

Scripts auxiliares gestionados por hilos:

-movementDetection.py
-qrDetector.py
-thermalCamera.py
-runyolov10.py
-slam.py
