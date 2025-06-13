#!/usr/bin/env python
import cv2
import subprocess
import os
import signal
import threading
import time
import numpy as np
import yaml
import math
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PointStamped
import tf2_ros
import tf2_geometry_msgs

# -------------------- ROS COMMANDS AND CONTROL --------------------

# Lista para almacenar los procesos lanzados (terminales)
procesos = []


def minimizar_terminal(title):
    try:
        out = subprocess.check_output(["xdotool", "search", "--name", title]).decode().strip()
        if out:
            window_id = out.split()[0]
            subprocess.run(["xdotool", "windowminimize", window_id])
            print(f"Terminal con título '{title}' minimizada.")
    except Exception as e:
        print(f"Error al minimizar terminal '{title}': {e}")


def ejecutar_en_nueva_terminal(comando, title):
    full_command = f'echo -ne "\033]0;{title}\007"; {comando}; exec bash'
    print(f"Ejecutando en nueva terminal: {full_command} con título '{title}'")
    proceso = subprocess.Popen(
        ['gnome-terminal', '--', 'bash', '-c', full_command],
        preexec_fn=os.setsid
    )
    procesos.append(proceso)
    time.sleep(2)
    minimizar_terminal(title)


def cerrar_procesos_ros():
    print("Cerrando procesos ROS...")
    subprocess.run("pkill -f '/home/nix/catkin_ws/src/rplidar_ros/launch/rplidar_a2m12.launch'", shell=True)
    subprocess.run("pkill -f '/home/nix/catkin_ws/src/hector_slam/hector_slam_launch/launch/tutorial.launch'",
                   shell=True)
    print("Procesos ROS cerrados.")


def ejecutar_comandos():
    print("Ejecutando comando 1: sudo chmod 666 /dev/ttyUSB0")
    contrasena = "nix123"  # Evita dejar contraseñas en claro en producción
    subprocess.run(f'echo {contrasena} | sudo -S chmod 666 /dev/ttyUSB0', shell=True)

    print("Ejecutando comando 2: cd ~/catkin_ws/")
    os.chdir(os.path.expanduser("~/catkin_ws"))

    print("Ejecutando comando 3: roslaunch rplidar_ros rplidar_a2m12.launch")
    ejecutar_en_nueva_terminal("roslaunch /home/nix/catkin_ws/src/rplidar_ros/launch/rplidar_a2m12.launch",
                               "Terminal_RPLidar")

    time.sleep(5)

    print("Ejecutando comando 4: roslaunch hector_slam_launch tutorial.launch")
    ejecutar_en_nueva_terminal(
        "roslaunch /home/nix/catkin_ws/src/hector_slam/hector_slam_launch/launch/tutorial.launch",
        "Terminal_HectorSLAM")


def iniciar_comandos_en_hilo():
    try:
        ejecutar_comandos()
    except Exception as e:
        print(f"Error al ejecutar comandos: {e}")
        os._exit(1)


# -------------------- Marker Subscriber and Overlay --------------------

# Global para almacenar los markers recibidos
# Estructura: clave = marker id, valor = dict con { 'x', 'y', 'color', 'text' }
global_markers = {}


def marker_callback(msg):
    global global_markers
    if msg.ns == "hazmat":
        global_markers[msg.id] = {
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'color': (msg.color.r, msg.color.g, msg.color.b),
            'text': global_markers.get(msg.id, {}).get('text', "")
        }
    elif msg.ns == "hazmat_text":
        if msg.id in global_markers:
            global_markers[msg.id]['text'] = msg.text
        else:
            global_markers[msg.id] = {
                'x': msg.pose.position.x,
                'y': msg.pose.position.y,
                'color': (1.0, 1.0, 1.0),
                'text': msg.text
            }


def overlay_markers_on_map(yaml_path, image_path, output_path):
    # Cargar información del mapa (.yaml) para obtener resolución y origen
    with open(yaml_path, 'r') as f:
        map_info = yaml.safe_load(f)
    resolution = map_info["resolution"]
    origin = map_info["origin"]  # [origin_x, origin_y, theta]

    # Cargar la imagen del mapa (.pgm)
    img = cv2.imread(image_path)
    if img is None:
        print("Error al cargar la imagen del mapa.")
        return
    height, width, _ = img.shape

    # Para cada marker, calcular posición en píxeles y dibujar
    for marker_id, data in global_markers.items():
        marker_x = data['x']
        marker_y = data['y']
        pixel_x = int((marker_x - origin[0]) / resolution)
        pixel_y = int(height - ((marker_y - origin[1]) / resolution))
        color = data['color']
        color_bgr = (int(color[2] * 255), int(color[1] * 255), int(color[0] * 255))
        cv2.circle(img, (pixel_x, pixel_y), 5, color_bgr, -1)
        if data['text']:
            cv2.putText(img, data['text'], (pixel_x + 5, pixel_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_bgr, 1)

    cv2.imwrite(output_path, img)
    print(f"Mapa con markers guardado en: {output_path}")


# -------------------- Ventana de Control --------------------

def ventana_control():
    # Inicializar el nodo ROS si no lo está
    if not rospy.core.is_initialized():
        rospy.init_node('control_node', anonymous=True)
    # Suscribirse al tópico de markers
    rospy.Subscriber('/hazmat_marker', Marker, marker_callback)

    cv2.namedWindow("Control Hector SLAM")
    img = np.zeros((300, 500, 3), dtype="uint8")
    cv2.putText(img, "Presiona 'm' para guardar el mapeo con señales", (20, 150),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)

    while True:
        cv2.imshow("Control Hector SLAM", img)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q') or key == 27:  # 'q' o Esc para salir
            cerrar_procesos_ros()
            os._exit(0)
        elif key == ord('m'):
            print("Guardando el mapeo...")
            # Guardar el mapa usando map_saver
            subprocess.run("rosrun map_server map_saver -f ~/catkin_ws/mapa_guardado", shell=True)
            print("Mapeo guardado. Ahora se sobrepone la detección de señales en el mapa.")
            time.sleep(2)  # Esperar a que se guarden los archivos
            mapa_jpg = os.path.expanduser("~/catkin_ws/mapa_guardado_con_señales.jpg")
            overlay_markers_on_map(os.path.expanduser("~/catkin_ws/mapa_guardado.yaml"),
                                   os.path.expanduser("~/catkin_ws/mapa_guardado.pgm"),
                                   mapa_jpg)
            subprocess.run("xdg-open " + mapa_jpg, shell=True)
            cerrar_procesos_ros()
            os._exit(0)

    cv2.destroyAllWindows()


# -------------------- Función Principal --------------------

if __name__ == "__main__":
    # Ejecutar los comandos ROS en un hilo separado
    hilo_comandos = threading.Thread(target=iniciar_comandos_en_hilo, daemon=True)
    hilo_comandos.start()

    # Ejecutar la ventana de control (que incluye la funcionalidad de 'm' para guardar el mapeo)
    ventana_control()
