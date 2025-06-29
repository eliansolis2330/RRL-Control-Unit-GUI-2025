import customtkinter as ctk
from PIL import Image, ImageTk, ImageDraw
import cv2
import math
import threading
import subprocess
import serial
import sys
import json

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
from rplidar import RPLidar, RPLidarException
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from os import path
import time

colorTheme = '#12fe35'  # Este es el color verde
SERIAL_PORT = "/dev/ttyTHS0"
BAUD_RATE = 115200

LIDAR_BAUD_RATE: int = 256000
LIDAR_TIMEOUT: float = 0.05
LIDAR_D_MAX: int = 5000
LIDAR_I_MIN: int = 0
LIDAR_I_MAX: int = 150
LIDAR_SCAN_BUFFER: int = 25000
LIDAR_POINT_SIZE: int = 10
LIDAR_FRAME_RATE: int = 30
LINUX_DEVICE_PATH: str = '/dev/ttyUSB0'

lidar_instance = None
lidar_ani = None
lidar_line = None
lidar_ax = None
lidar_fig = None
lidar_canvas_tkagg = None

# Lista global para mantener los objetos VideoCapture y sus hilos
active_camera_caps = {} # Diccionario para cap: thread_object
camera_stop_events = {} # Diccionario para cap: threading.Event

ser = None  # Inicializar ser como None para un manejo seguro
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
    print("Conexión establecida con ESP32")
except serial.SerialException:
    print("No se pudo abrir el puerto serie. Continuando sin conexión serial.")
except Exception as e:
    print(f"Error inesperado al intentar abrir el puerto serial: {e}. Continuando sin conexión serial.")


def configure_lidar_plot(parent_frame):
    global lidar_fig, lidar_ax, lidar_line, lidar_canvas_tkagg

    plt.rcParams['toolbar'] = 'None'
    plt.rcParams['figure.facecolor'] = 'black'
    plt.rcParams['axes.facecolor'] = 'black'

    fig = plt.Figure(figsize=(4.5, 4.5), facecolor='black', dpi=100)
    ax = fig.add_subplot(111, projection='polar')
    ax.set_facecolor('black')

    ax.set_theta_zero_location('N')
    ax.set_theta_direction(-1)
    ax.set_rmax(LIDAR_D_MAX)
    ax.set_title('Inicializando LIDAR...', color='cyan', pad=20, fontsize=12)
    ax.grid(True, color='#00FF00', linestyle='-', alpha=0.3)
    ax.tick_params(colors='cyan')

    line = ax.scatter(
        [], [],
        s=LIDAR_POINT_SIZE,
        cmap='gist_ncar',
        vmin=LIDAR_I_MIN,
        vmax=LIDAR_I_MAX,
        alpha=0.9,
        edgecolors='none'
    )

    cbar = fig.colorbar(line, ax=ax, pad=0.08, fraction=0.046)
    cbar.set_label('Intensidad', rotation=270, color='white', labelpad=20)
    cbar.ax.yaxis.set_tick_params(color='white')
    plt.setp(cbar.ax.get_yticklabels(), color='white')

    canvas_tkagg = FigureCanvasTkAgg(fig, master=parent_frame)
    canvas_tkagg.draw()
    canvas_tkagg.get_tk_widget().pack(side="top", fill="both", expand=True)

    lidar_fig = fig
    lidar_ax = ax
    lidar_line = line
    lidar_canvas_tkagg = canvas_tkagg

    return fig, ax, line, canvas_tkagg


def start_lidar_animation():
    global lidar_ani, lidar_instance, lidar_line, lidar_ax, lidar_fig

    if lidar_instance is None:
        lidar_instance = create_lidar_gui()
        if lidar_instance is None:
            print("No se pudo iniciar el LIDAR, la animación no se iniciará.")
            return

    if lidar_line is None or lidar_ax is None or lidar_fig is None:
        print("Gráfico LIDAR no configurado. Llamando a configure_lidar_plot primero.")
        return

    INTERVAL = int(1000 / LIDAR_FRAME_RATE)

    lidar_ani = animation.FuncAnimation(
        lidar_fig,
        update_frame_lidar,
        interval=INTERVAL,
        blit=True,
        cache_frame_data=False
    )
    lidar_fig.canvas.mpl_connect("close_event", stop_lidar_animation)
    lidar_fig.canvas.draw_idle()


def read_sensors():
    global ser  # Asegurarse de usar la variable global 'ser'
    if ser and ser.is_open:  # Verificar si la conexión serial está abierta
        try:
            line = ""
            while ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()  # Leer la última línea disponible
            if line:
                values = line.split(",")
                if len(values) == 2:
                    gas_value = int(values[0])
                    mag_analog = int(values[1])
                    return gas_value, mag_analog
            return None, None
        except Exception as e:
            print(f"Error al leer los sensores: {e}")
            return None, None
    return None, None  # Retornar None si la conexión serial no está disponible


def read_magnetometer():
    _, mag_analog = read_sensors()
    if mag_analog is not None:
        scaled_value = (mag_analog / 4095.0) * 1000  # Escalar a un rango más manejable (0-1000)
        return int(scaled_value)
    return 0


def read_mq7_sensor():
    gas_value, _ = read_sensors()
    if gas_value is not None:
        ppm = gas_value
        return int(ppm - 100) if ppm > 100 else 0  # Asegurarse de que no sea negativo
    return 0


def run_script(script_name):
    try:
        subprocess.run(["python3", script_name], check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error al ejecutar {script_name}: {e}")
    except FileNotFoundError:
        print(
            f"Error: El script '{script_name}' no se encontró. Asegúrate de que la ruta sea correcta y el script exista.")
    except Exception as e:
        print(f"Error inesperado al ejecutar {script_name}: {e}")


def execute_script(script_name):
    thread = threading.Thread(target=run_script, args=(script_name,), daemon=True)
    thread.start()


def create_flippers_animation(canvas):
    # Ya has confirmado que esta sección funciona como quieres, así que no la modifico.
    width = 200
    height = 200

    flipper_axis_x = 180
    flipper_axis_y = 145

    flipper_length = 90
    flipper_width = 25
    colors = ["#00ffff", colorTheme]

    flipper_angles = [0, 0]
    direction = 1

    def draw_rounded_rectangle(x1, y1, x2, y2, radius=30, **kwargs):
        points = [
            x1 + radius, y1,
            x2 - radius, y1,
            x2, y1,
            x2, y1 + radius,
            x2, y2 - radius,
            x2, y2,
            x2 - radius, y2,
            x1 + radius, y2,
            x1, y2,
            x1, y2 - radius,
            x1, y1 + radius,
            x1, y1
        ]
        return canvas.create_polygon(points, **kwargs, smooth=True, tag="flipper")

    def draw_flippers():
        canvas.delete("flipper")

        draw_rounded_rectangle(40, 130, 250, 160, radius=25,
                               fill="#333333", outline=colorTheme, width=3)

        for i in range(2):
            angle_rad = math.radians(flipper_angles[i])
            end_x = flipper_axis_x + flipper_length * math.cos(angle_rad)
            end_y = flipper_axis_y - flipper_length * math.sin(angle_rad)

            flipper_color = colors[i]

            canvas.create_line(flipper_axis_x, flipper_axis_y, end_x, end_y,
                               fill=flipper_color, width=flipper_width,
                               capstyle="round", tag="flipper")

            if i == 0:
                canvas.create_oval(flipper_axis_x - 18, flipper_axis_y - 18,
                                   flipper_axis_x + 18, flipper_axis_y + 18,
                                   fill="#444444", outline="white", width=2, tag="flipper")

    def update_flippers():
        nonlocal flipper_angles, direction

        flipper_angles[0] += direction * 1
        flipper_angles[1] -= direction * 1

        if flipper_angles[0] > 80 or flipper_angles[0] < -80:
            direction *= -1

        draw_flippers()

        canvas.after(40, update_flippers)

    update_flippers()

def update_air_quality(air_quality_label):
    ppm = read_mq7_sensor()
    air_quality_label.configure(text=f"CO: {ppm} PPM")
    air_quality_label.after(1000, update_air_quality, air_quality_label)


def update_magnetometer(magnetometer_label):
    mag_value = read_magnetometer()
    magnetometer_label.configure(text=f"Mag: {mag_value}")
    magnetometer_label.after(1000, update_magnetometer, magnetometer_label)

# --- INICIO DE MODIFICACIONES PARA CÁMARAS EN HILOS SEPARADOS ---

def update_video_thread(cap, camera_label, stop_event):
    """Función que se ejecuta en un hilo separado para actualizar una cámara."""
    while not stop_event.is_set() and cap.isOpened():
        ret, frame = cap.read()
        if ret:
            # Obtener el tamaño actual del label para redimensionar el frame
            label_width = camera_label.winfo_width()
            label_height = camera_label.winfo_height()

            if label_width > 1 and label_height > 1: # Asegurarse de que el label tiene tamaño
                frame_height, frame_width, _ = frame.shape
                aspect_ratio = frame_width / frame_height

                if label_width / label_height > aspect_ratio:
                    new_height = label_height
                    new_width = int(new_height * aspect_ratio)
                else:
                    new_width = label_width
                    new_height = int(new_width / aspect_ratio)

                frame = cv2.resize(frame, (new_width, new_height), interpolation=cv2.INTER_AREA)

            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frame_image = Image.fromarray(frame_rgb)
            frame_photo = ImageTk.PhotoImage(frame_image)

            # Usar after() para actualizar la GUI desde el hilo principal de Tkinter
            camera_label.after(0, lambda ph=frame_photo: camera_label.configure(image=ph))
            camera_label.after(0, lambda ph=frame_photo: setattr(camera_label, 'image', ph)) # Mantener la referencia

        # Pequeña pausa para no sobrecargar la CPU
        time.sleep(0.01) # Ajusta esto si la fluidez es muy baja, pero no lo bajes demasiado

    if cap.isOpened():
        cap.release()
    print(f"Hilo de cámara {cap} terminado.")


def setup_cameras(indices, camera_labels):
    global active_camera_caps, camera_stop_events

    # Detener y liberar cámaras existentes antes de configurar nuevas
    stop_all_camera_threads()

    active_camera_caps.clear()
    camera_stop_events.clear()

    for i, idx in enumerate(indices):
        cap = cv2.VideoCapture(idx)
        if cap.isOpened():
            print(f"Cámara {idx} abierta exitosamente.")
            stop_event = threading.Event()
            thread = threading.Thread(target=update_video_thread, args=(cap, camera_labels[i], stop_event), daemon=True)
            thread.start()
            active_camera_caps[cap] = thread
            camera_stop_events[cap] = stop_event
        else:
            print(f"Error al abrir la cámara con índice {idx}")
            # Si una cámara no se abre, asegura que su label esté vacío
            camera_labels[i].configure(image="")
            camera_labels[i].image = None


def stop_all_camera_threads():
    global active_camera_caps, camera_stop_events
    for cap, stop_event in camera_stop_events.items():
        if not stop_event.is_set():
            stop_event.set() # Señal para detener el hilo
        if cap.isOpened():
            cap.release() # Liberar el objeto VideoCapture
            print(f"Cámara {cap} liberada.")
    active_camera_caps.clear()
    camera_stop_events.clear()

# --- FIN DE MODIFICACIONES PARA CÁMARAS EN HILOS SEPARADOS ---


def create_lidar_gui():
    global lidar_instance
    try:
        if not path.exists(LINUX_DEVICE_PATH):
            print(f"ERROR: Dispositivo no encontrado en {LINUX_DEVICE_PATH}")
            print("Conecte el LIDAR o verifique permisos (sudo chmod 666 /dev/ttyUSB0)")
            return None

        lidar = RPLidar(port=LINUX_DEVICE_PATH, baudrate=LIDAR_BAUD_RATE, timeout=LIDAR_TIMEOUT)
        lidar.start_motor()
        time.sleep(0.2)
        print(f"LIDAR conectado en {LINUX_DEVICE_PATH}")

        try:
            info = lidar.get_info()
            print(f"Modelo: {info['model']}, Firmware: {info['firmware']}, Hardware: {info['hardware']}")
            health = lidar.get_health()
            print(f"Estado: {health[0]}")
        except RPLidarException as e:
            print(f"Información del dispositivo no disponible: {e}")

        lidar_instance = lidar
        return lidar
    except (RPLidarException, OSError) as e:
        print(f"Error de inicialización LIDAR: {str(e)}")
        print("Solución de problemas:")
        print("1. Verifique conexión USB")
        print("2. Ejecute: sudo chmod 666 /dev/ttyUSB0")
        print("3. Desconecte/reconecte el dispositivo")
        return None


def update_frame_lidar(num):
    global lidar_line, lidar_ax, lidar_instance, lidar_fig

    if lidar_instance is None or lidar_line is None or lidar_ax is None:
        return []

    try:
        if not hasattr(lidar_instance, 'iter_scans'):
            print("LIDAR instance no es un objeto RPLidar válido.")
            return []

        scan = next(lidar_instance.iter_scans(
            max_buf_meas=LIDAR_SCAN_BUFFER,
            min_len=3
        ))

        if len(scan) == 0:
            return lidar_line,

        angles = np.empty(len(scan), dtype=np.float32)
        distances = np.empty(len(scan), dtype=np.float32)
        intensities = np.empty(len(scan), dtype=np.float32)

        for i, meas in enumerate(scan):
            angles[i] = np.radians(meas[1])
            distances[i] = meas[2]
            intensities[i] = meas[0]

        valid = (distances > 20) & (distances < LIDAR_D_MAX) & (intensities > 0)
        angles = angles[valid]
        distances = distances[valid]
        intensities = intensities[valid]

        offsets = np.column_stack((angles, distances))
        lidar_line.set_offsets(offsets)
        lidar_line.set_array(intensities)

        lidar_ax.set_title(f'Escaneo LIDAR - {len(distances)} puntos', color='cyan', pad=20, fontsize=12)

        return lidar_line,

    except (RPLidarException, StopIteration, RuntimeError, TypeError) as e:
        print(f"Error en update_frame_lidar (LIDAR connection/data): {str(e)}")
        return lidar_line,
    except Exception as e:
        print(f"Error crítico en update_frame_lidar: {str(e)}")
        return lidar_line,


def stop_lidar_animation():
    global lidar_ani, lidar_instance
    if lidar_ani:
        lidar_ani.event_source.stop()
        lidar_ani = None
    clean_shutdown_lidar()


def clean_shutdown_lidar():
    global lidar_instance
    try:
        if lidar_instance:
            lidar_instance.stop()
            lidar_instance.stop_motor()
            lidar_instance.disconnect()
            print("\nLIDAR desconectado correctamente")
            lidar_instance = None
    except Exception as e:
        print(f"\nError en apagado del LIDAR: {str(e)}")


def on_closing(root):
    print("Cerrando aplicación...")
    stop_lidar_animation()
    stop_all_camera_threads() # Detener todos los hilos de cámara
    global ser
    if ser and ser.is_open:
        try:
            ser.close()
            print("Conexión serial ESP32 cerrada.")
        except Exception as e:
            print(f"Error al cerrar serial ESP32: {e}")
    root.destroy()


def create_gui():
    ctk.set_appearance_mode("dark")
    ctk.set_default_color_theme("dark-blue")

    root = ctk.CTk()
    root.title("Unidad de Control del Robot de Rescate")
    root.attributes("-fullscreen", True)

    root.protocol("WM_DELETE_WINDOW", lambda: on_closing(root))

    icon_path = "/home/elian/PycharmProjects/PythonProject1/.venv/elbueno.ico"
    try:
        icon_image = Image.open(icon_path)
        icon_photo = ImageTk.PhotoImage(icon_image)
        root.wm_iconphoto(True, icon_photo)
    except Exception as e:
        print(f"Error al cargar el ícono '{icon_path}': {e}")

    header_frame = ctk.CTkFrame(root, height=100, fg_color="black")
    header_frame.pack(fill="x")

    header_label = ctk.CTkLabel(header_frame, text="Unidad de Control", font=("Arial", 45, "bold"), text_color="white")
    header_label.pack(side="left", padx=20)

    # Widget de CO2
    air_quality_header_frame = ctk.CTkFrame(header_frame, fg_color="#1e1e1e", corner_radius=15, border_width=2,
                                            border_color=colorTheme)
    air_quality_header_frame.pack(side="left", padx=20, pady=10)

    air_quality_label = ctk.CTkLabel(air_quality_header_frame, text="CO: -- PPM", font=("Arial", 18, "bold"),
                                     text_color="white", width=150)
    air_quality_label.pack(padx=10, pady=5)
    update_air_quality(air_quality_label)

    # Widget para el Magnetómetro
    magnetometer_header_frame = ctk.CTkFrame(header_frame, fg_color="#1e1e1e", corner_radius=15, border_width=2,
                                             border_color=colorTheme)
    magnetometer_header_frame.pack(side="left", padx=20, pady=10)

    magnetometer_label = ctk.CTkLabel(magnetometer_header_frame, text="Mag: --", font=("Arial", 18, "bold"),
                                      text_color="white", width=150)
    magnetometer_label.pack(padx=10, pady=5)
    update_magnetometer(magnetometer_label)

    logo_image_path = "/home/elian/PycharmProjects/PythonProject1/.venv/nixlogo.png"
    try:
        logo_image = Image.open(logo_image_path)
        logo_image = logo_image.resize((90, 90), Image.Resampling.LANCZOS)
        logo_photo = ImageTk.PhotoImage(logo_image)
        logo_label = ctk.CTkLabel(header_frame, image=logo_photo, text="")
        logo_label.image = logo_photo
        logo_label.pack(side="right", padx=20)
    except Exception as e:
        print(f"Error al cargar el logo '{logo_image_path}': {e}")
        logo_label = ctk.CTkLabel(header_frame, text="[Logo]", font=("Arial", 14), text_color="gray")
        logo_label.pack(side="right", padx=20)

    button_frame = ctk.CTkFrame(root, height=50, fg_color="black")
    button_frame.pack(fill="x")

    buttons = [
        ('Detectar Movimiento', lambda: execute_script("movementDetection.py")),
        ("Cámara Térmica", lambda: execute_script("thermalCamera.py")),
        ("YOLOv10", lambda: execute_script("runyolov10.py")),
        ("SLAM", lambda: execute_script("slam.py")),
        ("Diagrama Cinemático", lambda: execute_script("kinematic_diagram.py")), # Llama al nuevo script
    ]

    for text, command in buttons:
        button = ctk.CTkButton(button_frame, text=text, width=210, height=40, font=("Arial", 18, "bold"),
                               text_color="black", fg_color="white", hover_color="#cccccc", command=command,
                               border_width=3, border_color=colorTheme, corner_radius=17)
        button.pack(side="left", padx=20, pady=10)

    camera_indices = []

    def save_indices():
        indices_str = camera_input.get()
        if all(part.strip().isdigit() for part in indices_str.split(',')):
            camera_indices.clear()
            camera_indices.extend(map(int, indices_str.split(",")))
            setup_cameras(camera_indices, camera_labels)
        else:
            print("Entrada de índices de cámara inválida. Por favor, introduce números separados por comas (ej: 0,1).")

    camera_input = ctk.CTkEntry(header_frame, width=300, placeholder_text="Índices de cámaras (ej: 0,1)",
                                font=("Arial", 14), fg_color="white", text_color="black")
    camera_input.pack(side="right", padx=10)

    save_button = ctk.CTkButton(header_frame, text="Configurar Cámaras", font=("Arial", 14, "bold"),
                                text_color="black", fg_color=colorTheme, hover_color="#cccccc", command=save_indices,
                                corner_radius=15)
    save_button.pack(side="right", padx=10)

    main_frame = ctk.CTkFrame(root, fg_color="black")
    main_frame.pack(expand=True, fill="both")
    main_frame.grid_rowconfigure(1, weight=1)
    main_frame.grid_columnconfigure(0, weight=3)
    main_frame.grid_columnconfigure(1, weight=1)

    camera_labels = []
    # Usaremos una sola etiqueta de cámara principal para este ejemplo, si necesitas más,
    # necesitarás crear más CTkLabel y pasarlos a setup_cameras.
    camera_main_frame = ctk.CTkFrame(main_frame, fg_color="#1e1e1e", corner_radius=17)
    camera_main_frame.grid(row=0, column=0, rowspan=2, padx=10, pady=10, sticky="nsew")
    camera_label_main = ctk.CTkLabel(camera_main_frame, text="Cámara Principal", font=("Arial", 18, "bold"), text_color="white")
    camera_label_main.pack(expand=True, fill="both")
    camera_labels.append(camera_label_main)

    widget_frame = ctk.CTkFrame(main_frame, fg_color="black", corner_radius=17)
    widget_frame.grid(row=0, column=1, rowspan=2, padx=10, pady=10, sticky="nsew")
    widget_frame.grid_rowconfigure((0, 1), weight=1)
    widget_frame.grid_columnconfigure(0, weight=1)

    # Flipper Animation Widget
    flipper_frame = ctk.CTkFrame(widget_frame, fg_color="#1e1e1e", corner_radius=17, border_width=2,
                                 border_color=colorTheme)
    flipper_frame.grid(row=0, column=0, pady=10, padx=10, sticky="nsew")

    flipper_label = ctk.CTkLabel(flipper_frame, text="Estado Flippers", font=("Arial", 18, "bold"),
                                 text_color="white")
    flipper_label.pack(pady=5)

    flipper_canvas = ctk.CTkCanvas(flipper_frame, bg="#1e1e1e", highlightthickness=0)
    flipper_canvas.pack(expand=True, fill="both")
    root.after(100, lambda: create_flippers_animation(flipper_canvas))

    # LIDAR Visualization Widget
    lidar_frame = ctk.CTkFrame(widget_frame, fg_color="#1e1e1e", corner_radius=17, border_width=2,
                               border_color=colorTheme)
    lidar_frame.grid(row=1, column=0, pady=10, padx=10, sticky="nsew")

    lidar_label = ctk.CTkLabel(lidar_frame, text="Visualización LIDAR", font=("Arial", 18, "bold"),
                               text_color="white")
    lidar_label.pack(pady=5)

    configure_lidar_plot(lidar_frame)
    root.after(100, start_lidar_animation)

    root.mainloop()


if __name__ == "__main__":
    create_gui()