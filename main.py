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

colorTheme = '#12fe35'
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

# Variables globales para el brazo robótico
robot_arm_ani = None
robot_arm_line1 = None
robot_arm_line2 = None
robot_arm_line3 = None
robot_arm_fig = None
robot_arm_ax = None
robot_arm_joint_points = None  # Para los marcadores de las articulaciones

# Poses definidas para el movimiento del brazo (theta1, theta2, theta3) en radianes
# Estas poses se han elegido para simular un movimiento "realista"
# theta1: ángulo de la base (plano XY)
# theta2: ángulo de la primera articulación (elevación/descenso)
# theta3: ángulo de la segunda articulación (extensión/retracción)
ROBOT_ARM_POSES = [
    (0, np.pi / 4, np.pi / 4),  # Pose inicial (brazo ligeramente elevado)
    (np.pi / 3, np.pi / 6, np.pi / 3),  # Moverse a la derecha, extender un poco
    (-np.pi / 4, np.pi / 2, 0),  # Moverse a la izquierda, más elevado, más recto
    (0, np.pi / 8, np.pi / 2),  # Centro, extender más
    (np.pi / 6, np.pi / 4, np.pi / 6)  # Otra pose de trabajo
]
POSE_TRANSITION_FRAMES = 50  # Número de frames para transicionar entre cada pose

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
    print("Conexión establecida con ESP32")
except serial.SerialException:
    print("No se pudo abrir el puerto serie")

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
# --- FIN DE configure_lidar_plot MOVIDA ---

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
    print("Conexión establecida con ESP32")
except serial.SerialException:
    print("No se pudo abrir el puerto serie")

def start_lidar_animation():
    global lidar_ani, lidar_instance, lidar_line, lidar_ax, lidar_fig

    if lidar_instance is None:
        lidar_instance = create_lidar_gui()
        if lidar_instance is None:
            print("No se pudo iniciar el LIDAR, la animación no se iniciará.")
            return

    # Esta parte se asegura de que el gráfico ya esté configurado antes de iniciar la animación
    if lidar_line is None or lidar_ax is None or lidar_fig is None:
        print("Gráfico LIDAR no configurado. Llamando a configure_lidar_plot primero.")
        # Opcional: podrías llamar configure_lidar_plot(lidar_frame) aquí si quieres que se auto-configure
        # Esto requeriría pasar lidar_frame como argumento o hacer que lidar_frame sea global.
        # Por ahora, asumimos que configure_lidar_plot ya fue llamada en create_gui.
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
    try:
        while ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').strip()
        values = line.split(",")
        if len(values) == 2:
            gas_value = int(values[0])
            mag_analog = int(values[1])
            return gas_value, mag_analog
    except Exception as e:
        print(f"Error al leer los sensores: {e}")
    return None, None


def read_magnetometer():
    _, mag_analog = read_sensors()
    if mag_analog is not None:
        scaled_value = (mag_analog - 1800) / 1800 * 550
        return int(scaled_value)
    return 0


def read_mq7_sensor():
    gas_value, _ = read_sensors()
    if gas_value is not None:
        ppm = gas_value
        return int(ppm - 100)
    return 0


def run_script(script_name):
    try:
        subprocess.run(["python3", script_name], check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error al ejecutar {script_name}: {e}")


def execute_script(script_name):
    thread = threading.Thread(target=run_script, args=(script_name,), daemon=True)
    thread.start()



def create_flippers_animation(canvas):
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


def setup_cameras(indices, camera_frames):
    caps = []
    for i, idx in enumerate(indices):
        cap = cv2.VideoCapture(idx)
        if cap.isOpened():
            caps.append(cap)
            update_video(i, cap, camera_frames[i])
        else:
            print(f"Error al abrir la cámara con índice {idx}")
    return caps


def update_video(index, cap, camera_label):
    ret, frame = cap.read()
    if ret:
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame_image = Image.fromarray(frame_rgb)
        frame_photo = ImageTk.PhotoImage(frame_image)
        camera_label.configure(image=frame_photo)
        camera_label.image = frame_photo
    camera_label.after(30, update_video, index, cap, camera_label)


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

    if lidar_instance == None or lidar_line == None or lidar_ax == None:
        return []

    try:
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

    except (RPLidarException, StopIteration, RuntimeError) as e:
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


# --- Funciones para el Diagrama Cinemático del Brazo Robótico ---
def update_robot_arm(frame_num, L1, L2, L3, ax, line1, line2, line3, joint_points, poses, transition_frames):
    global robot_arm_line1, robot_arm_line2, robot_arm_line3, robot_arm_joint_points

    num_poses = len(poses)
    total_animation_frames = num_poses * transition_frames

    # Determine current and next pose based on frame_num
    current_pose_idx = (frame_num // transition_frames) % num_poses
    next_pose_idx = (current_pose_idx + 1) % num_poses
    frame_in_transition = frame_num % transition_frames

    start_angles = poses[current_pose_idx]
    end_angles = poses[next_pose_idx]

    # Linear interpolation of angles
    t = frame_in_transition / transition_frames
    theta1 = start_angles[0] + (end_angles[0] - start_angles[0]) * t
    theta2 = start_angles[1] + (end_angles[1] - start_angles[1]) * t
    theta3 = start_angles[2] + (end_angles[2] - start_angles[2]) * t

    # Forward Kinematics (same as before)
    x0, y0 = 0, 0  # Base (fixed at origin for simplicity)

    # End of Link 1
    x1 = L1 * np.cos(theta1)
    y1 = L1 * np.sin(theta1)

    # End of Link 2 (relative to end of Link 1)
    x2 = x1 + L2 * np.cos(theta1 + theta2)
    y2 = y1 + L2 * np.sin(theta1 + theta2)

    # End of Link 3 (relative to end of Link 2)
    x3 = x2 + L3 * np.cos(theta1 + theta2 + theta3)
    y3 = y2 + L3 * np.sin(theta1 + theta2 + theta3)

    # Update the lines
    line1.set_data([x0, x1], [y0, y1])
    line2.set_data([x1, x2], [y1, y2])
    line3.set_data([x2, x3], [y2, y3])

    # Update joint markers
    joint_points.set_offsets([[x0, y0], [x1, y1], [x2, y2], [x3, y3]])

    return line1, line2, line3, joint_points  # Return all artists for blitting


def open_kinematic_diagram_window():
    global robot_arm_fig, robot_arm_ax, robot_arm_line1, robot_arm_line2, robot_arm_line3, robot_arm_ani, robot_arm_joint_points

    # Create a new Toplevel window
    kinematic_window = ctk.CTkToplevel()
    kinematic_window.title("Diagrama Cinemático del Brazo Robótico")
    kinematic_window.geometry("600x600")  # Adjust size as needed
    kinematic_window.configure(fg_color="black")  # Set background color

    # Set up the Matplotlib figure and axes for the kinematic diagram
    robot_arm_fig = plt.Figure(figsize=(5, 5), dpi=100, facecolor='black')
    robot_arm_ax = robot_arm_fig.add_subplot(111)
    robot_arm_ax.set_facecolor('black')

    # Link lengths
    L1, L2, L3 = 1.0, 0.8, 0.6  # Puedes ajustar estas longitudes

    # Initialize the lines for the robot arm segments
    robot_arm_line1, = robot_arm_ax.plot([], [], '-', lw=4, color='cyan', label='Link 1')
    robot_arm_line2, = robot_arm_ax.plot([], [], '-', lw=4, color='#00ffff', label='Link 2')
    robot_arm_line3, = robot_arm_ax.plot([], [], '-', lw=4, color='white', label='Link 3')

    # Plot origin and joint points as a scatter plot
    robot_arm_joint_points = robot_arm_ax.scatter([], [], color='red', s=100, zorder=5)  # Larger points for joints

    # Set axis limits
    ax_limit = L1 + L2 + L3 + 0.5  # A bit of padding
    robot_arm_ax.set_xlim([-ax_limit, ax_limit])
    robot_arm_ax.set_ylim([-ax_limit, ax_limit])
    robot_arm_ax.set_aspect('equal', adjustable='box')  # Keep aspect ratio square

    # Set labels and title
    robot_arm_ax.set_xlabel("X-axis", color='white')
    robot_arm_ax.set_ylabel("Y-axis", color='white')
    robot_arm_ax.set_title("Diagrama Cinemático 3 DoF", color='white', fontsize=16)
    robot_arm_ax.tick_params(axis='x', colors='white')
    robot_arm_ax.tick_params(axis='y', colors='white')
    robot_arm_ax.grid(True, color='gray', linestyle='--', alpha=0.5)

    # Embed the Matplotlib figure into the Tkinter window
    canvas_tkagg = FigureCanvasTkAgg(robot_arm_fig, master=kinematic_window)
    canvas_tkagg.draw()
    canvas_tkagg.get_tk_widget().pack(side=ctk.TOP, fill=ctk.BOTH, expand=True)

    # Animation
    total_frames_for_animation = len(ROBOT_ARM_POSES) * POSE_TRANSITION_FRAMES
    robot_arm_ani = animation.FuncAnimation(
        robot_arm_fig,
        update_robot_arm,
        frames=total_frames_for_animation,  # Total frames for one full cycle
        fargs=(L1, L2, L3, robot_arm_ax, robot_arm_line1, robot_arm_line2, robot_arm_line3, robot_arm_joint_points,
               ROBOT_ARM_POSES, POSE_TRANSITION_FRAMES),
        interval=50,  # Milliseconds between frames
        blit=True,  # Optimized drawing
        repeat=True  # Loop animation
    )

    # Function to stop animation when window is closed
    def on_kinematic_window_close():
        global robot_arm_ani
        if robot_arm_ani:
            robot_arm_ani.event_source.stop()
            robot_arm_ani = None  # Clear reference
        plt.close(robot_arm_fig)  # Close the matplotlib figure to free resources
        kinematic_window.destroy()

    kinematic_window.protocol("WM_DELETE_WINDOW", on_kinematic_window_close)
    # kinematic_window.mainloop() # Do not call mainloop here, let the main root handle it


def on_closing(root):
    print("Cerrando aplicación...")
    stop_lidar_animation()
    global robot_arm_ani
    if robot_arm_ani:
        robot_arm_ani.event_source.stop()
        robot_arm_ani = None
        # plt.close(robot_arm_fig) # Closing the figure here might cause issues if window is already destroyed

    if ser.is_open:
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
        root.tk.call('wm', 'iconphoto', root._w, icon_photo)
    except Exception as e:
        print(f"Error al cargar el ícono: {e}")

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

    logo_image = Image.open("/home/elian/PycharmProjects/PythonProject1/.venv/nixlogo.png")
    logo_image = logo_image.resize((120, 120), Image.Resampling.LANCZOS)
    logo_photo = ImageTk.PhotoImage(logo_image)
    logo_label = ctk.CTkLabel(header_frame, image=logo_photo, text="")
    logo_label.image = logo_photo
    logo_label.pack(side="right", padx=20)

    button_frame = ctk.CTkFrame(root, height=50, fg_color="black")
    button_frame.pack(fill="x")

    buttons = [
        ('Detectar Movimiento', lambda: execute_script("movementDetection.py")),
        ("Cámara Térmica", lambda: execute_script("thermalCamera.py")),
        ("YOLOv10", lambda: execute_script("runyolov10.py")),
        ("SLAM", lambda: execute_script("slam.py")),
        ("Diagrama Cinemático", open_kinematic_diagram_window),
    ]

    for text, command in buttons:
        button = ctk.CTkButton(button_frame, text=text, width=210, height=40, font=("Arial", 18, "bold"),
                               text_color="black", fg_color="white", hover_color="#cccccc", command=command,
                               border_width=3, border_color=colorTheme, corner_radius=17)
        button.pack(side="left", padx=20, pady=10)

    camera_indices = []

    def save_indices():
        indices = camera_input.get()
        camera_indices.clear()
        camera_indices.extend(map(int, indices.split(",")))
        setup_cameras(camera_indices, camera_labels)

    camera_input = ctk.CTkEntry(header_frame, width=300, placeholder_text="Índices de cámaras (ej: 1,2)",
                                font=("Arial", 14), fg_color="white", text_color="black")
    camera_input.pack(side="right", padx=10)

    save_button = ctk.CTkButton(header_frame, text="Configurar Cámaras", font=("Arial", 14, "bold"),
                                text_color="black", fg_color=colorTheme, hover_color="#cccccc", command=save_indices,
                                corner_radius=15)
    save_button.pack(side="right", padx=10)

    main_frame = ctk.CTkFrame(root, fg_color="black")
    main_frame.pack(expand=True, fill="both")
    main_frame.grid_rowconfigure(1, weight=1)
    main_frame.grid_columnconfigure(0, weight=3)  # Cámara principal (más ancha)
    main_frame.grid_columnconfigure(1, weight=1)  # Columna de widgets (más estrecha)

    camera_labels = []
    for i in range(1):
        frame = ctk.CTkFrame(main_frame, fg_color="#1e1e1e", corner_radius=17)
        frame.grid(row=i // 2, column=i % 2, padx=10, pady=10, sticky="nsew")
        label = ctk.CTkLabel(frame, text=f"Cámara {i + 1}", font=("Arial", 18, "bold"), text_color="white")
        label.pack(expand=True, fill="both")
        camera_labels.append(label)

    # --- WIDGETS COLUMN (RIGHT SIDE) ---
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
    flipper_label.pack()

    flipper_canvas = ctk.CTkCanvas(flipper_frame, width=280, height=200, bg="#1e1e1e", highlightthickness=0)
    flipper_canvas.pack()
    create_flippers_animation(flipper_canvas)

    # --- LIDAR Visualization Widget ---
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