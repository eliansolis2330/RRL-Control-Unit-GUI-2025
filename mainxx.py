import customtkinter as ctk, cv2, math, threading, subprocess, serial, sys, json
import matplotlib.animation as animation, matplotlib.pyplot as plt, numpy as np
from PIL import Image, ImageTk
from rplidar import RPLidar, RPLidarException
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from os import path
import time

# --- Configuration Constants (Moved to top for easy access and modification) ---
COLOR_THEME = '#12fe35'
SERIAL_PORT = "/dev/ttyUSB1" # !!! IMPORTANTE: Verifica que este sea el puerto correcto.
                             # Para ESP32 por USB, suele ser '/dev/ttyUSB0' o '/dev/ttyACM0'.
BAUD_RATE = 9600             # !!! IMPORTANTE: BAUD RATE COINCIDE CON TU ESP32 (Serial1.begin(9600))

LIDAR_BAUD_RATE = 256000
LIDAR_TIMEOUT = 0.05
LIDAR_D_MAX = 5000
LIDAR_I_MIN = 0
LIDAR_I_MAX = 150
LIDAR_SCAN_BUFFER = 25000
LIDAR_POINT_SIZE = 10
LIDAR_FRAME_RATE = 30
LINUX_DEVICE_PATH = '/dev/ttyUSB0'

# Flipper animation constants
FLIPPER_AXIS_X, FLIPPER_AXIS_Y, FLIPPER_LENGTH, FLIPPER_WIDTH = 180, 145, 90, 25
# IMPORTANT: Adjust this value based on the actual maximum reading from your encoders
# Si los encoders son de 10 bits y se leen analógicamente directamente, 1023.
# Si son Modbus, su rango puede ser diferente (ej. 0-360, o 0-65535 para 16-bit).
# Por ahora, mantengamos un valor que pueda acomodar 0-360 como ejemplo.
ENCODER_MAX_VALUE = 360 # Placeholder: Ajusta esto al rango máximo real de tus encoders
FLIPPER_ANGLE_RANGE = 80 # Max angle deviation from center (e.g., -80 to +80 degrees)

# --- Global Variables (for shared state, minimized) ---
lidar_inst, lidar_anim, lidar_line, lidar_ax, lidar_fig, lidar_canvas_tkagg = [None] * 6
active_cam_caps, cam_stop_events = {}, {}
ser = None # Serial connection for ESP32
# Global variable to store the latest sensor/encoder data
current_sensor_data = {
    "e1": 0, "e2": 0, "e3": 0, "e4": 0, "e5": 0,
    "mq2": 0, "ky024": 0
}
data_lock = threading.Lock() # To protect access to current_sensor_data

# Attempt serial connection
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01) # Timeout corto para lectura no bloqueante
    print(f"Conexión ESP32 establecida en {SERIAL_PORT} @ {BAUD_RATE} baudios.")
except (serial.SerialException, Exception) as e:
    print(f"Error serial al conectar con ESP32: {e}. Continuando sin conexión serial.")
    print(f"Por favor, verifica el SERIAL_PORT ({SERIAL_PORT}) y BAUD_RATE ({BAUD_RATE}).")
    print("También, asegúrate de tener permisos: 'sudo usermod -a -G dialout $USER' y reinicia o 'sudo chmod 666 {SERIAL_PORT}'")


# --- Helper Functions ---
def config_lidar_plot(parent_frame):
    global lidar_fig, lidar_ax, lidar_line, lidar_canvas_tkagg
    plt.rcParams.update({'toolbar': 'None', 'figure.facecolor': 'black', 'axes.facecolor': 'black'})
    fig = plt.Figure(figsize=(4.5, 4.5), facecolor='black', dpi=100)
    ax = fig.add_subplot(111, projection='polar')
    ax.set_facecolor('black'); ax.set_theta_zero_location('N'); ax.set_theta_direction(-1); ax.set_rmax(LIDAR_D_MAX)
    ax.set_title('Inicializando LIDAR...', color='cyan', pad=20, fontsize=12)
    ax.grid(True, color='#00FF00', linestyle='-', alpha=0.3); ax.tick_params(colors='cyan')
    line = ax.scatter([], [], s=LIDAR_POINT_SIZE, cmap='gist_ncar', vmin=LIDAR_I_MIN, vmax=LIDAR_I_MAX, alpha=0.9, edgecolors='none')
    cbar = fig.colorbar(line, ax=ax, pad=0.08, fraction=0.046); cbar.set_label('Intensidad', rotation=270, color='white', labelpad=20)
    cbar.ax.yaxis.set_tick_params(color='white'); plt.setp(cbar.ax.get_yticklabels(), color='white')
    canvas_tkagg = FigureCanvasTkAgg(fig, master=parent_frame); canvas_tkagg.draw(); canvas_tkagg.get_tk_widget().pack(side="top", fill="both", expand=True)
    lidar_fig, lidar_ax, lidar_line, lidar_canvas_tkagg = fig, ax, line, canvas_tkagg
    return fig, ax, line, canvas_tkagg

def start_lidar_anim():
    global lidar_anim, lidar_inst, lidar_line, lidar_ax, lidar_fig
    if lidar_inst is None:
        lidar_inst = create_lidar_inst()
        if lidar_inst is None: print("LIDAR no iniciado."); return
    if lidar_ax: lidar_ax.set_title('Escaneo LIDAR', color='cyan', pad=20, fontsize=12); lidar_fig.canvas.draw_idle()
    if lidar_line is None or lidar_ax is None or lidar_fig is None: print("Gráfico LIDAR no configurado."); return
    if lidar_anim and lidar_anim.event_source.is_running(): print("LIDAR anim. ya en ejec.") ; return

    interval_ms = int(1000 / LIDAR_FRAME_RATE)
    lidar_anim = animation.FuncAnimation(lidar_fig, update_frame_lidar, interval=interval_ms, blit=True, cache_frame_data=False)
    lidar_fig.canvas.mpl_connect("close_event", stop_lidar_anim); lidar_fig.canvas.draw_idle()
    print("Anim. LIDAR iniciada.")

# This function now reads ALL data and updates the global `current_sensor_data` dictionary
def read_and_update_sensor_data():
    global ser, current_sensor_data, data_lock
    if not (ser and ser.is_open):
        return # Do nothing if serial is not open

    try:
        # Clear the input buffer to get the latest data
        # while ser.in_waiting: # Read all pending data
        #     ser.read() # Read and discard bytes

        line = ""
        # Read the *last* complete line available in the buffer
        # This loop ensures we process the most recent full line
        while ser.in_waiting > 0:
            current_line_bytes = ser.readline()
            try:
                current_line = current_line_bytes.decode('utf-8').strip()
                if current_line: # Only update if the line is not empty
                    line = current_line
            except UnicodeDecodeError:
                # print(f"Error de decodificación: {current_line_bytes}")
                pass # Skip malformed lines

        if line:
            # print(f"Python recibió línea: '{line}'") # Debugging line
            # Expected format: e1,e2,e3,e4,e5,mq2,ky024
            values = line.split(",")
            if len(values) == 7:
                try:
                    new_data = {
                        "e1": int(values[0]),
                        "e2": int(values[1]),
                        "e3": int(values[2]),
                        "e4": int(values[3]),
                        "e5": int(values[4]),
                        "mq2": int(values[5]),
                        "ky024": int(values[6])
                    }
                    with data_lock: # Protect shared data with a lock
                        current_sensor_data.update(new_data)
                    # print(f"Datos actualizados: {current_sensor_data}") # Debugging line
                except ValueError:
                    print(f"Error parseando valores a int: {line}. Saltando actualización.")
            # else:
                # print(f"Error: Longitud de valores incorrecta ({len(values)}). Línea: '{line}'. Saltando actualización.")
    except Exception as e:
        print(f"Error general en read_and_update_sensor_data: {e}");

# Dedicated thread for continuous serial reading
def serial_read_thread_func():
    while True: # Run indefinitely until the application closes
        # print("Hilo serial activo...") # Debugging line to confirm thread is running
        read_and_update_sensor_data()
        time.sleep(0.05) # Read approximately every 50ms (20 times per second)

# Start the serial reading thread at application start
# This thread is started once when the script runs.
# It should be a daemon thread so it exits when the main app exits.
serial_thread = threading.Thread(target=serial_read_thread_func, daemon=True)
serial_thread.start()


# Sensor reading functions now simply retrieve from the global dictionary
def get_sensor_value(key, default=0):
    with data_lock:
        return current_sensor_data.get(key, default)

def read_magnetometer():
    mag_val = get_sensor_value("ky024")
    print('valor del fakin magnetómetro; ', mag_val)
    # Assuming KY024 max value is 4095 from ESP32 analogReadResolution(12)
    # Adjust 4095.0 if your KY024 sensor range is different or you're scaling it.
    # The ESP32 code reads it directly. Scaling to 0-1000 range.
    return int((mag_val / 4095.0) * 1000) if mag_val is not None else 0

def read_mq7_sensor():
    gas_val = get_sensor_value("mq2")
    print('valor del fakin mq: ',gas_val)
    # The ESP32 code reads MQ2 directly. The -100 is an arbitrary offset from previous logic.
    # Adjust this logic based on actual MQ7 calibration if needed.
    return int(gas_val - 100) if gas_val is not None and gas_val > 100 else 0


def run_script(script):
    try: subprocess.run(["python3", script], check=True)
    except (subprocess.CalledProcessError, FileNotFoundError, Exception) as e: print(f"Error ejecutar {script}: {e}")

def exec_script_thread(script): threading.Thread(target=run_script, args=(script,), daemon=True).start()

def create_flippers_anim(canvas):
    colors = ["#00ffff", COLOR_THEME] # Flipper 1 (celeste), Flipper 2 (verde)
    flipper_lines = [None, None]
    flipper_circle = None

    def draw_flippers(e1_angle, e2_angle):
        nonlocal flipper_lines, flipper_circle
        canvas.delete("flipper") # Clear previous drawings

        # Draw the base rectangle (body of the robot)
        canvas.create_rectangle(40, 130, 250, 160, fill="#333333", outline=COLOR_THEME, width=3, tag="flipper")

        # Flipper 1 (celeste - controlled by e1)
        # Angles in Tkinter's canvas are usually standard (0 right, increases counter-clockwise)
        # But for physical flippers, you might want 0 at "straight" and then positive/negative angles
        # The -math.sin(angle_rad) is for standard cartesian-to-canvas y-axis flip.
        angle_rad_e1 = math.radians(e1_angle)
        end_x1 = FLIPPER_AXIS_X + FLIPPER_LENGTH * math.cos(angle_rad_e1)
        end_y1 = FLIPPER_AXIS_Y - FLIPPER_LENGTH * math.sin(angle_rad_e1) # Y-axis is inverted in canvas
        flipper_lines[0] = canvas.create_line(FLIPPER_AXIS_X, FLIPPER_AXIS_Y, end_x1, end_y1,
                                                fill=colors[0], width=FLIPPER_WIDTH, capstyle="round", tag="flipper")

        # Flipper 2 (verde - controlled by e2)
        angle_rad_e2 = math.radians(e2_angle)
        end_x2 = FLIPPER_AXIS_X + FLIPPER_LENGTH * math.cos(angle_rad_e2)
        end_y2 = FLIPPER_AXIS_Y - FLIPPER_LENGTH * math.sin(angle_rad_e2) # Y-axis is inverted in canvas
        flipper_lines[1] = canvas.create_line(FLIPPER_AXIS_X, FLIPPER_AXIS_Y, end_x2, end_y2,
                                                fill=colors[1], width=FLIPPER_WIDTH, capstyle="round", tag="flipper")

        # Draw the central circle (pivot point)
        flipper_circle = canvas.create_oval(FLIPPER_AXIS_X - 18, FLIPPER_AXIS_Y - 18,
                                             FLIPPER_AXIS_X + 18, FLIPPER_AXIS_Y + 18,
                                             fill="#444444", outline="white", width=2, tag="flipper")

    # This function will be called repeatedly to update flipper angles from encoder data
    def update_flipper_angles():
        with data_lock:
            e1_val = current_sensor_data["e1"]
            e2_val = current_sensor_data["e2"]

        # Map encoder values to a desired angle range (e.g., -FLIPPER_ANGLE_RANGE to +FLIPPER_ANGLE_RANGE degrees)
        # Assuming encoder range 0 to ENCODER_MAX_VALUE maps to -FLIPPER_ANGLE_RANGE to +FLIPPER_ANGLE_RANGE
        # If your encoders return negative values or different range, adjust mapping as needed.
        # This formula maps:
        # 0 -> -FLIPPER_ANGLE_RANGE
        # ENCODER_MAX_VALUE/2 -> 0
        # ENCODER_MAX_VALUE -> +FLIPPER_ANGLE_RANGE

        angle1 = ((e1_val / ENCODER_MAX_VALUE) * (2 * FLIPPER_ANGLE_RANGE)) - FLIPPER_ANGLE_RANGE
        angle2 = ((e2_val / ENCODER_MAX_VALUE) * (2 * FLIPPER_ANGLE_RANGE)) - FLIPPER_ANGLE_RANGE

        # Draw the flippers with the new angles
        draw_flippers(angle1, angle2)

        # Schedule the next update (e.g., every 50ms for 20 FPS on flippers)
        canvas.after(50, update_flipper_angles)

    # Start the flipper angle updates immediately after the GUI is ready
    # Initial call to set up the flippers and start the update loop
    update_flipper_angles()


def update_air_quality(label):
    label.configure(text=f"CO: {read_mq7_sensor()} PPM")
    label.after(1000, update_air_quality, label)

def update_magnetometer(label):
    label.configure(text=f"Mag: {read_magnetometer()}")
    label.after(1000, update_magnetometer, label)

def update_video_thread(cap, label, stop_event):
    while not stop_event.is_set() and cap.isOpened():
        ret, frame = cap.read()
        if ret:
            lbl_w, lbl_h = label.winfo_width(), label.winfo_height()
            if lbl_w > 1 and lbl_h > 1:
                fr_h, fr_w = frame.shape[0], frame.shape[1]; aspect = fr_w / fr_h
                new_w, new_h = (lbl_w, int(lbl_w / aspect)) if lbl_w / lbl_h > aspect else (int(lbl_h * aspect), lbl_h)
                frame = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_AREA)
            photo = ImageTk.PhotoImage(Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)))
            label.after(0, lambda ph=photo: label.configure(image=ph)); label.after(0, lambda ph=photo: setattr(label, 'image', ph))
        time.sleep(0.01)
    if cap.isOpened(): cap.release(); print(f"Hilo cámara {cap} terminado.")

def stop_all_camera_threads():
    global active_cam_caps, cam_stop_events
    for cap, stop_event in cam_stop_events.items():
        if not stop_event.is_set(): stop_event.set()
        if cap.isOpened(): cap.release(); print(f"Cámara {cap} liberada.")
    active_cam_caps.clear(); cam_stop_events.clear()

def setup_cameras(indices, labels):
    global active_cam_caps, cam_stop_events
    stop_all_camera_threads(); active_cam_caps.clear(); cam_stop_events.clear()
    for i, idx in enumerate(indices):
        cap = cv2.VideoCapture(idx)
        if cap.isOpened():
            print(f"Cámara {idx} abierta.")
            stop_event = threading.Event(); thread = threading.Thread(target=update_video_thread, args=(cap, labels[i], stop_event), daemon=True)
            thread.start(); active_cam_caps[cap] = thread; cam_stop_events[cap] = stop_event
        else: print(f"Error al abrir la cámara {idx}"); labels[i].configure(image=""); labels[i].image = None

def create_lidar_inst():
    global lidar_inst
    if not path.exists(LINUX_DEVICE_PATH): print(f"ERROR: Dispositivo no encontrado en {LINUX_DEVICE_PATH}. Conecte el LIDAR o verifique permisos."); return None
    try:
        lidar = RPLidar(port=LINUX_DEVICE_PATH, baudrate=LIDAR_BAUD_RATE, timeout=LIDAR_TIMEOUT)
        lidar.start_motor(); time.sleep(0.2); print(f"LIDAR conectado en {LINUX_DEVICE_PATH}")
        try: info = lidar.get_info(); print(f"Modelo: {info['model']}, Firmware: {info['firmware']}, Hardware: {info['hardware']}\nEstado: {lidar.get_health()[0]}")
        except RPLidarException as e: print(f"Info LIDAR no dis: {e}")
        lidar_inst = lidar; return lidar
    except (RPLidarException, OSError) as e: print(f"Error init LIDAR: {e}\nSolución: 1.USB 2.chmod 666 /dev/ttyUSB0 3.Reconectar"); return None

def update_frame_lidar(frame_num): # frame_num argument is expected by FuncAnimation, even if not used with a generator
    global lidar_line, lidar_ax, lidar_inst, lidar_fig
    if lidar_inst is None or lidar_line is None or lidar_ax is None: return ()
    try:
        if not hasattr(lidar_inst, 'iter_scans'): print("LIDAR inst. no es RPLidar."); return ()
        scan = next(lidar_inst.iter_scans(max_buf_meas=LIDAR_SCAN_BUFFER, min_len=3))
        if len(scan) == 0: return (lidar_line,)
        angles, distances, intensities = np.array([(np.radians(m[1]), m[2], m[0]) for m in scan], dtype=np.float32).T
        valid = (distances > 20) & (distances < LIDAR_D_MAX) & (intensities > 0)
        angles, distances, intensities = angles[valid], distances[valid], intensities[valid]
        lidar_line.set_offsets(np.column_stack((angles, distances))); lidar_line.set_array(intensities)
        lidar_ax.set_title(f'Escaneo LIDAR - {len(distances)} puntos', color='cyan', pad=20, fontsize=12); return (lidar_line,)
    except (RPLidarException, StopIteration, RuntimeError, TypeError, Exception) as e:
        print(f"Error update_frame_lidar: {e}")
        if lidar_ax: lidar_ax.set_title('LIDAR Desconectado/Error', color='red', pad=20, fontsize=12)
        if lidar_fig and lidar_fig.canvas: lidar_fig.canvas.draw_idle()
        return ()

def stop_lidar_anim():
    global lidar_anim, lidar_inst
    if lidar_anim: lidar_anim.event_source.stop(); lidar_anim = None; print("Anim. LIDAR detenida.")
    clean_shutdown_lidar()
    if lidar_ax: lidar_line.set_offsets([]); lidar_line.set_array([])
    if lidar_ax: lidar_ax.set_title('LIDAR DETENIDO', color='red', pad=20, fontsize=12)
    if lidar_fig and lidar_fig.canvas: lidar_fig.canvas.draw_idle()

def clean_shutdown_lidar():
    global lidar_inst
    try:
        if lidar_inst: lidar_inst.stop(); lidar_inst.stop_motor(); lidar_inst.disconnect()
        print("\nLIDAR desconectado correctamente"); lidar_inst = None
    except Exception as e: print(f"\nError apagado LIDAR: {e}")

def on_closing(root_window):
    print("Cerrando app..."); stop_lidar_anim(); stop_all_camera_threads()
    global ser;
    if ser and ser.is_open:
        try: ser.close(); print("Conexión serial ESP32 cerrada.")
        except Exception as e: print(f"Error cerrar serial ESP32: {e}")
    root_window.destroy()

def reset_cameras(labels_list, input_widget):
    print("Reseteando cámaras..."); stop_all_camera_threads()
    for lbl in labels_list: lbl.configure(image=""); lbl.image = None
    input_widget.delete(0, ctk.END); input_widget.insert(0, "")

def create_gui():
    ctk.set_appearance_mode("dark"); ctk.set_default_color_theme("dark-blue")
    root = ctk.CTk(); root.title("Unidad de Control del Robot de Rescate"); root.attributes("-fullscreen", True)
    root.protocol("WM_DELETE_WINDOW", lambda: on_closing(root))

    icon_path = path.join(path.dirname(__file__), "elbueno.ico")
    try: root.wm_iconphoto(True, ImageTk.PhotoImage(Image.open(icon_path)))
    except Exception as e: print(f"Error cargar ícono '{icon_path}': {e}")

    # Header Frame
    header_frame = ctk.CTkFrame(root, height=100, fg_color="black"); header_frame.pack(fill="x")
    ctk.CTkLabel(header_frame, text="Unidad de Control", font=("Arial", 45, "bold"), text_color="white").pack(side="left", padx=20)

    # Air Quality
    aq_frame = ctk.CTkFrame(header_frame, fg_color="#1e1e1e", corner_radius=15, border_width=2, border_color=COLOR_THEME)
    aq_frame.pack(side="left", padx=20, pady=10)
    air_quality_label = ctk.CTkLabel(aq_frame, text="CO: -- PPM", font=("Arial", 18, "bold"), text_color="white", width=150)
    air_quality_label.pack(padx=10, pady=5); update_air_quality(air_quality_label)

    # Magnetometer
    mag_frame = ctk.CTkFrame(header_frame, fg_color="#1e1e1e", corner_radius=15, border_width=2, border_color=COLOR_THEME)
    mag_frame.pack(side="left", padx=20, pady=10)
    magnetometer_label = ctk.CTkLabel(mag_frame, text="Mag: --", font=("Arial", 18, "bold"), text_color="white", width=150)
    magnetometer_label.pack(padx=10, pady=5); update_magnetometer(magnetometer_label)

    # Logo
    logo_path = path.join(path.dirname(__file__), "nixlogo.png")
    try:
        logo_img = Image.open(logo_path).resize((90, 90), Image.Resampling.LANCZOS)
        logo_lbl = ctk.CTkLabel(header_frame, image=ImageTk.PhotoImage(logo_img), text=""); logo_lbl.image = ImageTk.PhotoImage(logo_img)
        logo_lbl.pack(side="right", padx=20)
    except Exception as e: print(f"Error cargar logo '{logo_path}': {e}"); ctk.CTkLabel(header_frame, text="[Logo]", font=("Arial", 14), text_color="gray").pack(side="right", padx=20)

    # Button Frame
    button_frame = ctk.CTkFrame(root, height=50, fg_color="black"); button_frame.pack(fill="x")
    btn_data = [('Detectar Movimiento', "movementDetection.py"), ('Lectura QR', "qrDetector.py"), ("Cámara Térmica", "thermalCamera.py"),
                ("YOLOv10", "runyolov10.py"), ("SLAM", "slam.py"), ("Diagrama Cinemático", "kinematic_diagram.py")]
    for text, script in btn_data:
        ctk.CTkButton(button_frame, text=text, width=210, height=40, font=("Arial", 18, "bold"), text_color="black",
                      fg_color="white", hover_color="#cccccc", command=lambda s=script: exec_script_thread(s),
                      border_width=3, border_color=COLOR_THEME, corner_radius=17).pack(side="left", padx=20, pady=10)

    camera_indices = []
    def save_indices():
        indices_str = camera_input.get()
        if all(p.strip().isdigit() for p in indices_str.split(',')):
            camera_indices[:] = map(int, indices_str.split(",")); setup_cameras(camera_indices, camera_labels)
        else: print("Entrada de índices de cámara inválida. (ej: 0,1).")

    camera_input = ctk.CTkEntry(header_frame, width=300, placeholder_text="Índices de cámaras (ej: 0,1)", font=("Arial", 14), fg_color="white", text_color="black")
    reset_camera_btn = ctk.CTkButton(header_frame, text="Reset Cámaras", font=("Arial", 14, "bold"), text_color="black", fg_color="red", hover_color="#ff6666", command=lambda: reset_cameras(camera_labels, camera_input), corner_radius=15)
    reset_camera_btn.pack(side="right", padx=10)
    save_btn = ctk.CTkButton(header_frame, text="Configurar Cámaras", font=("Arial", 14, "bold"), text_color="black", fg_color=COLOR_THEME, hover_color="#cccccc", command=save_indices, corner_radius=15)
    save_btn.pack(side="right", padx=10)
    camera_input.pack(side="right", padx=10)

    # Main Content Frame
    main_frame = ctk.CTkFrame(root, fg_color="black"); main_frame.pack(expand=True, fill="both")
    main_frame.grid_rowconfigure(1, weight=1); main_frame.grid_columnconfigure(0, weight=3); main_frame.grid_columnconfigure(1, weight=1)

    # Camera Display
    camera_labels = []
    cam_main_frame = ctk.CTkFrame(main_frame, fg_color="#1e1e1e", corner_radius=17); cam_main_frame.grid(row=0, column=0, rowspan=2, padx=10, pady=10, sticky="nsew")
    cam_lbl_main = ctk.CTkLabel(cam_main_frame, text="Cámara Principal", font=("Arial", 18, "bold"), text_color="white"); cam_lbl_main.pack(expand=True, fill="both")
    camera_labels.append(cam_lbl_main)

    # Widgets Frame (Flippers & LIDAR)
    widget_frame = ctk.CTkFrame(main_frame, fg_color="black", corner_radius=17); widget_frame.grid(row=0, column=1, rowspan=2, padx=10, pady=10, sticky="nsew")
    widget_frame.grid_rowconfigure((0, 1), weight=1); widget_frame.grid_columnconfigure(0, weight=1)

    # Flippers Widget
    flipper_frame = ctk.CTkFrame(widget_frame, fg_color="#1e1e1e", corner_radius=17, border_width=2, border_color=COLOR_THEME); flipper_frame.grid(row=0, column=0, pady=10, padx=10, sticky="nsew")
    ctk.CTkLabel(flipper_frame, text="Estado Flippers", font=("Arial", 18, "bold"), text_color="white").pack(pady=5)
    flipper_canvas = ctk.CTkCanvas(flipper_frame, bg="#1e1e1e", highlightthickness=0); flipper_canvas.pack(expand=True, fill="both")
    # Initialize flippers controlled by encoders (calls update_flipper_angles() internally)
    root.after(100, lambda: create_flippers_anim(flipper_canvas))

    # LIDAR Widget
    lidar_frame = ctk.CTkFrame(widget_frame, fg_color="#1e1e1e", corner_radius=17, border_width=2, border_color=COLOR_THEME); lidar_frame.grid(row=1, column=0, pady=10, padx=10, sticky="nsew")
    lidar_ctrl_frame = ctk.CTkFrame(lidar_frame, fg_color="#1e1e1e"); lidar_ctrl_frame.pack(pady=5)
    ctk.CTkLabel(lidar_ctrl_frame, text="Visualización LIDAR", font=("Arial", 18, "bold"), text_color="white").pack(side="left", padx=10)

    stop_lidar_btn = ctk.CTkButton(lidar_ctrl_frame, text="Detener LIDAR", font=("Arial", 16, "bold"), text_color="black", fg_color="red", hover_color="#ff6666", command=stop_lidar_anim, corner_radius=15)
    stop_lidar_btn.pack(side="left", padx=5)
    start_lidar_btn = ctk.CTkButton(lidar_ctrl_frame, text="Iniciar LIDAR", font=("Arial", 16, "bold"), text_color="black", fg_color="#00FF00", hover_color="#00AA00", command=start_lidar_anim, corner_radius=15)
    start_lidar_btn.pack(side="left", padx=5)

    config_lidar_plot(lidar_frame); root.after(100, start_lidar_anim)

    root.mainloop()

if __name__ == "__main__": create_gui()