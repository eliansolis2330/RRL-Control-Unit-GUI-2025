import customtkinter as ctk, cv2, math, threading, subprocess, serial, sys, json
import matplotlib.pyplot as plt, numpy as np
from PIL import Image, ImageTk
from os import path
import time

# --- Configuration Constants ---
COLOR_THEME = '#12fe35'
SERIAL_PORT = "/dev/ttyHTS1"
BAUD_RATE = 9600

FLIPPER_AXIS_X, FLIPPER_AXIS_Y, FLIPPER_LENGTH, FLIPPER_WIDTH = 180, 145, 90, 25
ENCODER_MAX_VALUE = 360
FLIPPER_ANGLE_RANGE = 80

ENCODER_DATA_FILE = "encoder_arm_data.json"

# --- New Camera Constant ---
CAMERA_DISPLAY_FPS = 15 # Max frames per second to update the GUI label for cameras

# --- Global Variables ---
active_cam_caps, cam_stop_events = {}, {}
ser = None
current_sensor_data = {"e1": 0, "e2": 0, "e3": 0, "e4": 0, "e5": 0, "mq2": 0, "ky024": 0}
data_lock = threading.Lock()

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
    print(f"Conexión ESP32 establecida en {SERIAL_PORT} @ {BAUD_RATE} baudios.")
except (serial.SerialException, Exception) as e:
    print(f"Error serial al conectar con ESP32: {e}. Continuando sin conexión serial.")

# --- Helper Functions ---
def read_and_update_sensor_data():
    global ser, current_sensor_data, data_lock
    if not (ser and ser.is_open): return

    try:
        line = ""
        while ser.in_waiting > 0:
            current_line_bytes = ser.readline()
            try: current_line = current_line_bytes.decode('utf-8').strip()
            except UnicodeDecodeError: pass
            if current_line: line = current_line

        if line:
            values = line.split(",")
            if len(values) == 7:
                try:
                    new_data = {
                        "e1": int(values[0]), "e2": int(values[1]), "e3": int(values[2]),
                        "e4": int(values[3]), "e5": int(values[4]),
                        "mq2": int(values[5]), "ky024": int(values[6])
                    }
                    with data_lock: current_sensor_data.update(new_data)
                except ValueError: pass
    except Exception as e: pass

def write_arm_encoders_to_file():
    global current_sensor_data, data_lock
    with data_lock:
        arm_encoders = {"e3": current_sensor_data["e3"], "e4": current_sensor_data["e4"], "e5": current_sensor_data["e5"]}
    try:
        with open(ENCODER_DATA_FILE, 'w') as f: json.dump(arm_encoders, f)
    except Exception as e: pass

def serial_read_thread_func():
    while True:
        read_and_update_sensor_data()
        write_arm_encoders_to_file()
        time.sleep(0.05)

serial_thread = threading.Thread(target=serial_read_thread_func, daemon=True)
serial_thread.start()

def get_sensor_value(key, default=0):
    with data_lock: return current_sensor_data.get(key, default)

def read_magnetometer():
    mag_val = get_sensor_value("ky024")
    return int((mag_val / 4095.0) * 1000) if mag_val is not None else 0

def read_mq7_sensor():
    gas_val = get_sensor_value("mq2")
    return int(gas_val - 100) if gas_val is not None and gas_val > 100 else 0

def run_script(script):
    try: subprocess.run(["python3", script], check=True)
    except (subprocess.CalledProcessError, FileNotFoundError, Exception) as e: print(f"Error ejecutar {script}: {e}")

def exec_script_thread(script): threading.Thread(target=run_script, args=(script,), daemon=True).start()

def create_flippers_anim(canvas):
    colors = ["#00ffff", COLOR_THEME]
    def draw_flippers(e1_angle, e2_angle):
        canvas.delete("flipper")
        canvas.create_rectangle(40, 130, 250, 160, fill="#333333", outline=COLOR_THEME, width=3, tag="flipper")
        angle_rad_e1 = math.radians(e1_angle)
        end_x1 = FLIPPER_AXIS_X + FLIPPER_LENGTH * math.cos(angle_rad_e1)
        end_y1 = FLIPPER_AXIS_Y - FLIPPER_LENGTH * math.sin(angle_rad_e1)
        canvas.create_line(FLIPPER_AXIS_X, FLIPPER_AXIS_Y, end_x1, end_y1, fill=colors[0], width=FLIPPER_WIDTH, capstyle="round", tag="flipper")
        angle_rad_e2 = math.radians(e2_angle)
        end_x2 = FLIPPER_AXIS_X + FLIPPER_LENGTH * math.cos(angle_rad_e2)
        end_y2 = FLIPPER_AXIS_Y - FLIPPER_LENGTH * math.sin(angle_rad_e2)
        canvas.create_line(FLIPPER_AXIS_X, FLIPPER_AXIS_Y, end_x2, end_y2, fill=colors[1], width=FLIPPER_WIDTH, capstyle="round", tag="flipper")
        canvas.create_oval(FLIPPER_AXIS_X - 18, FLIPPER_AXIS_Y - 18, FLIPPER_AXIS_X + 18, FLIPPER_AXIS_Y + 18, fill="#444444", outline="white", width=2, tag="flipper")

    def update_flipper_angles():
        with data_lock:
            e1_val = current_sensor_data["e1"]
            e2_val = current_sensor_data["e2"]
        angle1 = ((e1_val / ENCODER_MAX_VALUE) * (2 * FLIPPER_ANGLE_RANGE)) - FLIPPER_ANGLE_RANGE
        angle2 = ((e2_val / ENCODER_MAX_VALUE) * (2 * FLIPPER_ANGLE_RANGE)) - FLIPPER_ANGLE_RANGE
        draw_flippers(angle1, angle2)
        canvas.after(50, update_flipper_angles)
    update_flipper_angles()

def update_air_quality(label):
    label.configure(text=f"CO: {read_mq7_sensor()} PPM")
    label.after(1000, update_air_quality, label)

def update_magnetometer(label):
    label.configure(text=f"Mag: {read_magnetometer()}")
    label.after(1000, update_magnetometer, label)

def update_video_thread(cap, label, stop_event):
    """Thread function to continuously read and display camera frames."""
    last_update_time = time.time()
    min_interval = 1.0 / CAMERA_DISPLAY_FPS # Calculate minimum time between updates

    while not stop_event.is_set() and cap.isOpened():
        ret, frame = cap.read()
        if ret:
            current_time = time.time()
            if current_time - last_update_time >= min_interval:
                # Resize frame to fit label while maintaining aspect ratio
                lbl_w, lbl_h = label.winfo_width(), label.winfo_height()
                if lbl_w > 1 and lbl_h > 1: # Ensure label has valid dimensions
                    fr_h, fr_w = frame.shape[0], frame.shape[1]; aspect = fr_w / fr_h
                    new_w, new_h = (lbl_w, int(lbl_w / aspect)) if lbl_w / lbl_h > aspect else (int(lbl_h * aspect), lbl_h)
                    frame = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_AREA)
                photo = ImageTk.PhotoImage(Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)))
                # Schedule update on the main thread
                label.after(0, lambda ph=photo: (label.configure(image=ph), setattr(label, 'image', ph)))
                last_update_time = current_time
        time.sleep(0.001) # Small sleep to yield CPU, but min_interval controls effective FPS
    if cap.isOpened(): cap.release() # Release camera resource when thread stops


def stop_all_camera_threads():
    global active_cam_caps, cam_stop_events
    for cap, stop_event in cam_stop_events.items():
        if not stop_event.is_set(): stop_event.set() # Signal thread to stop
        if cap.isOpened():
            cap.release() # Release OpenCV capture object
    active_cam_caps.clear()
    cam_stop_events.clear()


def setup_cameras(main_cam_indices_str, secondary_cam_index_str, main_label, secondary_label, is_secondary_active):
    """
    Sets up and starts video capture for specified camera indices.
    main_cam_indices_str: string of comma-separated indices for the main camera display (e.g., "0,1")
    secondary_cam_index_str: string of a single index for the secondary camera
    main_label: CTkLabel for the main camera display
    secondary_label: CTkLabel for the secondary camera display
    is_secondary_active: Boolean indicating if the secondary camera should be active
    """
    global active_cam_caps, cam_stop_events
    stop_all_camera_threads() # Stop existing cameras before starting new ones

    # Process main camera indices
    main_indices = []
    if main_cam_indices_str:
        try:
            main_indices = list(map(int, main_cam_indices_str.split(',')))
        except ValueError:
            print(f"Advertencia: Índice de cámara principal inválido '{main_cam_indices_str}'. Ignorando.")
            main_indices = []

    # Process secondary camera index
    sec_idx = -1
    if secondary_cam_index_str:
        try:
            sec_idx = int(secondary_cam_index_str.strip())
        except ValueError:
            print(f"Advertencia: Índice de cámara secundaria inválido '{secondary_cam_index_str}'. Ignorando.")

    # Start main camera(s)
    if main_indices:
        cap = cv2.VideoCapture(main_indices[0]) # Use the first index for the main display
        if cap.isOpened():
            stop_event = threading.Event()
            thread = threading.Thread(target=update_video_thread, args=(cap, main_label, stop_event), daemon=True)
            thread.start()
            active_cam_caps[cap] = thread
            cam_stop_events[cap] = stop_event
            print(f"Cámara principal {main_indices[0]} iniciada con éxito.")
        else:
            main_label.configure(image="")
            main_label.image = None
            print(f"Error: No se pudo abrir la cámara principal {main_indices[0]}.")
    else:
        main_label.configure(image="")
        main_label.image = None


    # Start secondary camera if active and valid index
    if is_secondary_active and sec_idx != -1:
        cap_sec = cv2.VideoCapture(sec_idx)
        if cap_sec.isOpened():
            stop_event_sec = threading.Event()
            thread_sec = threading.Thread(target=update_video_thread, args=(cap_sec, secondary_label, stop_event_sec), daemon=True)
            thread_sec.start()
            active_cam_caps[cap_sec] = thread_sec
            cam_stop_events[cap_sec] = stop_event_sec
            print(f"Cámara secundaria {sec_idx} iniciada con éxito.")
        else:
            secondary_label.configure(image="")
            secondary_label.image = None
            print(f"Error: No se pudo abrir la cámara secundaria {sec_idx}.")
    else:
        secondary_label.configure(image="")
        secondary_label.image = None # Clear image if not active or invalid index


def on_closing(root_window):
    stop_all_camera_threads()
    global ser;
    if ser and ser.is_open:
        try: ser.close()
        except Exception as e: pass
    root_window.destroy()

def reset_cameras(main_input_widget, secondary_input_widget, main_label, secondary_label, sec_cam_switch_var):
    """
    Resets all camera inputs, stops camera feeds, and clears labels.
    sec_cam_switch_var: The BooleanVar associated with the secondary camera switch.
    """
    stop_all_camera_threads() # This stops all threads and releases caps
    main_label.configure(image=""); main_label.image = None # Clear image from label
    secondary_label.configure(image=""); secondary_label.image = None # Clear image from label
    main_input_widget.delete(0, ctk.END); main_input_widget.insert(0, "")
    secondary_input_widget.delete(0, ctk.END); secondary_input_widget.insert(0, "")
    sec_cam_switch_var.set(False) # Set the BooleanVar to False, deselecting the switch
    print("Cámaras reiniciadas y detenidas.")


def create_gui():
    # --- Ajuste de escalado DPI para CustomTkinter ---
    # Experimenta con estos valores (por ejemplo, 1.5, 2.0, 2.5) hasta que la interfaz se vea bien
    # en tu monitor de 12 pulgadas en la Jetson Orin Nano.
    # Un valor de 2.0 significa que los widgets serán el doble de grandes de su tamaño base.
    SCALING_FACTOR = 1.5 # Puedes ajustar este valor
    ctk.set_widget_scaling(SCALING_FACTOR)
    ctk.set_window_scaling(SCALING_FACTOR)
    # La línea `ctk.set_spacing_scaling(SCALING_FACTOR)` fue eliminada ya que no existe.
    # --- Fin del ajuste de escalado ---


    ctk.set_appearance_mode("dark"); ctk.set_default_color_theme("dark-blue")
    root = ctk.CTk(); root.title("Unidad de Control del Robot de Rescate"); root.attributes("-fullscreen", True)
    root.protocol("WM_DELETE_WINDOW", lambda: on_closing(root))

    # Mueve la declaración de secondary_camera_active aquí, después de que 'root' ha sido inicializado.
    secondary_camera_active = ctk.BooleanVar(value=False) # Controla el switch ON/OFF

    icon_path = path.join(path.dirname(__file__), "elbueno.ico")
    try: root.wm_iconphoto(True, ImageTk.PhotoImage(Image.open(icon_path)))
    except Exception as e: print(f"Error cargando el icono: {e}")

    header_frame = ctk.CTkFrame(root, height=100, fg_color="black"); header_frame.pack(fill="x")
    ctk.CTkLabel(header_frame, text="Unidad de Control", font=("Arial", 45, "bold"), text_color="white").pack(side="left", padx=20)

    aq_frame = ctk.CTkFrame(header_frame, fg_color="#1e1e1e", corner_radius=15, border_width=2, border_color=COLOR_THEME)
    aq_frame.pack(side="left", padx=20, pady=10)
    air_quality_label = ctk.CTkLabel(aq_frame, text="CO: -- PPM", font=("Arial", 18, "bold"), text_color="white", width=150)
    air_quality_label.pack(padx=10, pady=5); update_air_quality(air_quality_label)

    mag_frame = ctk.CTkFrame(header_frame, fg_color="#1e1e1e", corner_radius=15, border_width=2, border_color=COLOR_THEME)
    mag_frame.pack(side="left", padx=20, pady=10)
    magnetometer_label = ctk.CTkLabel(mag_frame, text="Mag: --", font=("Arial", 18, "bold"), text_color="white", width=150)
    magnetometer_label.pack(padx=10, pady=5); update_magnetometer(magnetometer_label)

    logo_path = path.join(path.dirname(__file__), "nixlogo.png")
    try:
        logo_img = Image.open(logo_path).resize((90, 90), Image.Resampling.LANCZOS)
        logo_lbl = ctk.CTkLabel(header_frame, image=ImageTk.PhotoImage(logo_img), text=""); logo_lbl.image = ImageTk.PhotoImage(logo_img)
        logo_lbl.pack(side="right", padx=20)
    except Exception as e: ctk.CTkLabel(header_frame, text="[Logo]", font=("Arial", 14), text_color="gray").pack(side="right", padx=20)

    button_frame = ctk.CTkFrame(root, height=50, fg_color="black"); button_frame.pack(fill="x")
    btn_data = [('Detectar Movimiento', "movementDetection.py"), ('Lectura QR', "qrDetector.py"), ("Cámara Térmica", "thermalCamera.py"),
                ("YOLOv10", "runyolov10.py"), ("SLAM", "slam.py"), ("Diagrama Cinemático", "kinematic_diagram.py")]
    for text, script in btn_data:
        ctk.CTkButton(button_frame, text=text, width=210, height=40, font=("Arial", 18, "bold"), text_color="black",
                      fg_color="white", hover_color="#cccccc", command=lambda s=script: exec_script_thread(s),
                      border_width=3, border_color=COLOR_THEME, corner_radius=17).pack(side="left", padx=20, pady=10)

    # --- Camera Configuration Widgets ---
    camera_config_frame = ctk.CTkFrame(header_frame, fg_color="black")
    camera_config_frame.pack(side="right", padx=10)

    # Frame para los inputs y switches de las cámaras
    camera_inputs_frame = ctk.CTkFrame(camera_config_frame, fg_color="black")
    camera_inputs_frame.pack(anchor="e", pady=5)

    # Columna izquierda para Cámara Principal
    main_cam_input_col = ctk.CTkFrame(camera_inputs_frame, fg_color="black")
    main_cam_input_col.pack(side="left", padx=5)
    ctk.CTkLabel(main_cam_input_col, text="Cámara Principal (Índices, ej: 0,1):", font=("Arial", 12), text_color="white").pack(anchor="w")
    main_camera_input = ctk.CTkEntry(main_cam_input_col, width=180, placeholder_text="ej: 0", font=("Arial", 14), fg_color="white", text_color="black")
    main_camera_input.pack(anchor="w")

    # Columna derecha para Cámara Secundaria
    sec_cam_input_col = ctk.CTkFrame(camera_inputs_frame, fg_color="black")
    sec_cam_input_col.pack(side="left", padx=5)
    ctk.CTkLabel(sec_cam_input_col, text="Cámara Secundaria (Índice):", font=("Arial", 12), text_color="white").pack(anchor="w")
    secondary_camera_input = ctk.CTkEntry(sec_cam_input_col, width=180, placeholder_text="ej: 1", font=("Arial", 14), fg_color="white", text_color="black")
    secondary_camera_input.pack(anchor="w")

    # Secondary Camera ON/OFF Switch (debajo del input de la secundaria)
    secondary_camera_switch = ctk.CTkSwitch(sec_cam_input_col,
                                            text="Cámara Secundaria ON/OFF",
                                            command=lambda: configure_all_cameras(
                                                main_camera_input.get(),
                                                secondary_camera_input.get(),
                                                main_cam_label,
                                                sec_cam_label,
                                                secondary_camera_active.get()
                                            ),
                                            variable=secondary_camera_active,
                                            onvalue=True, offvalue=False,
                                            button_color=COLOR_THEME,
                                            fg_color="gray",
                                            progress_color="#00AA00")
    secondary_camera_switch.pack(anchor="w", pady=(5,0))

    def configure_all_cameras(main_indices_str, secondary_index_str, main_lbl, sec_lbl, sec_active_state):
        setup_cameras(main_indices_str, secondary_index_str, main_lbl, sec_lbl, sec_active_state)


    # Control buttons for cameras
    cam_btns_frame = ctk.CTkFrame(camera_config_frame, fg_color="black")
    cam_btns_frame.pack(anchor="e", pady=(5,0))

    reset_camera_btn = ctk.CTkButton(cam_btns_frame, text="Reset Cámaras", font=("Arial", 14, "bold"), text_color="black", fg_color="red", hover_color="#ff6666",
                                     command=lambda: reset_cameras(main_camera_input, secondary_camera_input, main_cam_label, sec_cam_label, secondary_camera_active), corner_radius=15)
    reset_camera_btn.pack(side="left", padx=5)

    save_btn = ctk.CTkButton(cam_btns_frame, text="Configurar Cámaras", font=("Arial", 14, "bold"), text_color="black", fg_color=COLOR_THEME, hover_color="#cccccc",
                             command=lambda: configure_all_cameras(
                                 main_camera_input.get(),
                                 secondary_camera_input.get(),
                                 main_cam_label,
                                 sec_cam_label,
                                 secondary_camera_active.get()
                             ), corner_radius=15)
    save_btn.pack(side="left", padx=5)


    # --- Main Content Frame ---
    main_frame = ctk.CTkFrame(root, fg_color="black"); main_frame.pack(expand=True, fill="both")
    main_frame.grid_rowconfigure(0, weight=1); main_frame.grid_rowconfigure(1, weight=1)
    main_frame.grid_columnconfigure(0, weight=3); main_frame.grid_columnconfigure(1, weight=1)

    # Main Camera Display Frame
    main_cam_frame = ctk.CTkFrame(main_frame, fg_color="#1e1e1e", corner_radius=17); main_cam_frame.grid(row=0, column=0, rowspan=2, padx=10, pady=10, sticky="nsew")
    main_cam_label = ctk.CTkLabel(main_cam_frame, text="Cámara Principal", font=("Arial", 18, "bold"), text_color="white"); main_cam_label.pack(expand=True, fill="both")

    # Widget Frame (Flippers and Secondary Camera)
    widget_frame = ctk.CTkFrame(main_frame, fg_color="black", corner_radius=17); widget_frame.grid(row=0, column=1, rowspan=2, padx=10, pady=10, sticky="nsew")
    widget_frame.grid_rowconfigure(0, weight=1); widget_frame.grid_rowconfigure(1, weight=1)
    widget_frame.grid_columnconfigure(0, weight=1)

    # Flippers Animation Frame
    flipper_frame = ctk.CTkFrame(widget_frame, fg_color="#1e1e1e", corner_radius=17, border_width=2, border_color=COLOR_THEME); flipper_frame.grid(row=0, column=0, pady=10, padx=10, sticky="nsew")
    ctk.CTkLabel(flipper_frame, text="Estado Flippers", font=("Arial", 18, "bold"), text_color="white").pack(pady=5)
    flipper_canvas = ctk.CTkCanvas(flipper_frame, bg="#1e1e1e", highlightthickness=0); flipper_canvas.pack(expand=True, fill="both")
    root.after(100, lambda: create_flippers_anim(flipper_canvas))

    # Secondary Camera Frame (Replacing LIDAR)
    sec_cam_frame = ctk.CTkFrame(widget_frame, fg_color="#1e1e1e", corner_radius=17, border_width=2, border_color=COLOR_THEME); sec_cam_frame.grid(row=1, column=0, pady=10, padx=10, sticky="nsew")
    sec_cam_label = ctk.CTkLabel(sec_cam_frame, text="Cámara Secundaria", font=("Arial", 18, "bold"), text_color="white"); sec_cam_label.pack(expand=True, fill="both")

    # Initialize cameras on startup
    root.after(100, lambda: configure_all_cameras(
        main_camera_input.get(),
        secondary_camera_input.get(),
        main_cam_label,
        sec_cam_label,
        secondary_camera_active.get()
    ))

    root.mainloop()

if __name__ == "__main__": create_gui()
