import customtkinter as ctk, cv2, math, threading, subprocess, serial, sys, json
import matplotlib.animation as animation, matplotlib.pyplot as plt, numpy as np
from PIL import Image, ImageTk
from rplidar import RPLidar, RPLidarException
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from os import path
import time

# --- Configuration Constants (Moved to top for easy access and modification) ---
COLOR_THEME = '#12fe35'
SERIAL_PORT = "/dev/ttyTHS0"
BAUD_RATE = 112500

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

# --- Global Variables (for shared state, minimized) ---
lidar_inst, lidar_anim, lidar_line, lidar_ax, lidar_fig, lidar_canvas_tkagg = [None] * 6
active_cam_caps, cam_stop_events = {}, {}
ser = None # Serial connection for ESP32

# Attempt serial connection
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
    print("Conexión ESP32 establecida")
except (serial.SerialException, Exception) as e:
    print(f"Error serial: {e}. Continuando sin conexión.")

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

def read_sensors():
    if not (ser and ser.is_open): return None, None
    try:
        line = "";
        while ser.in_waiting > 0: line = ser.readline().decode('utf-8').strip()
        return (int(v) for v in line.split(",")) if len(line.split(",")) == 2 else (None, None)
    except Exception as e: print(f"Err lectura sen: {e}"); return None, None

def read_magnetometer():
    _, mag = read_sensors(); return int((mag / 4095.0) * 1000) if mag is not None else 0

def read_mq7_sensor():
    gas, _ = read_sensors(); return int(gas - 100) if gas is not None and gas > 100 else 0

def run_script(script):
    try: subprocess.run(["python3", script], check=True)
    except (subprocess.CalledProcessError, FileNotFoundError, Exception) as e: print(f"Error ejecutar {script}: {e}")

def exec_script_thread(script): threading.Thread(target=run_script, args=(script,), daemon=True).start()

def create_flippers_anim(canvas):
    colors = ["#00ffff", COLOR_THEME]; flipper_angles, direction = [0, 0], 1
    def draw_rounded_rect(x1, y1, x2, y2, r=30, **kwargs):
        pts = [x1+r, y1, x2-r, y1, x2, y1, x2, y1+r, x2, y2-r, x2, y2, x2-r, y2, x1+r, y2, x1, y2, x1, y2-r, x1, y1+r, x1, y1]
        return canvas.create_polygon(pts, **kwargs, smooth=True, tag="flipper")
    def draw_flippers():
        canvas.delete("flipper"); draw_rounded_rect(40, 130, 250, 160, r=25, fill="#333333", outline=COLOR_THEME, width=3)
        for i in range(2):
            angle_rad = math.radians(flipper_angles[i]); end_x = FLIPPER_AXIS_X + FLIPPER_LENGTH * math.cos(angle_rad)
            end_y = FLIPPER_AXIS_Y - FLIPPER_LENGTH * math.sin(angle_rad)
            canvas.create_line(FLIPPER_AXIS_X, FLIPPER_AXIS_Y, end_x, end_y, fill=colors[i], width=FLIPPER_WIDTH, capstyle="round", tag="flipper")
            if i == 0: canvas.create_oval(FLIPPER_AXIS_X-18, FLIPPER_AXIS_Y-18, FLIPPER_AXIS_X+18, FLIPPER_AXIS_Y+18, fill="#444444", outline="white", width=2, tag="flipper")
    def update_flippers():
        nonlocal flipper_angles, direction; flipper_angles[0] += direction * 1; flipper_angles[1] -= direction * 1
        if flipper_angles[0] > 80 or flipper_angles[0] < -80: direction *= -1
        draw_flippers(); canvas.after(40, update_flippers)
    update_flippers()

def update_air_quality(label): label.configure(text=f"CO: {read_mq7_sensor()} PPM"); label.after(1000, update_air_quality, label)

def update_magnetometer(label): label.configure(text=f"Mag: {read_magnetometer()}"); label.after(1000, update_magnetometer, label)

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

def update_frame_lidar(frame_num): # Use frame_num if generator is used, otherwise it's just a dummy argument
    global lidar_line, lidar_ax, lidar_inst, lidar_fig
    if lidar_inst is None or lidar_line is None or lidar_ax is None: return () # Return empty tuple for blit=True
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
    btn_data = [('Detectar Movimiento', "movementDetection.py"), ("Cámara Térmica", "thermalCamera.py"),
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