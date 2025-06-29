import serial, json, threading
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

esp32 = serial.Serial('/dev/ttyUSB0', 9600, timeout=0)

latest = {"encoder1":0, "encoder2":0, "encoder3":0}

def reader_thread():
    """Lee sin parar el serial y actualiza `latest`."""
    buffer = ""
    while True:
        chunk = esp32.read(128).decode('utf-8', errors='ignore')
        if not chunk:
            continue
        buffer += chunk
        if '\n' in buffer:
            line, buffer = buffer.split('\n', 1)
            try:
                data = json.loads(line.strip())
                latest.update(data)
            except json.JSONDecodeError:
                pass

# Arranca el hilo como demonio
t = threading.Thread(target=reader_thread, daemon=True)
t.start()

# Parámetros del brazo
L1, L2, L3 = 2.0, 1.5, 1.0

def enc_to_rad(x):
    return (x/1024.0)*2*np.pi

# Prepara figura y artistas
fig, ax = plt.subplots(figsize=(6,6))
ax.set_aspect('equal','box')
ax.set_xlim(-5,5); ax.set_ylim(-1,5)
lines = [ax.plot([],[], '-', lw=6, solid_capstyle='round')[0] for _ in range(3)]
joints = ax.plot([], [], 'o', ms=10, mec='k', mfc='w', mew=2)[0]

def get_pts():
    θ1 = enc_to_rad(latest["encoder1"])
    θ2 = enc_to_rad(latest["encoder2"])
    θ3 = enc_to_rad(latest["encoder3"])
    x1,y1 = L1*np.cos(θ1), L1*np.sin(θ1)
    x2,y2 = x1+L2*np.cos(θ1+θ2), y1+L2*np.sin(θ1+θ2)
    x3,y3 = x2+L3*np.cos(θ1+θ2+θ3), y2+L3*np.sin(θ1+θ2+θ3)
    return [(0,0),(x1,y1),(x2,y2),(x3,y3)]

def update(frame):
    pts = get_pts()
    xs, ys = zip(*pts)
    for i, ln in enumerate(lines):
        ln.set_data([xs[i], xs[i+1]], [ys[i], ys[i+1]])
    joints.set_data(xs, ys)
    return lines + [joints]

# Interval más corto para más FPS (e.g. 20 ms → 50 FPS)
ani = FuncAnimation(fig, update, interval=20, blit=True)
plt.show()
esp32.close()
