import os
import time
import numpy as np
import customtkinter as ctk
from PIL import Image, ImageDraw
from rplidar import RPLidar, RPLidarException

# Configuración de CustomTkinter
ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

# Configuración LIDAR
BAUD_RATE = 256000
TIMEOUT = 0.05
D_MAX = 5000
I_MIN = 0
I_MAX = 150
SCAN_BUFFER = 25000
LINUX_DEVICE_PATH = '/dev/ttyUSB0'
REFRESH_RATE = 30  # ms


class LidarVisualizationApp(ctk.CTk):
    def __init__(self):
        super().__init__()

        self.title("Visualización LIDAR M2A12 - CustomTkinter")
        self.geometry("800x800")

        # Configurar grid
        self.grid_rowconfigure(0, weight=1)
        self.grid_columnconfigure(0, weight=1)

        # Canvas para visualización
        self.canvas = ctk.CTkCanvas(self, bg="black", highlightthickness=0)
        self.canvas.grid(row=0, column=0, sticky="nsew")

        # Panel de control
        self.control_frame = ctk.CTkFrame(self)
        self.control_frame.grid(row=1, column=0, sticky="ew", padx=5, pady=5)

        # Botones
        self.start_button = ctk.CTkButton(
            self.control_frame, text="Iniciar", command=self.start_lidar
        )
        self.start_button.pack(side="left", padx=5)

        self.stop_button = ctk.CTkButton(
            self.control_frame, text="Detener", command=self.stop_lidar, state="disabled"
        )
        self.stop_button.pack(side="left", padx=5)

        # Etiqueta de información
        self.info_label = ctk.CTkLabel(
            self.control_frame, text="Estado: Desconectado", text_color="yellow"
        )
        self.info_label.pack(side="left", padx=10)

        # Variables
        self.lidar = None
        self.running = False
        self.image_width = 800
        self.image_height = 800
        self.center_x = self.image_width // 2
        self.center_y = self.image_height // 2
        self.scale_factor = self.image_width / (2 * D_MAX)

        # Imagen para el radar
        self.radar_image = Image.new("RGB", (self.image_width, self.image_height), "black")
        self.radar_photo = ctk.CTkImage(light_image=self.radar_image, size=(self.image_width, self.image_height))
        self.radar_label = ctk.CTkLabel(self.canvas, image=self.radar_photo, text="")
        self.radar_label.pack()

        # Dibujar guías del radar
        self.draw_radar_guides()

    def draw_radar_guides(self):
        """Dibuja las guías del radar en la imagen"""
        draw = ImageDraw.Draw(self.radar_image)

        # Círculos concéntricos
        for r in range(1000, D_MAX, 1000):
            scaled_r = r * self.scale_factor
            draw.ellipse(
                [
                    (self.center_x - scaled_r, self.center_y - scaled_r),
                    (self.center_x + scaled_r, self.center_y + scaled_r)
                ],
                outline="#00FF00",
                width=1
            )

        # Líneas de ángulo
        for angle in range(0, 360, 30):
            rad = np.radians(angle)
            end_x = self.center_x + D_MAX * self.scale_factor * np.sin(rad)
            end_y = self.center_y - D_MAX * self.scale_factor * np.cos(rad)
            draw.line(
                [(self.center_x, self.center_y), (end_x, end_y)],
                fill="#00FF00",
                width=1
            )

        # Actualizar imagen
        self.update_radar_image()

    def update_radar_image(self):
        """Actualiza la imagen del radar en el canvas"""
        self.radar_photo.configure(light_image=self.radar_image)
        self.radar_label.configure(image=self.radar_photo)

    def start_lidar(self):
        """Inicia la conexión con el LIDAR y el escaneo"""
        if not os.path.exists(LINUX_DEVICE_PATH):
            self.info_label.configure(text="ERROR: Dispositivo no encontrado", text_color="red")
            return

        try:
            self.lidar = RPLidar(port=LINUX_DEVICE_PATH, baudrate=BAUD_RATE, timeout=TIMEOUT)
            self.lidar.start_motor()
            time.sleep(0.2)

            info = self.lidar.get_info()
            health = self.lidar.get_health()
            status_text = f"Modelo: {info['model']} - Estado: {health[0]}"
            self.info_label.configure(text=status_text, text_color="cyan")

            self.start_button.configure(state="disabled")
            self.stop_button.configure(state="normal")
            self.running = True

            # Iniciar actualización periódica
            self.update_lidar_data()

        except Exception as e:
            self.info_label.configure(text=f"Error: {str(e)}", text_color="red")

    def stop_lidar(self):
        """Detiene el escaneo LIDAR"""
        self.running = False
        self.start_button.configure(state="normal")
        self.stop_button.configure(state="disabled")
        self.info_label.configure(text="Estado: Detenido", text_color="yellow")

        if self.lidar:
            try:
                self.lidar.stop()
                self.lidar.stop_motor()
                self.lidar.disconnect()
            except:
                pass

    def update_lidar_data(self):
        """Actualiza los datos del LIDAR"""
        if not self.running:
            return

        try:
            # Obtener datos del LIDAR
            scan = next(self.lidar.iter_scans(max_buf_meas=SCAN_BUFFER, min_len=3))

            # Crear nueva imagen (conservando las guías)
            self.radar_image = Image.new("RGB", (self.image_width, self.image_height), "black")
            draw = ImageDraw.Draw(self.radar_image)
            self.draw_radar_guides()  # Redibujar guías

            # Procesar puntos
            point_count = 0
            for measurement in scan:
                quality, angle, distance = measurement

                if distance > 20 and distance < D_MAX and quality > 0:
                    # Convertir a coordenadas cartesianas
                    rad = np.radians(angle)
                    x = self.center_x + distance * self.scale_factor * np.sin(rad)
                    y = self.center_y - distance * self.scale_factor * np.cos(rad)

                    # Mapear intensidad a color
                    intensity_norm = min(max((quality - I_MIN) / (I_MAX - I_MIN), 1.0), 0.0)
                    color = self.get_color(intensity_norm)

                    # Dibujar punto
                    draw.ellipse(
                        [(x - 2, y - 2), (x + 2, y + 2)],
                        fill=color,
                        outline=color
                    )
                    point_count += 1

            # Actualizar información
            self.info_label.configure(
                text=f"Puntos: {point_count} | Distancia máxima: {D_MAX} mm",
                text_color="cyan"
            )

            # Actualizar imagen
            self.update_radar_image()

        except Exception as e:
            self.info_label.configure(text=f"Error: {str(e)}", text_color="red")
            self.stop_lidar()

        # Programar próxima actualización
        self.after(REFRESH_RATE, self.update_lidar_data)

    def get_color(self, intensity):
        """Mapea la intensidad a un color"""
        # Mapeo simple de intensidad a color (azul a rojo)
        r = int(255 * intensity)
        g = 0
        b = int(255 * (1 - intensity))
        return (r, g, b)

    def on_closing(self):
        """Maneja el cierre de la ventana"""
        self.stop_lidar()
        self.destroy()


if __name__ == "__main__":
    app = LidarVisualizationApp()
    app.protocol("WM_DELETE_WINDOW", app.on_closing)
    app.mainloop()