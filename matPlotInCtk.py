import customtkinter as ctk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
import threading
import time
from tkinter import messagebox


class RealTimePlotApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Gráfico en Tiempo Real")
        self.root.geometry("900x650")

        # Configuración de CustomTkinter
        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("green")

        # Variables de control
        self.is_running = False
        self.update_interval = 100  # ms
        self.data_points = 100
        self.max_points = 200  # Máximo de puntos a mostrar

        # Frame principal
        self.main_frame = ctk.CTkFrame(root)
        self.main_frame.pack(fill="both", expand=True, padx=10, pady=10)

        # Frame de controles
        self.control_frame = ctk.CTkFrame(self.main_frame)
        self.control_frame.pack(fill="x", padx=5, pady=5)

        # Controles
        self.label_points = ctk.CTkLabel(self.control_frame, text="Puntos a mostrar:")
        self.label_points.grid(row=0, column=0, padx=5, pady=5)

        self.entry_points = ctk.CTkEntry(self.control_frame, width=80)
        self.entry_points.insert(0, str(self.data_points))
        self.entry_points.grid(row=0, column=1, padx=5, pady=5)

        self.label_interval = ctk.CTkLabel(self.control_frame, text="Intervalo (ms):")
        self.label_interval.grid(row=0, column=2, padx=5, pady=5)

        self.entry_interval = ctk.CTkEntry(self.control_frame, width=80)
        self.entry_interval.insert(0, str(self.update_interval))
        self.entry_interval.grid(row=0, column=3, padx=5, pady=5)

        self.btn_start = ctk.CTkButton(
            self.control_frame,
            text="Iniciar",
            command=self.start_updating,
            fg_color="green"
        )
        self.btn_start.grid(row=0, column=4, padx=5, pady=5)

        self.btn_stop = ctk.CTkButton(
            self.control_frame,
            text="Detener",
            command=self.stop_updating,
            fg_color="red",
            state="disabled"
        )
        self.btn_stop.grid(row=0, column=5, padx=5, pady=5)

        # Frame para el gráfico
        self.plot_frame = ctk.CTkFrame(self.main_frame)
        self.plot_frame.pack(fill="both", expand=True, padx=5, pady=5)

        # Inicializar gráfico
        self.fig, self.ax = plt.subplots(figsize=(7, 5), dpi=100)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.plot_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        # Datos iniciales
        self.x_data = np.arange(self.max_points)
        self.y_data = np.zeros(self.max_points)

        # Configuración inicial del gráfico
        self.line, = self.ax.plot(self.x_data, self.y_data, 'b-', linewidth=2)
        self.ax.set_title("Gráfico en Tiempo Real", fontsize=14)
        self.ax.set_xlabel("Tiempo (muestras)", fontsize=12)
        self.ax.set_ylabel("Valor", fontsize=12)
        self.ax.set_ylim(-0.1, 1.1)
        self.ax.grid(True, linestyle='--', alpha=0.7)
        self.canvas.draw()

    def start_updating(self):
        if not self.is_running:
            try:
                self.data_points = int(self.entry_points.get())
                self.update_interval = int(self.entry_interval.get())

                if self.data_points <= 0 or self.update_interval <= 0:
                    messagebox.showerror("Error", "Los valores deben ser mayores que cero")
                    return

                self.is_running = True
                self.btn_start.configure(state="disabled")
                self.btn_stop.configure(state="normal")

                # Iniciar hilo para actualización en tiempo real
                self.update_thread = threading.Thread(target=self.update_plot, daemon=True)
                self.update_thread.start()

            except ValueError:
                messagebox.showerror("Error", "Por favor ingrese valores numéricos válidos")

    def stop_updating(self):
        self.is_running = False
        self.btn_start.configure(state="normal")
        self.btn_stop.configure(state="disabled")

    def update_plot(self):
        while self.is_running:
            # Generar nuevo dato aleatorio
            new_value = np.random.rand()

            # Desplazar los datos hacia la izquierda
            self.y_data = np.roll(self.y_data, -1)
            self.y_data[-1] = new_value

            # Actualizar gráfico en el hilo principal
            self.root.after(0, self.draw_plot)

            # Esperar el intervalo especificado
            time.sleep(self.update_interval / 1000)

    def draw_plot(self):
        # Actualizar los datos de la línea
        self.line.set_ydata(self.y_data)

        # Ajustar los límites del eje x según los puntos a mostrar
        if self.data_points < self.max_points:
            self.ax.set_xlim(0, self.data_points)
        else:
            self.ax.set_xlim(0, self.max_points)

        # Redibujar el canvas
        self.canvas.draw_idle()
        self.canvas.flush_events()

    def on_closing(self):
        self.is_running = False
        self.root.destroy()


if __name__ == "__main__":
    root = ctk.CTk()
    app = RealTimePlotApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()