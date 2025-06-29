import subprocess
import os
import time

# --- Constantes ---
# Nombre del repositorio de GoPro
REPO_NAME = "gopro_as_webcam_on_linux"
# Ruta absoluta al directorio principal del usuario (~/)
HOME_DIR = os.path.expanduser("~")
# Ruta completa al repositorio de GoPro
REPO_PATH = os.path.join(HOME_DIR, REPO_NAME)
# Contraseña de administrador
ADMIN_PASSWORD = "0120" # ¡ADVERTENCIA DE SEGURIDAD! Considera métodos más seguros para entornos de producción.

# --- Funciones de Utilidad ---

def run_sudo_command(command, cwd=None, input_data=None, check_errors=True):
    """
    Ejecuta un comando con sudo, pasando la contraseña a través de STDIN usando sudo -S.

    Args:
        command (list): Lista de strings que representan el comando y sus argumentos.
        cwd (str, optional): Directorio de trabajo para el comando. Por defecto, None.
        input_data (str, optional): Datos adicionales para enviar al STDIN del comando (después de la contraseña).
        check_errors (bool, optional): Si es True, lanza una excepción si el comando falla. Por defecto, True.

    Returns:
        subprocess.CompletedProcess: El objeto que representa el resultado del comando.
    """
    # Prepend 'sudo -S' para que sudo lea la contraseña desde stdin
    full_command = ["sudo", "-S"] + command
    command_str = ' '.join(full_command)
    print(f"\nEjecutando: {command_str}")

    # Combinamos la contraseña con cualquier otro input_data
    # Agregamos un salto de línea después de la contraseña para simular el Enter
    combined_input = ADMIN_PASSWORD + "\n"
    if input_data:
        combined_input += input_data

    try:
        process = subprocess.run(
            full_command,
            cwd=cwd,
            stdin=subprocess.PIPE, # Necesario para enviar datos a stdin
            input=combined_input, # Este 'input' es el que se pasa al argumento 'input' de subprocess.run
            capture_output=True,
            text=True,
            check=check_errors
        )
        if process.stdout:
            print("STDOUT:")
            print(process.stdout)
        if process.stderr:
            print("STDERR:")
            print(process.stderr)
        return process
    except subprocess.CalledProcessError as e:
        print(f"Error al ejecutar '{command_str}': {e}")
        print(f"STDOUT: {e.stdout}")
        print(f"STDERR: {e.stderr}")
        raise
    except FileNotFoundError:
        print(f"Error: El comando '{full_command[0]}' no se encontró. Asegúrate de que esté instalado y en tu PATH.")
        raise
    except Exception as e:
        print(f"Ocurrió un error inesperado al ejecutar '{command_str}': {e}")
        raise

def ensure_repo_exists():
    """
    Verifica si el directorio del repositorio de GoPro existe en la ruta esperada.
    Si no, imprime un error y sale del script.
    """
    if not os.path.exists(REPO_PATH) or not os.path.isdir(REPO_PATH):
        print(f"Error: El directorio del repositorio '{REPO_PATH}' no se encontró.")
        print(f"Asegúrate de que el repositorio '{REPO_NAME}' esté clonado en tu directorio de usuario (~/).")
        print("Puedes clonarlo usando: git clone https://github.com/jschmid1/gopro_as_webcam_on_linux.git")
        exit(1)
    print(f"Repositorio '{REPO_NAME}' encontrado en '{REPO_PATH}'.")

# --- Lógica Principal de Automatización ---

def main():
    print("Iniciando la automatización para configurar tu GoPro como webcam...")

    # 1. Verificar y confirmar la existencia del repositorio
    ensure_repo_exists()
    print(f"Los comandos se ejecutarán desde el directorio: {REPO_PATH}")

    try:
        # cd gopro_as_webcam_on_linux/ [enter] -> Esto se maneja con `cwd=REPO_PATH` en cada comando.

        # sudo ./install.sh [enter] 0120 [enter]
        print("\n--- Ejecutando el script de instalación (sudo ./install.sh) ---")
        # El 'input_data="\n"' es el primer Enter que el script install.sh pide *antes* de la contraseña.
        # La contraseña '0120\n' se envía automáticamente por run_sudo_command.
        run_sudo_command(["./install.sh"], cwd=REPO_PATH, input_data="\n")
        print("El script de instalación se ejecutó. Si hubo alguna pregunta adicional, asegúrate de haberla respondido.")

        # sudo gopro webcam -f wide [enter] [enter]
        print("\n--- Configurando GoPro webcam: Modo Wide (sudo gopro webcam -f wide) ---")
        # Dos "enters" para el input_data
        run_sudo_command(["gopro", "webcam", "-f", "wide"], cwd=REPO_PATH, input_data="\n\n")
        print("Modo Wide configurado.")

        # [4 seg]
        print("Esperando 4 segundos...")
        time.sleep(4)

        # sudo gopro webcam -r 480 [enter] [enter]
        print("\n--- Configurando GoPro webcam: Resolución 480p (sudo gopro webcam -r 480) ---")
        # Dos "enters" para el input_data
        run_sudo_command(["gopro", "webcam", "-r", "480"], cwd=REPO_PATH, input_data="\n\n")
        print("Resolución 480p configurada.")

        # [4 seg]
        print("Esperando 4 segundos...")
        time.sleep(4)

        # sudo gopro webcam -a [enter] [enter]
        print("\n--- Activando GoPro webcam (sudo gopro webcam -a) ---")
        # Dos "enters" para el input_data
        run_sudo_command(["gopro", "webcam", "-a"], cwd=REPO_PATH, input_data="\n\n")
        print("GoPro webcam activada.")

        print("\n¡Configuración y activación de tu GoPro como webcam completadas exitosamente!")
        print("Ahora puedes usar tu GoPro en aplicaciones como OBS Studio, VLC, o cualquier otra que soporte cámaras web.")
        print("Para detener la webcam, es probable que necesites ejecutar 'sudo gopro webcam -d' o simplemente desconectar la GoPro.")

    except Exception as e:
        print(f"\n¡Se produjo un error durante la automatización! Por favor, revisa los mensajes anteriores para más detalles.")
        print(f"Error específico: {e}")
        print("Asegúrate de que tu GoPro esté conectada y encendida en modo webcam, y de que los permisos de sudo sean correctos.")

if __name__ == "__main__":
    main()