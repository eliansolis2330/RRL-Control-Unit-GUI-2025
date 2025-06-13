#!/usr/bin/env python3
import os
import subprocess
import argparse
from pathlib import Path

# CONFIGURACIÓN MANUAL - EDITA ESTA RUTA CON LA UBICACIÓN DE TU REPOSITORIO
REPO_PATH = "/ruta/completa/al/repositorio/gopro_as_webcam_on_linux"  # <-- CAMBIAR ESTO


def configure_gopro_webcam():
    # Verificar la ruta del repositorio
    repo_dir = Path(REPO_PATH).expanduser().absolute()
    script_path = repo_dir / "gopro_as_webcam.sh"

    if not script_path.exists():
        print(f"Error: No se encontró el script gopro_as_webcam.sh en {repo_dir}")
        print("Por favor edita la variable REPO_PATH en este script con la ruta correcta")
        return False

    print(f"Usando repositorio en: {repo_dir}")

    # Configurar parámetros (puedes editar estos valores también)
    fov = "wide"
    resolution = "480"
    auto_start = True

    print("\nConfigurando GoPro como webcam...")
    print(f"• FOV: {fov}")
    print(f"• Resolución: {resolution}p")
    print(f"• Auto-start: {'Activado' if auto_start else 'Desactivado'}\n")

    # Construir el comando
    cmd = [str(script_path), f"--fov={fov}", f"--resolution={resolution}"]
    if auto_start:
        cmd.append("--auto-start")

    try:
        # Cambiar al directorio del repositorio
        original_dir = os.getcwd()
        os.chdir(repo_dir)

        # Verificar permisos del script
        if not os.access("gopro_as_webcam.sh", os.X_OK):
            print("Otorgando permisos de ejecución al script...")
            os.chmod("gopro_as_webcam.sh", 0o755)

        # Ejecutar el comando
        print("Ejecutando:", " ".join(cmd))
        process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
            universal_newlines=True
        )

        # Mostrar salida en tiempo real
        for line in process.stdout:
            print(line, end='')

        # Esperar a que termine y verificar errores
        process.wait()
        if process.returncode != 0:
            print("\nError al ejecutar el script:")
            print(process.stderr.read())
            return False

        print("\n✅ Configuración completada exitosamente!")
        return True

    except FileNotFoundError:
        print("Error: No se encontró el script gopro_as_webcam.sh")
        return False
    except Exception as e:
        print(f"\n❌ Error inesperado: {str(e)}")
        return False
    finally:
        # Volver al directorio original
        os.chdir(original_dir)


if __name__ == "__main__":
    # Configuración de argumentos (opcional)
    parser = argparse.ArgumentParser(
        description='Configura GoPro como webcam en Linux',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=f"""Ejemplos de uso:
  {os.path.basename(__file__)}                  # Usa configuración por defecto
  {os.path.basename(__file__)} --fov linear     # Cambia FOV a linear
  {os.path.basename(__file__)} --resolution 720 # Usa resolución 720p
  {os.path.basename(__file__)} --no-auto-start  # Desactiva auto-start"""
    )

    parser.add_argument('--fov', default='wide',
                        help='Campo de visión (wide, linear, narrow)')
    parser.add_argument('--resolution', default='480',
                        help='Resolución (480, 720, 1080)')
    parser.add_argument('--no-auto-start', action='store_false', dest='auto_start',
                        help='Desactivar inicio automático')

    args = parser.parse_args()

    # Ejecutar la configuración
    success = configure_gopro_webcam()

    if not success:
        print("\n🔧 Sugerencias para solución de problemas:")
        print("1. Verifica que la variable REPO_PATH en el script apunta al directorio correcto")
        print("2. Asegúrate que la GoPro está en modo USB (Cámara GoPro)")
        print("3. Confirma que tienes instalados los requisitos:")
        print("   - ffmpeg")
        print("   - v4l2loopback-dkms")
        print("   - v4l2loopback-utils")
        print("4. Revisa que el script gopro_as_webcam.sh existe y tiene permisos de ejecución")
        print("\nPara más ayuda, consulta: https://github.com/jschmid1/gopro_as_webcam_on_linux")

        exit(1)