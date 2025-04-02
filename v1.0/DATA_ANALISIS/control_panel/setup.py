#!/usr/bin/env python
"""
Script de instalación para Rocket Control Panel
"""
import os
import sys
import subprocess
import shutil
from pathlib import Path


def check_python_version():
    """Verificar la versión de Python"""
    print("Verificando versión de Python...")
    version = sys.version_info
    if version.major < 3 or (version.major == 3 and version.minor < 7):
        print("ERROR: Se requiere Python 3.7 o superior")
        print(f"Versión actual: {version.major}.{version.minor}.{version.micro}")
        return False
    print(f"OK: Usando Python {version.major}.{version.minor}.{version.micro}")
    return True


def install_requirements():
    """Instalar los paquetes requeridos"""
    print("Instalando paquetes requeridos...")
    try:
        subprocess.check_call(
            [sys.executable, "-m", "pip", "install", "customtkinter", "matplotlib", "numpy", "pyserial", "pygame",
             "scipy"])
        print("OK: Paquetes instalados correctamente")
        return True
    except subprocess.CalledProcessError as e:
        print(f"ERROR: No se pudieron instalar los paquetes: {e}")
        return False


def create_directories():
    """Crear directorios necesarios"""
    print("Creando directorios...")
    directories = ["flight_logs", "sounds"]
    for directory in directories:
        if not os.path.exists(directory):
            os.makedirs(directory)
            print(f"Creado: {directory}/")
        else:
            print(f"Ya existe: {directory}/")
    return True


def download_sound_samples():
    """Descargar archivos de sonido de muestra"""
    # Esta función podría implementarse para descargar sonidos de muestra
    # desde un repositorio en línea. Por ahora, solo crearemos archivos vacíos.
    print("Creando archivos de sonido de muestra...")
    sound_dir = Path("sounds")
    sound_files = [
        "connect.wav",
        "disconnect.wav",
        "command_sent.wav",
        "alert.wav",
        "launch.wav",
        "parachute_deploy.wav",
        "abort.wav",
        "state_change.wav",
        "apogee.wav",
        "landing.wav"
    ]

    for sound_file in sound_files:
        sound_path = sound_dir / sound_file
        if not sound_path.exists():
            # Crear archivo vacío (serán generados automáticamente al ejecutar)
            sound_path.touch()
            print(f"Creado: {sound_path}")
    return True


def create_config_file():
    """Crear archivo de configuración"""
    print("Creando archivo de configuración...")
    config_content = """{
    "connection": {
        "default_port": null,
        "baud_rate": 115200,
        "auto_connect": false
    },
    "telemetry": {
        "log_directory": "flight_logs",
        "auto_start_logging": true,
        "max_points_on_graph": 100,
        "update_rate": 20
    },
    "display": {
        "theme": "dark",
        "altitude_graph_color": "#FF5500",
        "acceleration_graph_colors": {
            "x": "#FF0000",
            "y": "#00FF00",
            "z": "#0000FF"
        },
        "gyro_graph_colors": {
            "x": "#FF5500",
            "y": "#55FF00",
            "z": "#0055FF"
        },
        "temperature_graph_color": "#FF0000",
        "pressure_graph_color": "#0000FF"
    },
    "safety": {
        "require_launch_confirmation": true,
        "require_abort_confirmation": true,
        "auto_deploy_parachute_altitude": 100,
        "auto_abort_on_excessive_rotation": false,
        "max_rotation_rate": 360
    },
    "sounds": {
        "enable_sounds": true,
        "play_sound_on_state_change": true,
        "play_sound_on_alert": true,
        "play_sound_on_command_sent": true
    },
    "alerts": {
        "altitude_threshold": 500,
        "acceleration_threshold": 30,
        "temperature_threshold_high": 50,
        "temperature_threshold_low": -10,
        "connection_loss_timeout": 5
    }
}"""

    config_path = Path("config.json")
    if not config_path.exists():
        with open(config_path, "w") as f:
            f.write(config_content)
        print(f"Creado: {config_path}")
    else:
        print(f"Ya existe: {config_path}")
    return True


def check_files():
    """Verificar que existan todos los archivos necesarios"""
    print("Verificando archivos...")
    required_files = [
        "main.py",
        "rocket_simulator.py"
    ]

    all_files_present = True
    for file in required_files:
        if not os.path.exists(file):
            print(f"ERROR: No se encuentra {file}")
            all_files_present = False
        else:
            print(f"OK: {file}")

    return all_files_present


def create_launcher():
    """Crear script de inicio"""
    print("Creando script de inicio...")

    # Launcher para Windows
    windows_launcher = "@echo off\npython main.py\npause"
    with open("start_rocket_panel.bat", "w") as f:
        f.write(windows_launcher)
    print("Creado: start_rocket_panel.bat")

    # Launcher para Linux/Mac
    unix_launcher = "#!/bin/bash\npython3 main.py"
    with open("start_rocket_panel.sh", "w") as f:
        f.write(unix_launcher)
    os.chmod("start_rocket_panel.sh", 0o755)  # Hacer ejecutable
    print("Creado: start_rocket_panel.sh")

    return True


def main():
    """Función principal de instalación"""
    print("=== Instalación de Rocket Control Panel ===")

    steps = [
        ("Verificar versión de Python", check_python_version),
        ("Instalar paquetes requeridos", install_requirements),
        ("Crear directorios", create_directories),
        ("Crear archivos de sonido de muestra", download_sound_samples),
        ("Crear archivo de configuración", create_config_file),
        ("Verificar archivos", check_files),
        ("Crear scripts de inicio", create_launcher)
    ]

    success = True
    for step_name, step_func in steps:
        print(f"\n>> {step_name}")
        step_success = step_func()
        if not step_success:
            success = False
            print(f"ADVERTENCIA: Paso '{step_name}' no completado correctamente")

    if success:
        print("\n=== Instalación completada correctamente ===")
        print("Puedes iniciar la aplicación con:")
        print("  - Windows: start_rocket_panel.bat")
        print("  - Linux/Mac: ./start_rocket_panel.sh")
    else:
        print("\n=== Instalación completada con advertencias ===")
        print("Revisa los mensajes anteriores para solucionar los problemas.")

    return 0


if __name__ == "__main__":
    sys.exit(main())