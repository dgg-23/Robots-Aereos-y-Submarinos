import time
import math
import os
import csv
import cv2  # librería de visión para leer la cámara
from datetime import datetime
from pymavlink import mavutil

# --- CONFIGURACIÓN ---
CONNECTION_STRING = 'udp:127.0.0.1:14550'
ALTITUD_VUELO = 10          # Metros
RADIO_ORBITA = 15           # Metros
FOTOS_POR_VUELTA = 20       # Solape alto

CENTRO_LAT = -35.363261     # Latitud del objeto (SITL)
CENTRO_LON = 149.165230     # Longitud del objeto (SITL)

# Crear carpeta para el proyecto
CARPETA_DATASET = f"dataset_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
if not os.path.exists(CARPETA_DATASET):
    os.makedirs(CARPETA_DATASET)

# Archivo de registro (Log) para fotogrametría
CSV_FILE = os.path.join(CARPETA_DATASET, 'telemetria.csv')


def log_telemetria(writer, foto_id, lat, lon, alt, heading):
    """Guarda la posición exacta donde se tomó la foto"""
    writer.writerow([foto_id, f"foto_{foto_id:03d}.jpg", lat, lon, alt, heading])

    # Simular creación de imagen (Placeholder)
    with open(os.path.join(CARPETA_DATASET, f"foto_{foto_id:03d}.jpg"), 'w') as f:
        f.write("Imagen simulada")


def conectar():
    print(f"Conectando a {CONNECTION_STRING}...")
    vehicle = mavutil.mavlink_connection(CONNECTION_STRING)
    vehicle.wait_heartbeat()
    print("¡Conectado al vehículo!")
    return vehicle


def obtener_telemetria(vehicle):
    # Pedir posición y orientación actual
    msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
    hdg_msg = vehicle.recv_match(type='VFR_HUD', blocking=True, timeout=1)

    if msg and hdg_msg:
        return (
            msg.lat / 1e7,
            msg.lon / 1e7,
            msg.relative_alt / 1000.0,
            hdg_msg.heading
        )

    return 0, 0, 0, 0


def armar_y_despegar(vehicle, altura):
    print("Cambiando a modo GUIDED...")
    vehicle.set_mode('GUIDED')
    vehicle.arducopter_arm()
    vehicle.motors_armed_wait()

    print(f"Despegando a {altura}m...")
    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0,
        0, 0,
        altura
    )
    time.sleep(10)


def apuntar_al_centro(vehicle, lat, lon, alt):
    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_ROI,
        0,
        0, 0, 0, 0,
        0,
        lat, lon, alt
    )


def ir_a(vehicle, lat, lon, alt):
    vehicle.mav.set_position_target_global_int_send(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b110111111000,
        int(lat * 1e7),
        int(lon * 1e7),
        alt,
        0, 0, 0,
        0, 0, 0,
        0, 0
    )


def generar_orbita(centro_lat, centro_lon, radio, altura, num_puntos):
    waypoints = []
    for i in range(num_puntos):
        angulo = math.radians((360 / num_puntos) * i)
        d_lat = (radio * math.cos(angulo)) / 111320
        d_lon = (radio * math.sin(angulo)) / (40075000 * math.cos(math.radians(centro_lat)) / 360)
        waypoints.append((centro_lat + d_lat, centro_lon + d_lon, altura))
    return waypoints


# --- EJECUCIÓN PRINCIPAL ---
drone = conectar()

# Preparar CSV
with open(CSV_FILE, 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(["ID", "Filename", "Lat", "Lon", "Alt", "Heading"])  # Cabecera

    armar_y_despegar(drone, ALTITUD_VUELO)
    apuntar_al_centro(drone, CENTRO_LAT, CENTRO_LON, 0)

    puntos = generar_orbita(
        CENTRO_LAT,
        CENTRO_LON,
        RADIO_ORBITA,
        ALTITUD_VUELO,
        FOTOS_POR_VUELTA
    )

    print(f"Iniciando escaneo. Guardando datos en: {CARPETA_DATASET}")

    for i, wp in enumerate(puntos):
        lat, lon, alt = wp
        ir_a(drone, lat, lon, alt)

        # Esperar a llegar y estabilizarse
        time.sleep(3)

        # Foto y registrar datos reales
        curr_lat, curr_lon, curr_alt, curr_hdg = obtener_telemetria(drone)
        log_telemetria(writer, i + 1, curr_lat, curr_lon, curr_alt, curr_hdg)

        print(f"[FOTO {i+1}/{FOTOS_POR_VUELTA}] Guardada. Heading: {curr_hdg}°")

print("Misión completada. Aterrizando...")
drone.set_mode('LAND')