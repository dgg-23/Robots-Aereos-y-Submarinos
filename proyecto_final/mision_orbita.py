import time
import math
import os
import csv
import subprocess
from datetime import datetime
from pymavlink import mavutil

# --- CONFIGURACIÓN ---
CONNECTION_STRING = 'udp:127.0.0.1:14550'
ALTITUD_VUELO = 10       # Metros
RADIO_ORBITA = 15        # Metros
FOTOS_POR_VUELTA = 20    # Sube a 30-40 para mejor reconstrucción
CENTRO_LAT = -35.363261  # Latitud del objeto (SITL)
CENTRO_LON = 149.165230  # Longitud del objeto (SITL)

# Streaming cámara (SDP que ya te funciona con ffplay)
SDP_PATH = os.path.expanduser("~/gz_cam.sdp")

# Crear carpeta para el proyecto (con fecha y hora)
CARPETA_DATASET = f"dataset_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
os.makedirs(CARPETA_DATASET, exist_ok=True)

# Archivo de registro (Log) para fotogrametría
CSV_FILE = os.path.join(CARPETA_DATASET, 'telemetria.csv')


def conectar():
    print(f"Conectando a {CONNECTION_STRING}...")
    vehicle = mavutil.mavlink_connection(CONNECTION_STRING)
    vehicle.wait_heartbeat()
    print("¡Conectado al vehículo!")
    return vehicle


def obtener_telemetria(vehicle):
    msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
    hdg_msg = vehicle.recv_match(type='VFR_HUD', blocking=True, timeout=1)

    if msg and hdg_msg:
        return msg.lat / 1e7, msg.lon / 1e7, msg.relative_alt / 1000.0, hdg_msg.heading
    return 0, 0, 0, 0


def armar_y_despegar(vehicle, altura):
    print("Cambiando a modo GUIDED...")
    vehicle.set_mode('GUIDED')
    vehicle.arducopter_arm()
    vehicle.motors_armed_wait()
    print(f"Despegando a {altura}m...")

    vehicle.mav.command_long_send(
        vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, altura
    )
    time.sleep(10)


def apuntar_al_centro(vehicle, lat, lon, alt):
    vehicle.mav.command_long_send(
        vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_ROI,
        0, 0, 0, 0, 0, lat, lon, alt
    )


T0 = time.time()

def ir_a(vehicle, lat, lon, alt):
    vehicle.mav.set_position_target_global_int_send(
        int((time.time() - T0) * 1000),          # time_boot_ms
        vehicle.target_system,                   # target_system correcto
        vehicle.target_component,                # target_component correcto
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b110111111000,
        int(lat * 1e7), int(lon * 1e7), alt,
        0, 0, 0,                                 # vx, vy, vz
        0, 0, 0,                                 # afx, afy, afz
        0, 0                                      # yaw, yaw_rate
    )


def generar_orbita(centro_lat, centro_lon, radio, altura, num_puntos):
    waypoints = []
    for i in range(num_puntos):
        angulo = math.radians((360 / num_puntos) * i)
        d_lat = (radio * math.cos(angulo)) / 111320
        d_lon = (radio * math.sin(angulo)) / (40075000 * math.cos(math.radians(centro_lat)) / 360)
        waypoints.append((centro_lat + d_lat, centro_lon + d_lon, altura))
    return waypoints


def capturar_foto_ffmpeg(output_path, max_reintentos=2):
    """
    Captura 1 frame del stream RTP/H264 descrito por SDP_PATH usando ffmpeg.
    Reintenta para evitar fallos por PPS/SPS al enganchar el stream.
    """
    if not os.path.exists(SDP_PATH):
        raise FileNotFoundError(f"No existe el SDP en {SDP_PATH}. Créalo con el cat > ~/gz_cam.sdp ...")

    cmd = [
        "ffmpeg", "-y",
        "-loglevel", "error",
        "-protocol_whitelist", "file,udp,rtp",
        "-i", SDP_PATH,
        "-frames:v", "1",
        output_path
    ]

    for intento in range(1, max_reintentos + 1):
        try:
            subprocess.run(cmd, check=True)
            return True
        except subprocess.CalledProcessError:
            if intento < max_reintentos:
                time.sleep(0.4)  # pequeño delay para esperar keyframe
            else:
                return False


def warmup_camara():
    """Captura 1-2 frames de calentamiento para que ffmpeg enganche SPS/PPS/keyframe."""
    print("Warm-up cámara (enganchar keyframe)...")
    tmp = "/tmp/warmup.jpg"
    for _ in range(2):
        capturar_foto_ffmpeg(tmp, max_reintentos=2)
        time.sleep(0.5)


# --- EJECUCIÓN PRINCIPAL ---
drone = conectar()

with open(CSV_FILE, 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(["ID", "Filename", "Lat", "Lon", "Alt", "Heading"])

    # Warm-up antes de empezar a guardar el dataset
    warmup_camara()

    armar_y_despegar(drone, ALTITUD_VUELO)
    apuntar_al_centro(drone, CENTRO_LAT, CENTRO_LON, 0)

    puntos = generar_orbita(CENTRO_LAT, CENTRO_LON, RADIO_ORBITA, ALTITUD_VUELO, FOTOS_POR_VUELTA)

    print(f"Iniciando escaneo. Guardando datos en la carpeta: {CARPETA_DATASET}")

    for i, (lat, lon, alt) in enumerate(puntos, start=1):
        ir_a(drone, lat, lon, alt)

        # Esperar a estabilizar (para fotogrametría, mejor 4s que 3s)
        time.sleep(4)

        # Telemetría justo en el momento
        curr_lat, curr_lon, curr_alt, curr_hdg = obtener_telemetria(drone)

        # Capturar imagen real
        nombre_archivo = f"foto_{i:03d}.jpg"
        ruta_foto = os.path.join(CARPETA_DATASET, nombre_archivo)
        ok = capturar_foto_ffmpeg(ruta_foto, max_reintentos=3)

        if not ok:
            print(f"[FOTO {i}/{FOTOS_POR_VUELTA}] ERROR capturando frame (se deja registro igualmente).")
        else:
            print(f"[FOTO {i}/{FOTOS_POR_VUELTA}] Guardada {nombre_archivo}. Heading: {curr_hdg}°")

        # Registrar en CSV (aunque la foto falle, queda trazabilidad)
        writer.writerow([i, nombre_archivo, curr_lat, curr_lon, curr_alt, curr_hdg])

    print("Misión completada. Aterrizando...")
    drone.set_mode('LAND')
