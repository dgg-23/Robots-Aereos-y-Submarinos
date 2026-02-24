import time
import math
import os
import csv
import subprocess
from datetime import datetime
from pymavlink import mavutil

# ----------------------------
# CONFIGURACIÓN
# ----------------------------
CONNECTION_STRING = "udp:127.0.0.1:14550"

RADIO_ORBITA = 6

# En vez de 20, para fotogrametría suele ir mejor 30–40
FOTOS_POR_VUELTA = 40

# Alturas para “doble órbita” (paralaje vertical)
ALTITUDES = [5, 9]

# Centro (idealmente el objeto). Si tu objeto está en el HOME, deja estos valores como están o usa HOME_POSITION.
CENTRO_LAT = -35.363261
CENTRO_LON = 149.165230
ROI_ALT = 2.0  # altura a la que “apunta” el ROI (2m funciona bien para estructuras)

# Cámara (SDP que ya te funciona con ffplay)
SDP_PATH = os.path.expanduser("~/gz_cam.sdp")

# Captura
TIEMPO_ESTABILIZACION_S = 4.0
ESPERA_YAW_S = 1.0

# Gimbal (opcional)
USAR_GIMBAL_PITCH = True  # pon True si ves que el gimbal responde---------------------------------------------------------------------------------------------------------IGUAL PONER FALSE
GIMBAL_PITCH_DEG = -50     # negativo = hacia abajo
# Algunas configuraciones usan centi-grados. Si no responde, prueba -3500 (y pon USAR_GIMBAL_PITCH=True).
GIMBAL_USA_CENTIDEG = False


# ----------------------------
# UTILIDADES GEO
# ----------------------------
def calcular_bearing(lat1, lon1, lat2, lon2):
    """Bearing (0=Norte) desde (lat1,lon1) a (lat2,lon2) en grados."""
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dlon = math.radians(lon2 - lon1)

    y = math.sin(dlon) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlon)
    brng = (math.degrees(math.atan2(y, x)) + 360) % 360
    return brng


def distancia_aprox_m(lat1, lon1, lat2, lon2):
    """Distancia aproximada en metros (suficiente para esperar llegada)."""
    R = 6371000.0
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * R * math.asin(math.sqrt(a))


# ----------------------------
# MAVLINK / DRON
# ----------------------------
def conectar():
    print(f"Conectando a {CONNECTION_STRING}...")
    vehicle = mavutil.mavlink_connection(CONNECTION_STRING)
    vehicle.wait_heartbeat()
    print("¡Conectado al vehículo!")
    return vehicle


def obtener_telemetria(vehicle):
    msg = vehicle.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1)
    hdg_msg = vehicle.recv_match(type="VFR_HUD", blocking=True, timeout=1)

    if msg and hdg_msg:
        return msg.lat / 1e7, msg.lon / 1e7, msg.relative_alt / 1000.0, hdg_msg.heading
    return None


def armar_y_despegar(vehicle, altura):
    print("Cambiando a modo GUIDED...")
    vehicle.set_mode("GUIDED")
    time.sleep(1.0)

    print("Armando...")
    vehicle.arducopter_arm()
    vehicle.motors_armed_wait()

    print(f"Despegando a {altura}m...")
    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0,
        altura
    )
    time.sleep(10)


def apuntar_roi(vehicle, lat, lon, alt):
    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_ROI,
        0,
        0, 0, 0, 0,
        lat, lon, alt
    )


def yaw_hacia(vehicle, heading_deg, rate_deg_s=25):
    """Gira el dron hacia un heading absoluto."""
    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        float(heading_deg),       # angle
        float(rate_deg_s),        # yaw rate
        1,                        # direction (1 = CW)
        0,                        # relative (0 = absolute)
        0, 0, 0
    )


def set_gimbal_pitch(vehicle, pitch_deg):
    """Inclina gimbal (si tu modelo lo soporta)."""
    val = pitch_deg
    if GIMBAL_USA_CENTIDEG:
        val = int(pitch_deg * 100)

    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
        0,
        float(val), 0, 0,   # pitch, roll, yaw
        0, 0, 0,
        0
    )


T0 = time.time()

def ir_a(vehicle, lat, lon, alt):
    # Máscara: solo posición (ignoramos velocidades/aceleraciones/yaw)
    type_mask = 0b0000111111111000
    time_boot_ms = int((time.time() - T0) * 1000)

    vehicle.mav.set_position_target_global_int_send(
        time_boot_ms,
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        type_mask,
        int(lat * 1e7), int(lon * 1e7), float(alt),
        0, 0, 0,   # vx, vy, vz
        0, 0, 0,   # afx, afy, afz
        0, 0,
        0       # yaw, yaw_rate
    )


def esperar_cerca(vehicle, lat_obj, lon_obj, umbral_m=2.5, timeout_s=20):
    """Espera hasta estar cerca del punto para mejorar estabilidad del dataset."""
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        tel = obtener_telemetria(vehicle)
        if tel is None:
            continue
        lat, lon, _, _ = tel
        if distancia_aprox_m(lat, lon, lat_obj, lon_obj) <= umbral_m:
            return True
        time.sleep(0.3)
    return False


def generar_orbita(centro_lat, centro_lon, radio, altura, num_puntos):
    waypoints = []
    for i in range(num_puntos):
        angulo = math.radians((360 / num_puntos) * i)
        d_lat = (radio * math.cos(angulo)) / 111320
        d_lon = (radio * math.sin(angulo)) / (40075000 * math.cos(math.radians(centro_lat)) / 360)
        waypoints.append((centro_lat + d_lat, centro_lon + d_lon, altura))
    return waypoints


# ----------------------------
# CÁMARA (ffmpeg + SDP)
# ----------------------------
def capturar_foto_ffmpeg(output_path, max_reintentos=5, timeout_s=8):
    if not os.path.exists(SDP_PATH):
        raise FileNotFoundError(f"No existe el SDP en {SDP_PATH}.")

    cmd = [
        "ffmpeg",
        "-nostdin",
        "-hide_banner",
        "-loglevel", "error",
        "-protocol_whitelist", "file,udp,rtp",
        "-fflags", "nobuffer",
        "-flags", "low_delay",
        "-analyzeduration", "0",
        "-probesize", "32",
        "-y",
        "-i", SDP_PATH,
        "-frames:v", "1",
        "-q:v", "2",
        output_path
    ]

    for intento in range(1, max_reintentos + 1):
        try:
            subprocess.run(cmd, check=True, timeout=timeout_s)
            return os.path.exists(output_path) and os.path.getsize(output_path) > 0
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired):
            time.sleep(0.4)
    return False


def warmup_camara():
    print("Warm-up cámara (enganchar keyframe)...")
    tmp = "/tmp/warmup.jpg"
    for _ in range(6):
        capturar_foto_ffmpeg(tmp, max_reintentos=2, timeout_s=4)
        time.sleep(0.4)


# ----------------------------
# MAIN
# ----------------------------
def main():
    # Crear dataset
    carpeta_dataset = f"dataset_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    os.makedirs(carpeta_dataset, exist_ok=True)
    csv_file = os.path.join(carpeta_dataset, "telemetria.csv")

    drone = conectar()

    # Warm-up: importante para H264 (SPS/PPS/keyframe)
    warmup_camara()

    # Despegue al primer nivel
    armar_y_despegar(drone, ALTITUDES[0])

    # ROI y gimbal
    apuntar_roi(drone, CENTRO_LAT, CENTRO_LON, ROI_ALT)
    if USAR_GIMBAL_PITCH:
        set_gimbal_pitch(drone, GIMBAL_PITCH_DEG)
        time.sleep(0.5)

    # Generar puntos para TODAS las alturas (doble órbita)
    puntos = []
    for alt in ALTITUDES:
        puntos.extend(generar_orbita(CENTRO_LAT, CENTRO_LON, RADIO_ORBITA, alt, FOTOS_POR_VUELTA))

    print(f"Iniciando escaneo: {len(puntos)} fotos totales ({len(ALTITUDES)} alturas).")
    print(f"Guardando en: {carpeta_dataset}")

    with open(csv_file, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["ID", "Filename", "Lat", "Lon", "Alt", "Heading", "AltObjetivo"])

        for i, (lat_wp, lon_wp, alt_wp) in enumerate(puntos, start=1):
            # Ir al waypoint
            ir_a(drone, lat_wp, lon_wp, alt_wp)

            # Esperar aproximación + estabilización
            esperar_cerca(drone, lat_wp, lon_wp, umbral_m=2.5, timeout_s=20)
            time.sleep(TIEMPO_ESTABILIZACION_S)

            tel = obtener_telemetria(drone)
            if tel is None:
                print(f"[{i}] Telemetría no disponible, reintentando...")
                time.sleep(0.5)
                tel = obtener_telemetria(drone)

            if tel is None:
                curr_lat, curr_lon, curr_alt, curr_hdg = 0, 0, 0, 0
            else:
                curr_lat, curr_lon, curr_alt, curr_hdg = tel

            # Yaw hacia el centro (clave para que el objeto quede centrado)
            bearing = calcular_bearing(curr_lat, curr_lon, CENTRO_LAT, CENTRO_LON)
            yaw_hacia(drone, bearing)
            time.sleep(ESPERA_YAW_S)

            # ROI + pitch por si el gimbal lo soporta
            apuntar_roi(drone, CENTRO_LAT, CENTRO_LON, ROI_ALT)
            if USAR_GIMBAL_PITCH:
                set_gimbal_pitch(drone, GIMBAL_PITCH_DEG)

            time.sleep(0.8) #tiempo de sleep para que la foto salga seguida--------------------------------------------------

            # Captura
            nombre_archivo = f"foto_{i:03d}.jpg"
            ruta_foto = os.path.join(carpeta_dataset, nombre_archivo)
            ok = capturar_foto_ffmpeg(ruta_foto, max_reintentos=4, timeout_s=6)

            writer.writerow([i, nombre_archivo, curr_lat, curr_lon, curr_alt, curr_hdg, alt_wp])

            if ok:
                print(f"[FOTO {i}/{len(puntos)}] OK ({nombre_archivo}) alt={alt_wp} yaw->centro={bearing:.1f}°")
            else:
                print(f"[FOTO {i}/{len(puntos)}] ERROR capturando frame (se registra telemetría igualmente).")

    print("Misión completada. Aterrizando...")
    drone.set_mode("LAND")


if __name__ == "__main__":
    main()