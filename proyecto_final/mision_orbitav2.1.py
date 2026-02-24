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
FOTOS_POR_VUELTA = 20
ALTITUDES = [5]

CENTRO_LAT = -35.363261
CENTRO_LON = 149.165230
ROI_ALT = 2.0

SDP_PATH = os.path.expanduser("~/gz_cam.sdp")

# Tiempos
TIEMPO_ESTABILIZACION_S = 4.0
ESPERA_YAW_S = 1.0
ESPERA_CAMARA_PRE_FOTO_S = 0.8   # tiempo para que ROI/gimbal terminen de asentarse
ESPERA_POST_FOTO_S = 1.0         # lo que pediste: 1 segundo tras hacer la foto

# Criterios para "parado por completo"
UMBRAL_DIST_M = 2.0              # cerca del waypoint
UMBRAL_VEL_MS = 0.25             # velocidad casi 0 (m/s)
TIEMPO_ESTABLE_PARADO_S = 1.0    # cuánto tiempo debe mantenerse parado

# Yaw (cara al centro)
TOL_YAW_DEG = 6.0                # tolerancia de orientación

# Gimbal
USAR_GIMBAL_PITCH = False
GIMBAL_PITCH_DEG = -65
GIMBAL_USA_CENTIDEG = False      # si no se nota, ponlo en True


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
    """Distancia aproximada en metros."""
    R = 6371000.0
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * R * math.asin(math.sqrt(a))


def diff_ang_deg(a, b):
    """Diferencia mínima entre ángulos en grados."""
    d = (a - b + 180) % 360 - 180
    return abs(d)


# ----------------------------
# MAVLINK / DRON
# ----------------------------
def conectar():
    print(f"Conectando a {CONNECTION_STRING}...")
    vehicle = mavutil.mavlink_connection(CONNECTION_STRING)
    vehicle.wait_heartbeat()
    print("¡Conectado al vehículo!")
    return vehicle


def obtener_estado(vehicle):
    """
    Devuelve: lat, lon, alt, heading_deg, groundspeed_mps, climb_mps
    """
    pos = vehicle.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1)
    hud = vehicle.recv_match(type="VFR_HUD", blocking=True, timeout=1)

    if not pos or not hud:
        return None

    lat = pos.lat / 1e7
    lon = pos.lon / 1e7
    alt = pos.relative_alt / 1000.0
    heading = float(hud.heading)
    groundspeed = float(hud.groundspeed)  # m/s
    climb = float(hud.climb)              # m/s
    return lat, lon, alt, heading, groundspeed, climb


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

    # ✅ corregido: command_long_send lleva 7 params tras el '0'
    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
        0,
        float(val), 0, 0,
        0, 0, 0, 0
    )


T0 = time.time()

def ir_a(vehicle, lat, lon, alt):
    # Máscara: solo posición
    type_mask = 0b0000111111111000
    time_boot_ms = int((time.time() - T0) * 1000)

    # ✅ corregido: yaw,yaw_rate son 2 valores (no 3)
    vehicle.mav.set_position_target_global_int_send(
        time_boot_ms,
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        type_mask,
        int(lat * 1e7), int(lon * 1e7), float(alt),
        0, 0, 0,   # vx, vy, vz
        0, 0, 0,   # afx, afy, afz
        0, 0       # yaw, yaw_rate
    )


def generar_orbita(centro_lat, centro_lon, radio, altura, num_puntos):
    waypoints = []
    for i in range(num_puntos):
        angulo = math.radians((360 / num_puntos) * i)
        d_lat = (radio * math.cos(angulo)) / 111320
        d_lon = (radio * math.sin(angulo)) / (40075000 * math.cos(math.radians(centro_lat)) / 360)
        waypoints.append((centro_lat + d_lat, centro_lon + d_lon, altura))
    return waypoints


def esperar_llegada_y_parada(vehicle, lat_obj, lon_obj, timeout_s=35):
    """
    Espera a:
      - estar cerca del waypoint (dist <= UMBRAL_DIST_M)
      - groundspeed <= UMBRAL_VEL_MS durante TIEMPO_ESTABLE_PARADO_S
    """
    t0 = time.time()
    t_estable_inicio = None

    while time.time() - t0 < timeout_s:
        st = obtener_estado(vehicle)
        if st is None:
            continue
        lat, lon, alt, hdg, gs, climb = st

        dist = distancia_aprox_m(lat, lon, lat_obj, lon_obj)

        cerca = dist <= UMBRAL_DIST_M
        parado = gs <= UMBRAL_VEL_MS and abs(climb) <= 0.25

        if cerca and parado:
            if t_estable_inicio is None:
                t_estable_inicio = time.time()
            elif time.time() - t_estable_inicio >= TIEMPO_ESTABLE_PARADO_S:
                return True
        else:
            t_estable_inicio = None

        time.sleep(0.2)

    return False


def esperar_yaw_al_centro(vehicle, heading_obj, timeout_s=12):
    """
    Espera hasta que el heading se acerque a heading_obj
    mientras el dron se mantiene casi parado.
    """
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        st = obtener_estado(vehicle)
        if st is None:
            continue
        lat, lon, alt, hdg, gs, climb = st

        if diff_ang_deg(hdg, heading_obj) <= TOL_YAW_DEG and gs <= (UMBRAL_VEL_MS + 0.2):
            return True
        time.sleep(0.2)
    return False


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

    for _ in range(max_reintentos):
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
        time.sleep(0.7)


# ----------------------------
# MAIN
# ----------------------------
def main():
    carpeta_dataset = f"dataset_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    os.makedirs(carpeta_dataset, exist_ok=True)
    csv_file = os.path.join(carpeta_dataset, "telemetria.csv")

    drone = conectar()
    warmup_camara()

    # despegar
    armar_y_despegar(drone, ALTITUDES[0])

    # apuntar al centro desde el principio
    apuntar_roi(drone, CENTRO_LAT, CENTRO_LON, ROI_ALT)
    if USAR_GIMBAL_PITCH:
        set_gimbal_pitch(drone, GIMBAL_PITCH_DEG)
        time.sleep(0.8)

    # waypoints (doble altura)
    puntos = []
    for alt in ALTITUDES:
        puntos.extend(generar_orbita(CENTRO_LAT, CENTRO_LON, RADIO_ORBITA, alt, FOTOS_POR_VUELTA))

    print(f"Iniciando escaneo: {len(puntos)} fotos totales ({len(ALTITUDES)} alturas).")
    print(f"Guardando en: {carpeta_dataset}")

    with open(csv_file, "w", newline="") as f:
        writer = csv.writer(f)
        # Usamos tu cabecera original del CSV exactamente igual
        writer.writerow(["ID", "Filename", "Lat", "Lon", "Alt", "Heading", "AltObjetivo"])

        for i, (lat_wp, lon_wp, alt_wp) in enumerate(puntos, start=1):
            
            # --- 1. VIAJE Y PARADA ---
            ir_a(drone, lat_wp, lon_wp, alt_wp)
            print(f"[{i}] Viajando al punto y frenando...")
            esperar_llegada_y_parada(drone, lat_wp, lon_wp, timeout_s=35)
            time.sleep(TIEMPO_ESTABILIZACION_S)

            # --- 2. ORIENTACIÓN CONDICIONAL AL CENTRO ---
            st = obtener_estado(drone)
            curr_lat, curr_lon = (st[0], st[1]) if st else (lat_wp, lon_wp)
            
            bearing = calcular_bearing(curr_lat, curr_lon, CENTRO_LAT, CENTRO_LON)
            yaw_hacia(drone, bearing)
            
            # ¡AQUÍ ESTÁ LA MAGIA CONDICIONAL!
            # En vez de esperar un tiempo fijo, lee la brújula hasta que encaje con el centro
            print(f"[{i}] Girando y comprobando que el morro apunte al centro ({bearing:.1f}°)...")
            logrado = esperar_yaw_al_centro(drone, bearing, timeout_s=15)
            
            if not logrado:
                print(f"[{i}] ⚠️ Aviso: Tardó mucho en girar, continuando de todos modos.")

            # --- 3. ENFOQUE FINAL ---
            apuntar_roi(drone, CENTRO_LAT, CENTRO_LON, ROI_ALT)
            if USAR_GIMBAL_PITCH:
                set_gimbal_pitch(drone, GIMBAL_PITCH_DEG)
            time.sleep(ESPERA_CAMARA_PRE_FOTO_S)

            # --- 4. CAPTURA Y TELEMETRÍA ---
            nombre_archivo = f"foto_{i:03d}.jpg"
            ruta_foto = os.path.join(carpeta_dataset, nombre_archivo)
            
            ok = capturar_foto_ffmpeg(ruta_foto, max_reintentos=4, timeout_s=6)

            st_final = obtener_estado(drone)
            if st_final is None:
                curr_lat, curr_lon, curr_alt, curr_hdg = 0, 0, 0, 0
            else:
                curr_lat, curr_lon, curr_alt, curr_hdg = st_final[0], st_final[1], st_final[2], st_final[3]

            writer.writerow([i, nombre_archivo, curr_lat, curr_lon, curr_alt, curr_hdg, alt_wp])

            if ok:
                print(f"[FOTO {i}/{len(puntos)}] OK ({nombre_archivo}) alt={alt_wp} yaw_real={curr_hdg:.1f}°")
            else:
                print(f"[FOTO {i}/{len(puntos)}] ERROR capturando frame (se registra telemetría igualmente).")
            
            # --- 5. ESPERA FINAL ---
            time.sleep(ESPERA_POST_FOTO_S)

    print("Misión completada. Aterrizando...")
    drone.set_mode("LAND")

if __name__ == "__main__":
    main()