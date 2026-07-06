import math
import os
from controller import Robot, Camera

# ============================================================
#  Controlador REATIVO (campo potencial) - patrulha de patio
#  Atracao ao waypoint + repulsao lidar. Sem mapa, sem A*.
# ============================================================

# ---- Parametros ----
MAX_SPEED = 6.0          # rad/s (teto do motor)
CRUISE = 5.0             # velocidade de cruzeiro
WP_REACH = 1.5           # dist. (m) p/ considerar waypoint atingido

K_ATT = 1.0              # ganho do vetor de atracao (alvo)
K_REP = 0.9              # ganho do vetor de repulsao (obstaculos)
K_TAN = 1.4              # ganho do campo tangencial (contorna paredes/quinas)
REP_RANGE = 1.2          # dist. (m) de influencia dos obstaculos
KP_STEER = 3.0           # ganho de conversao (erro de angulo -> giro)

FRONT_STOP = 0.5         # dist. (m) frontal p/ girar no lugar (anti-colisao)
FRONT_SLOW = 2.0         # dist. (m) frontal p/ comecar a frear
FRONT_SECTOR = 20        # meia-abertura (deg) do setor frontal de seguranca

# Contorno de quina (semicirculo ao redor do fim de uma parede)
OPEN_DIST = 3.0          # dist. (m) p/ considerar um lado "aberto"
ARC_FWD = 2.2            # velocidade de avanco durante o arco
ARC_TURN = 2.6           # giro durante o arco (define o raio do semicirculo)
ALIGN_EXIT = 0.6         # erro de angulo (rad) p/ sair do contorno

# Deteccao por camera + estados (prioridade: pessoa > cubo, como no A*)
CV_STEP = 2              # subamostragem de pixels
DETECT_MIN_PIXELS = 6    # minimo p/ INICIAR aproximacao
LOST_MIN_PIXELS = 2      # abaixo disso, alvo perdido de vista
COOLDOWN = 20.0          # s de cooldown por tipo de anomalia
KP_GOAL = 2.5            # ganho de direcao ao blob (aproximacao)
APPROACH_DIST = 5.0      # dist. (m) final de aproximacao (nao bate no alvo)
APPROACH_SPEED = 5.0     # velocidade ao se aproximar do alvo
ALIGN_TOL = 0.12         # centralizacao do blob (fracao da largura)
FOLLOW_TIME = 10.0       # s seguindo a pessoa

# Rota de patrulha (loop). Cobre os 4 setores do patio.
WAYPOINTS = [
    (-24.0, 0.0),
    (-24.0, -24.0),
    (17.0, -26.0),
    (20.0, 6.0),
    (20.0, 20.0),
    (6.0, 20.0),
    (-21.0, 20.0),
    (-21.0, 8.0),
    (3.0, 0.0),
]

# ---- Dispositivos ----
robot = Robot()
timestep = int(robot.getBasicTimeStep())

gps = robot.getDevice("gps")
gps.enable(timestep)

imu = robot.getDevice("inertial unit")
imu.enable(timestep)

lidar = robot.getDevice("Sick LMS 291")
lidar.enable(timestep)
lidar.enablePointCloud()

camera = robot.getDevice("camera")
camera.enable(timestep)
cam_w = camera.getWidth()
cam_h = camera.getHeight()

keyboard = robot.getKeyboard()
keyboard.enable(timestep)

motor_names = ["front left wheel", "front right wheel",
               "back left wheel", "back right wheel"]
motors = []
for n in motor_names:
    m = robot.getDevice(n)
    m.setPosition(float("inf"))
    m.setVelocity(0.0)
    motors.append(m)


# Aplica velocidade (rad/s) as rodas esquerda/direita, com clamp.
def set_speed(v_left, v_right):
    v_left = max(-MAX_SPEED, min(MAX_SPEED, v_left))
    v_right = max(-MAX_SPEED, min(MAX_SPEED, v_right))
    motors[0].setVelocity(v_left)
    motors[1].setVelocity(v_right)
    motors[2].setVelocity(v_left)
    motors[3].setVelocity(v_right)


# ---- Log de trajetoria ----
log_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "trajetoria_gps.txt")
log_file = open(log_path, "w")
print("[LOG] Gravando trajetoria em: %s" % log_path)
print("[INFO] Pressione 'F' na janela 3D para encerrar.")


# ---- Utilidades ----
def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def dist(ax, ay, bx, by):
    return math.hypot(ax - bx, ay - by)


# ---- LIDAR ----
lidar_fov = lidar.getFov()
lidar_res = lidar.getHorizontalResolution()
lidar_max = lidar.getMaxRange()


# Angulo (rad) do feixe de indice i. Positivo = esquerda (CCW).
def beam_angle(i):
    return lidar_fov / 2.0 - i * lidar_fov / (lidar_res - 1)


# Vetor de repulsao (rx, ry) no ref. do robo (x=frente, y=esquerda).
# Cada obstaculo empurra o robo na direcao oposta ao feixe.
def repulsion(ranges):
    rx = ry = 0.0
    for i in range(lidar_res):
        r = ranges[i]
        if r is None or math.isinf(r) or math.isnan(r) or r >= REP_RANGE:
            continue
        r = max(r, 0.15)
        w = K_REP * (1.0 / r - 1.0 / REP_RANGE)
        ang = beam_angle(i)
        rx -= w * math.cos(ang)
        ry -= w * math.sin(ang)
    return rx, ry


# Menor distancia no setor frontal [-FRONT_SECTOR, +FRONT_SECTOR] deg.
def front_min(ranges):
    half = math.radians(FRONT_SECTOR)
    lo = int(round((lidar_fov / 2.0 - half) * (lidar_res - 1) / lidar_fov))
    hi = int(round((lidar_fov / 2.0 + half) * (lidar_res - 1) / lidar_fov))
    best = float("inf")
    for i in range(max(0, min(lo, hi)), min(lidar_res, max(lo, hi) + 1)):
        r = ranges[i]
        if r is None or math.isinf(r) or math.isnan(r):
            continue
        best = min(best, r)
    return best


# Menor distancia num setor angular [lo_deg, hi_deg] (esq. positivo).
def sector_min(ranges, lo_deg, hi_deg):
    lo = int(round((lidar_fov / 2.0 - math.radians(hi_deg)) * (lidar_res - 1) / lidar_fov))
    hi = int(round((lidar_fov / 2.0 - math.radians(lo_deg)) * (lidar_res - 1) / lidar_fov))
    best = float("inf")
    for i in range(max(0, min(lo, hi)), min(lidar_res, max(lo, hi) + 1)):
        r = ranges[i]
        if r is None or math.isinf(r) or math.isnan(r):
            continue
        best = min(best, r)
    return best


# ---- Visao computacional ----
# (n_pixels, cx) do blob da cor `want`; cx in [-1,1], 0 = centro.
def detect_blob(image, want):
    count = 0
    sum_x = 0
    for py in range(0, cam_h, CV_STEP):
        for px in range(0, cam_w, CV_STEP):
            r = Camera.imageGetRed(image, cam_w, px, py)
            g = Camera.imageGetGreen(image, cam_w, px, py)
            b = Camera.imageGetBlue(image, cam_w, px, py)
            if want == "red":
                hit = r > 120 and g < 80 and b < 80
            else:
                hit = g > 110 and r < 100 and b < 100
            if hit:
                count += 1
                sum_x += px
    if count == 0:
        return 0, 0.0
    return count, (sum_x / count - cam_w / 2.0) / (cam_w / 2.0)


# Imprime alerta de monitoramento com GPS e orientacao.
def alerta(tipo, extra=""):
    p = gps.getValues()
    yaw = imu.getRollPitchYaw()[2]
    print("=" * 60)
    print("### ALERTA MONITORAMENTO :: %s" % tipo)
    print("    Tempo........: %.2f s" % robot.getTime())
    print("    Posicao GPS..: x=%.3f y=%.3f z=%.3f" % (p[0], p[1], p[2]))
    print("    Orientacao...: yaw=%.1f deg" % math.degrees(yaw))
    if extra:
        print("    Info.........: %s" % extra)
    print("=" * 60)


# ---- Estado (FSM, mesma logica/prioridade do A*) ----
MODE_PATROL = 0
MODE_APPROACH = 1          # dirige rumo ao blob detectado
MODE_FOLLOW_PERSON = 2     # segue a pessoa por FOLLOW_TIME
mode = MODE_PATROL
target_type = None         # 'green' (pessoa) ou 'red' (cubo) em MODE_APPROACH

wp_index = 0
goal = WAYPOINTS[wp_index]
last_cube_time = -COOLDOWN
last_person_time = -COOLDOWN
follow_start = 0.0
last_follow_log = 0.0
rounding_dir = 0           # 0=nenhum, +1=contorna p/ esquerda, -1=p/ direita
finished = False

# ---- Loop principal ----
while robot.step(timestep) != -1:
    key = keyboard.getKey()
    if key in (ord('F'), ord('f')):
        finished = True
        break

    pos = gps.getValues()
    x, y = pos[0], pos[1]
    yaw = imu.getRollPitchYaw()[2]
    log_file.write("%f %f %f\n" % (pos[0], pos[1], pos[2]))
    log_file.flush()

    ranges = lidar.getRangeImage()
    image = camera.getImage()
    t = robot.getTime()

    # ===== MODE_APPROACH: dirige ate o blob e reporta =====
    if mode == MODE_APPROACH:
        n, cx = detect_blob(image, target_type)
        if n < LOST_MIN_PIXELS:
            print("[CV] Alvo perdido de vista. Retomando patrulha.")
            mode = MODE_PATROL
            continue
        steer = KP_GOAL * (-cx)
        front = front_min(ranges)
        reached = front <= APPROACH_DIST
        fwd = 0.0 if reached else APPROACH_SPEED
        set_speed(fwd - steer, fwd + steer)
        if reached and abs(cx) < ALIGN_TOL:
            set_speed(0.0, 0.0)
            if target_type == "red":
                alerta("ANOMALIA (CUBO VERMELHO) REGISTRADA",
                       "distancia=%.2f m" % front)
                last_cube_time = t
                mode = MODE_PATROL
            else:
                alerta("INTRUSO (PESSOA) -- INICIANDO ACOMPANHAMENTO",
                       "distancia=%.2f m, seguira %ds" % (front, int(FOLLOW_TIME)))
                mode = MODE_FOLLOW_PERSON
                follow_start = t
                last_follow_log = t
        continue

    # ===== MODE_FOLLOW_PERSON: acompanha a pessoa por FOLLOW_TIME =====
    if mode == MODE_FOLLOW_PERSON:
        n_green, cx = detect_blob(image, "green")
        if t - follow_start >= FOLLOW_TIME:
            alerta("FIM DO ACOMPANHAMENTO DA PESSOA", "retomando rota")
            last_person_time = t
            mode = MODE_PATROL
            continue
        front = front_min(ranges)
        if n_green < LOST_MIN_PIXELS:
            set_speed(1.5, -1.5)          # procura girando
        else:
            steer = KP_GOAL * (-cx)
            fwd = max(-1.0, min(APPROACH_SPEED, 1.5 * (front - APPROACH_DIST)))
            set_speed(fwd - steer, fwd + steer)
        if t - last_follow_log >= 1.0:
            print("  [SEGUINDO PESSOA t=%.1fs] GPS=(%.2f,%.2f) dist=%.2fm blob=%d px"
                  % (t, x, y, front, n_green))
            last_follow_log = t
        continue

    # ===== MODE_PATROL =====
    # Deteccao com PRIORIDADE: pessoa (green) antes de cubo (red).
    n_green, _ = detect_blob(image, "green")
    if n_green >= DETECT_MIN_PIXELS and t - last_person_time >= COOLDOWN:
        print("[CV] Pessoa avistada (%d px). Aproximando..." % n_green)
        target_type = "green"
        mode = MODE_APPROACH
        continue

    n_red, _ = detect_blob(image, "red")
    if n_red >= DETECT_MIN_PIXELS and t - last_cube_time >= COOLDOWN:
        print("[CV] Cubo vermelho avistado (%d px). Aproximando..." % n_red)
        target_type = "red"
        mode = MODE_APPROACH
        continue

    # --- Avanca waypoint ---
    if dist(x, y, goal[0], goal[1]) < WP_REACH:
        wp_index = (wp_index + 1) % len(WAYPOINTS)
        goal = WAYPOINTS[wp_index]
        print("[NAV] Waypoint atingido. Proximo #%d = %s" % (wp_index, goal))

    bearing = wrap(math.atan2(goal[1] - y, goal[0] - x) - yaw)
    front = front_min(ranges)
    left = sector_min(ranges, 15, 90)
    right = sector_min(ranges, -90, -15)

    # ---- Contorno de quina (semicirculo) ----
    # Ao seguir uma parede e um lado se abrir, o campo potencial trava
    # (o fim fino da parede ainda gera repulsao e o robo oscila). Aqui
    # comprometemos um arco continuo rumo ao lado aberto ate a frente
    # liberar E o robo reapontar ao objetivo. Contorna a quina mesmo
    # com parede fina, em vez de girar parado.
    if rounding_dir == 0 and front < FRONT_SLOW and max(left, right) > OPEN_DIST:
        rounding_dir = 1 if left >= right else -1
        print("[NAV] Contornando quina (%s)." % ("esq" if rounding_dir > 0 else "dir"))

    if rounding_dir != 0:
        # Sai quando a frente esta livre e o robo aponta ao objetivo.
        if front > FRONT_SLOW and abs(bearing) < ALIGN_EXIT:
            rounding_dir = 0
        elif front < FRONT_STOP and max(left, right) < OPEN_DIST:
            rounding_dir = 0            # virou beco: cai no giro-no-lugar
        else:
            turn = ARC_TURN * rounding_dir
            set_speed(ARC_FWD - turn, ARC_FWD + turn)
            continue

    # ---- Campo potencial (atracao + repulsao radial + vortice) ----
    ax = K_ATT * math.cos(bearing)
    ay = K_ATT * math.sin(bearing)
    rx, ry = repulsion(ranges)

    # Campo tangencial (vortice): perpendicular a repulsao, no sentido do
    # alvo. Faz o robo CONTORNAR a parede em vez de so ser empurrado.
    rmag = math.hypot(rx, ry)
    tx = ty = 0.0
    if rmag > 1e-6:
        p1x, p1y = -ry, rx
        p2x, p2y = ry, -rx
        if p1x * ax + p1y * ay >= p2x * ax + p2y * ay:
            tx, ty = p1x, p1y
        else:
            tx, ty = p2x, p2y
        norm = K_TAN / rmag
        tx *= norm
        ty *= norm

    vx, vy = ax + rx + tx, ay + ry + ty
    desired = math.atan2(vy, vx)          # angulo no ref. do robo (0 = frente)

    if front < FRONT_STOP:
        # Beco sem saida: gira no lugar p/ o lado mais livre.
        turn = 1.0 if left >= right else -1.0
        set_speed(-CRUISE * 0.4 * turn, CRUISE * 0.4 * turn)
    else:
        turn = KP_STEER * desired
        slow = max(0.2, min(1.0, (front - FRONT_STOP) / (FRONT_SLOW - FRONT_STOP)))
        fwd = CRUISE * max(0.15, math.cos(desired)) * slow
        set_speed(fwd - turn, fwd + turn)

# ---- Encerramento ----
set_speed(0.0, 0.0)
log_file.close()
print("[LOG] Arquivo fechado: %s" % log_path)
if finished:
    print("[INFO] Simulacao encerrada pelo operador (tecla F).")
