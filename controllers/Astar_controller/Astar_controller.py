import heapq
import math
import os
from controller import Robot, Camera

# ---- Parametros globais ----
MAX_SPEED = 6.0
CRUISE = 6.0
ACTIVATE_DIST = 0.6
SAFE_STOP = 0.6
KP_GOAL = 2.5

CV_STEP = 1
DETECT_MIN_PIXELS = 5
LOST_MIN_PIXELS = 2
COOLDOWN = 30.0
FOLLOW_TIME = 10.0
ALIGN_TOL = 0.12
APPROACH_DIST = 5.0
APPROACH_SPEED = 6.0

WAYPOINTS = [
    (-24.0, 0.0),
    (-24.0, -24.0),
    (17.0, -26.0),
    (20.0, 6.0),
    (28.0, 9.0),
    (28.0, 19.0),
    (20.0, 20.0),
    (17.0, 27.0),
    (6.0, 28.0),
    (6.0, 20.0),
    (-21.0, 16.0),
    (-21.0, 20.0),
    (-21.0, 8.0),
    (7.0, 7.0),
    (7.0, 0.0),
    (3.0, 0.0)
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


def set_speed(v_left, v_right):
    v_left = max(-MAX_SPEED, min(MAX_SPEED, v_left))
    v_right = max(-MAX_SPEED, min(MAX_SPEED, v_right))
    motors[0].setVelocity(v_left)
    motors[1].setVelocity(v_right)
    motors[2].setVelocity(v_left)
    motors[3].setVelocity(v_right)

print("Para parar a simulação e mostrar os logs, aperte F")
# ---- Log de trajetoria ----
log_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "trajetoria_gps.txt")
log_file = open(log_path, "w")


# ---- Utilidades ----
def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def dist(ax, ay, bx, by):
    return math.hypot(ax - bx, ay - by)


# ---- Mapa estatico + occupancy grid + A* ----
GRID_RES = 0.35
ARENA_LIM = 29.8
WALL_FACE = 29.9
WALL_MARGIN = 1.8
INFLATE = 1.0            # raio robo + margem, somado ao raio do obstaculo
PANEL_HALF = 0.1
PATH_REACH = 0.4

# Extraidos de worlds/pioneer3at-trab-2026-v1.wbt.
# Obstaculos (x, y, raio): cilindros r=0.4, cubos box 0.5 -> r=0.35.
OBSTACLES = [
    (13.96, -28.26, 0.4), (22.01, 14.90, 0.4), (11.32, -24.59, 0.4),
    (7.14, -26.19, 0.4), (2.23, -28.08, 0.4), (3.42, -24.53, 0.4),
    (6.43, -20.88, 0.4), (16.63, -20.88, 0.4), (13.10, -20.88, 0.4),
    (21.73, 26.71, 0.4), (17.66, 18.48, 0.4), (23.64, 21.73, 0.4),
    (2.54, 8.16, 0.4), (-2.66, 5.95, 0.4), (15.02, 24.90, 0.4),
    (21.90, 6.25, 0.35), (7.69, 15.03, 0.35), (-28.31, -28.74, 0.35),
]
# Paineis como segmentos (x1, y1, x2, y2).
PANELS = [
    (10.04, -13.0, 10.04, 19.4),    # x=10 vertical (panel + panel(1) + panel(4))
    (-5.55, 11.87, 14.45, 11.87),   # panel(3) horizontal
    (-30.0, 3.48, -12.5, 3.48),     # panel(2)
]
NEIGH = [(-1, 0), (1, 0), (0, -1), (0, 1),
         (-1, -1), (-1, 1), (1, -1), (1, 1)]


def seg_dist(px, py, x1, y1, x2, y2):
    dx, dy = x2 - x1, y2 - y1
    l2 = dx * dx + dy * dy
    tp = 0.0 if l2 == 0 else max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / l2))
    return math.hypot(px - (x1 + tp * dx), py - (y1 + tp * dy))


# Occupancy grid: conversao mundo<->celula e estampagem das primitivas do mapa.
class OccupancyGrid:
    def __init__(self, res, lim):
        self.res = res
        self.lim = lim
        self.n = int(2 * lim / res) + 1
        self.cells = [[False] * self.n for _ in range(self.n)]

    def world_to_cell(self, wx, wy):
        return (int(round((wx + self.lim) / self.res)),
                int(round((wy + self.lim) / self.res)))

    def cell_center(self, col, row):
        return (-self.lim + col * self.res, -self.lim + row * self.res)

    def in_bounds(self, col, row):
        return 0 <= col < self.n and 0 <= row < self.n

    def is_free(self, col, row):
        return self.in_bounds(col, row) and not self.cells[row][col]

    def stamp_border(self, inner):
        for row in range(self.n):
            for col in range(self.n):
                wx, wy = self.cell_center(col, row)
                if abs(wx) > inner or abs(wy) > inner:
                    self.cells[row][col] = True

    def stamp_circle(self, cx, cy, radius):
        rc = int(radius / self.res) + 1
        c0, r0 = self.world_to_cell(cx, cy)
        for row in range(r0 - rc, r0 + rc + 1):
            for col in range(c0 - rc, c0 + rc + 1):
                if not self.in_bounds(col, row):
                    continue
                wx, wy = self.cell_center(col, row)
                if math.hypot(wx - cx, wy - cy) < radius:
                    self.cells[row][col] = True

    def stamp_segment(self, x1, y1, x2, y2, radius):
        rc = int(radius / self.res) + 1
        ca, ra = self.world_to_cell(x1, y1)
        cb, rb = self.world_to_cell(x2, y2)
        for row in range(min(ra, rb) - rc, max(ra, rb) + rc + 1):
            for col in range(min(ca, cb) - rc, max(ca, cb) + rc + 1):
                if not self.in_bounds(col, row):
                    continue
                wx, wy = self.cell_center(col, row)
                if seg_dist(wx, wy, x1, y1, x2, y2) < radius:
                    self.cells[row][col] = True

    def nearest_free(self, col, row):
        if self.is_free(col, row):
            return (col, row)
        for r in range(1, self.n):
            for dc in range(-r, r + 1):
                for dr in range(-r, r + 1):
                    if self.is_free(col + dc, row + dr):
                        return (col + dc, row + dr)
        return (col, row)


def build_grid():
    g = OccupancyGrid(GRID_RES, ARENA_LIM)
    g.stamp_border(WALL_FACE - WALL_MARGIN)
    for ox, oy, orad in OBSTACLES:
        g.stamp_circle(ox, oy, orad + INFLATE)
    for seg in PANELS:
        g.stamp_segment(seg[0], seg[1], seg[2], seg[3], PANEL_HALF + INFLATE)
    return g


GRID = build_grid()


# A* 8-conexo sem cortar quinas; lista de celulas ou [] se falhar.
def astar(start, goal):
    sc = GRID.nearest_free(*start)
    gc = GRID.nearest_free(*goal)
    openh = [(0.0, sc)]
    came = {}
    gscore = {sc: 0.0}
    while openh:
        _, cur = heapq.heappop(openh)
        if cur == gc:
            path = [cur]
            while cur in came:
                cur = came[cur]
                path.append(cur)
            path.reverse()
            return path
        cc, cr = cur
        for dc, dr in NEIGH:
            nc, nr = cc + dc, cr + dr
            if not GRID.is_free(nc, nr):
                continue
            if dc and dr and not (GRID.is_free(cc + dc, cr) and GRID.is_free(cc, cr + dr)):
                continue
            ng = gscore[cur] + math.hypot(dc, dr)
            if ng < gscore.get((nc, nr), float("inf")):
                gscore[(nc, nr)] = ng
                came[(nc, nr)] = cur
                h = math.hypot(nc - gc[0], nr - gc[1])
                heapq.heappush(openh, (ng + h, (nc, nr)))
    return []


def line_of_sight(a, b):
    ax, ay = GRID.cell_center(*a)
    bx, by = GRID.cell_center(*b)
    n = int(math.hypot(bx - ax, by - ay) / (GRID_RES / 3.0)) + 1
    for i in range(n + 1):
        t = i / n
        if not GRID.is_free(*GRID.world_to_cell(ax + t * (bx - ax),
                                                ay + t * (by - ay))):
            return False
    return True


# String-pulling: descarta pontos com visada reta livre (remove a escada do A*).
def simplify(cells):
    if len(cells) <= 2:
        return cells
    out = [cells[0]]
    anchor = 0
    for i in range(1, len(cells) - 1):
        if not line_of_sight(cells[anchor], cells[i + 1]):
            out.append(cells[i])
            anchor = i
    out.append(cells[-1])
    return out


def plan_path(sx, sy, gx, gy):
    cells = astar(GRID.world_to_cell(sx, sy), GRID.world_to_cell(gx, gy))
    if not cells:
        return [(gx, gy)]
    pts = [GRID.cell_center(c, r) for c, r in simplify(cells)]
    pts.append((gx, gy))
    return pts


# ---- LIDAR ----
lidar_fov = lidar.getFov()
lidar_res = lidar.getHorizontalResolution()
lidar_max = lidar.getMaxRange()


def angle_to_index(theta):
    i = (lidar_fov / 2.0 - theta) * (lidar_res - 1) / lidar_fov
    return int(max(0, min(lidar_res - 1, round(i))))


def sector_min(ranges, lo_deg, hi_deg):
    i1 = angle_to_index(math.radians(hi_deg))
    i2 = angle_to_index(math.radians(lo_deg))
    best = float("inf")
    for i in range(min(i1, i2), max(i1, i2) + 1):
        r = ranges[i]
        if r is None or math.isinf(r) or math.isnan(r):
            r = lidar_max
        best = min(best, r)
    return best


# ---- Visao computacional (deteccao de anomalias) ----
# Retorna (n_pixels, cx) do blob da cor `want`; cx in [-1,1], 0 = centro.
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


# Registro de anomalia: alerta com GPS e orientacao.
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


# ---- Estado ----
MODE_PATROL = 0
MODE_APPROACH = 1
MODE_FOLLOW_PERSON = 2
mode = MODE_PATROL
target_type = None

wp_index = 0
goal = WAYPOINTS[wp_index]
path = []
path_i = 0
need_plan = True
reacted = False

last_cube_time = -COOLDOWN
last_person_time = -COOLDOWN
follow_start = 0.0
last_follow_log = 0.0
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
    front = sector_min(ranges, -13, 13)
    image = camera.getImage()
    t = robot.getTime()

    if mode == MODE_APPROACH:
        n, cx = detect_blob(image, target_type)
        if n < LOST_MIN_PIXELS:
            mode = MODE_PATROL
            need_plan = True
            continue
        steer = KP_GOAL * (-cx)
        reached = front <= APPROACH_DIST
        fwd = 0.0 if reached else APPROACH_SPEED
        set_speed(fwd - steer, fwd + steer)
        if reached and abs(cx) < ALIGN_TOL:
            set_speed(0.0, 0.0)
            if target_type == "red":
                alerta("CUBO VERMELHO REGISTRADO (foto)", "distancia=%.2f m" % front)
                last_cube_time = t
                mode = MODE_PATROL
                need_plan = True      # recalcula A* apos registrar a anomalia
            else:
                alerta("INTRUSO (PESSOA) -- INICIANDO ACOMPANHAMENTO",
                       "distancia=%.2f m, seguira %ds" % (front, int(FOLLOW_TIME)))
                mode = MODE_FOLLOW_PERSON
                follow_start = t
                last_follow_log = t
        continue

    if mode == MODE_FOLLOW_PERSON:
        n_green, cx = detect_blob(image, "green")
        if t - follow_start >= FOLLOW_TIME:
            alerta("FIM DO ACOMPANHAMENTO DA PESSOA", "retomando rota")
            last_person_time = t
            mode = MODE_PATROL
            need_plan = True          # recalcula A* apos registrar a anomalia
            continue
        if n_green < LOST_MIN_PIXELS:
            set_speed(1.5, -1.5)
        else:
            steer = KP_GOAL * (-cx)
            fwd = max(-1.0, min(APPROACH_SPEED, 1.5 * (front - APPROACH_DIST)))
            set_speed(fwd - steer, fwd + steer)
        if t - last_follow_log >= 1.0:
            print("  [SEGUINDO PESSOA t=%.1fs] GPS=(%.2f,%.2f) dist=%.2fm blob=%d px"
                  % (t, x, y, front, n_green))
            last_follow_log = t
        continue

    n_green, _ = detect_blob(image, "green")
    if n_green >= DETECT_MIN_PIXELS and t - last_person_time >= COOLDOWN:
        target_type = "green"
        mode = MODE_APPROACH
        continue

    n_red, _ = detect_blob(image, "red")
    if n_red >= DETECT_MIN_PIXELS and t - last_cube_time >= COOLDOWN:
        target_type = "red"
        mode = MODE_APPROACH
        continue

    if need_plan:
        path = plan_path(x, y, goal[0], goal[1])
        path_i = 0
        need_plan = False
        print("[NAV] Indo p/ waypoint #%d = (%.1f, %.1f) | rota A* %d pontos"
              % (wp_index, goal[0], goal[1], len(path)))

    tx, ty = path[path_i]
    if dist(x, y, tx, ty) < PATH_REACH:
        if path_i < len(path) - 1:
            path_i += 1
            tx, ty = path[path_i]
        else:
            wp_index = (wp_index + 1) % len(WAYPOINTS)
            goal = WAYPOINTS[wp_index]
            need_plan = True
            print("[NAV] Waypoint #%d atingido." % wp_index)

    fL = sector_min(ranges, 5, 55)
    fR = sector_min(ranges, -55, -5)

    # Reativo anti-colisao; ao sair do desvio o A* recalcula a rota.
    if min(front, fL, fR) < SAFE_STOP:
        if fL < fR:
            set_speed(CRUISE * 0.25, -CRUISE * 0.2)
        else:
            set_speed(-CRUISE * 0.2, CRUISE * 0.25)
        reacted = True
    elif front < ACTIVATE_DIST:
        set_speed(CRUISE * 0.3, -CRUISE * 0.1)
        reacted = True
    else:
        if reacted:
            need_plan = True
            reacted = False
        err = wrap(math.atan2(ty - y, tx - x) - yaw)
        fwd = CRUISE * max(0.0, math.cos(err))   # desacelera na curva p/ nao cortar a quina
        set_speed(fwd - KP_GOAL * err, fwd + KP_GOAL * err)

# ---- Encerramento ----
set_speed(0.0, 0.0)
log_file.close()
if finished:
    print("[INFO] Simulacao encerrada pelo operador (tecla F).")
