# run_on_spike_fixed_v2.py
# -----------------------------------------------------------------------------
# Grid Explorer for LEGO Spike Prime
#
# Main design choices:
# 1) One simple full-grid boustrophedon traversal only.
# 2) Keep the existing 180-degree turning approach: two 90-degree turns.
# 3) Never intentionally move out of the grid.
# 4) Treat valid in-bounds ultrasonic hits as box detections and keep them.
# 5) If a waypoint becomes unreachable, skip it and continue exploration.
# 6) Stop early and return home as soon as 2 red cells and 2 boxes are found.
# -----------------------------------------------------------------------------

from spike import PrimeHub, MotorPair, DistanceSensor, ColorSensor
from spike.control import wait_for_seconds, Timer

# -----------------------------------------------------------------------------
# USER CONFIGURATION
# -----------------------------------------------------------------------------

START_PROFILE = 1
# 1 = start at (0,0), front ultrasonic sensor (port B) points toward column 5 (+X)
# 2 = start at (5,3), front ultrasonic sensor (port B) points toward column 0 (-X)

# Ports
PORT_MOTOR_LEFT = 'A'
PORT_MOTOR_RIGHT = 'C'
PORT_ULTRASONIC_FRONT = 'B'
PORT_ULTRASONIC_SIDE = 'F'
PORT_COLOR = 'D'

# Grid geometry
GRID_COLS = 6
GRID_ROWS = 4
CELL_DIST_CM = 23

# Motion
DRIVE_SPEED = 30
MAX_CELL_TRAVEL_SECONDS = 5.0
CELL_CENTER_DRIVE_SECONDS = 0.01

# Turn control
HEADING_TOLERANCE_DEG = 1
TURN_SETTLE_SECONDS = 0.50

# Sensors / labels
RED_COLOR_LABEL = 'red'
BORDER_COLOR_LABEL = 'black'

# This was reduced from 15 cm to 10 cm to avoid over-reacting to noisy short
# readings that are not really a box in the next cell.
DIRECT_NEIGHBOR_THRESHOLD_CM = 10.0

# Extra safety while the robot is already moving forward. If the front sensor
# suddenly reports a very near obstacle several times in a row, stop the move
# before touching the box.
EMERGENCY_FRONT_STOP_CM = 7.0
EMERGENCY_FRONT_STOP_CONFIRMATIONS = 2

# Exploration budget
TIME_BUDGET_SECONDS = None

# Assignment target counts
REQUIRED_RED_CELLS = 2
REQUIRED_BOX_CELLS = 2

# -----------------------------------------------------------------------------
# CONSTANTS
# -----------------------------------------------------------------------------

UNKNOWN = 'U'
EMPTY = 'E'
RED_STATE = 'R'
BOX = 'B'

START_PROFILES = {
    1: (0, 0, 'E'),
    2: (5, 3, 'W'),
}

found_objects = []


def direction_name(facing):
    return {
        'N': 'North',
        'S': 'South',
        'E': 'East',
        'W': 'West',
    }.get(facing, facing)


def facing_axis_label(facing):
    return {
        'E': '+X',
        'W': '-X',
        'N': '+Y',
        'S': '-Y',
    }.get(facing, facing)


def start_profile_description(start_x, start_y, start_facing):
    target_text = {
        'E': 'col 5 (+X direction)',
        'W': 'col 0 (-X direction)',
        'N': 'row 3 (+Y direction)',
        'S': 'row 0 (-Y direction)',
    }.get(start_facing, facing_axis_label(start_facing))
    return 'Start: ({},{}) | Front sensor -> {}'.format(start_x, start_y, target_text)


# -----------------------------------------------------------------------------
# GRID MAP
# -----------------------------------------------------------------------------

class GridMap:
    def __init__(self):
        self._cells = [[UNKNOWN] * GRID_ROWS for _ in range(GRID_COLS)]

    def in_bounds(self, x, y):
        return 0 <= x < GRID_COLS and 0 <= y < GRID_ROWS

    def get(self, x, y):
        if not self.in_bounds(x, y):
            return None
        return self._cells[x][y]

    def set_empty_if_unknown(self, x, y):
        if self.in_bounds(x, y) and self._cells[x][y] == UNKNOWN:
            self._cells[x][y] = EMPTY

    def mark_red(self, x, y):
        if not self.in_bounds(x, y):
            return False
        old = self._cells[x][y]
        self._cells[x][y] = RED_STATE
        return old != RED_STATE

    def mark_box(self, x, y):
        if not self.in_bounds(x, y):
            return False
        old = self._cells[x][y]
        self._cells[x][y] = BOX
        return old != BOX

    def mark_empty(self, x, y):
        if not self.in_bounds(x, y):
            return
        if self._cells[x][y] in (BOX, RED_STATE):
            return
        self._cells[x][y] = EMPTY

    def is_obstacle(self, x, y):
        return self.in_bounds(x, y) and self._cells[x][y] == BOX

    def red_cells(self):
        cells = []
        for x in range(GRID_COLS):
            for y in range(GRID_ROWS):
                if self._cells[x][y] == RED_STATE:
                    cells.append((x, y))
        return cells

    def box_cells(self):
        cells = []
        for x in range(GRID_COLS):
            for y in range(GRID_ROWS):
                if self._cells[x][y] == BOX:
                    cells.append((x, y))
        return cells

    def targets_complete(self):
        return len(self.red_cells()) >= REQUIRED_RED_CELLS and len(self.box_cells()) >= REQUIRED_BOX_CELLS

    def report_findings(self):
        for x in range(GRID_COLS):
            for y in range(GRID_ROWS):
                s = self._cells[x][y]
                if s == RED_STATE:
                    print('({}, {}, R)'.format(x, y))
                elif s == BOX:
                    print('({}, {}, B)'.format(x, y))

    def debug_print(self):
        for y in range(GRID_ROWS - 1, -1, -1):
            row = ' '.join('{:2s}'.format(self._cells[x][y]) for x in range(GRID_COLS))
            print('y={}: {}'.format(y, row))


def print_grid_map(grid_map, robot_x, robot_y):
    symbols = {UNKNOWN: '??', EMPTY: '', RED_STATE: 'RR', BOX: 'BB'}
    print('[MAP]')
    for y in range(GRID_ROWS - 1, -1, -1):
        row_parts = []
        for x in range(GRID_COLS):
            if x == robot_x and y == robot_y:
                row_parts.append('@@')
            else:
                row_parts.append(symbols.get(grid_map.get(x, y), '??'))
        print('y{}| {} |'.format(y, ''.join(row_parts)))
    print('    ' + ''.join('x{}'.format(x) for x in range(GRID_COLS)))


# -----------------------------------------------------------------------------
# HARDWARE
# -----------------------------------------------------------------------------

def init_hardware():
    hub = PrimeHub()
    motors = MotorPair(PORT_MOTOR_LEFT, PORT_MOTOR_RIGHT)
    front_us = DistanceSensor(PORT_ULTRASONIC_FRONT)
    side_us = DistanceSensor(PORT_ULTRASONIC_SIDE)
    color = ColorSensor(PORT_COLOR)
    return {
        'hub': hub,
        'motors': motors,
        'front_us': front_us,
        'side_us': side_us,
        'color': color,
    }


def median_distance_cm(sensor):
    readings = []
    for _ in range(5):
        d = sensor.get_distance_cm()
        if d is not None:
            readings.append(d)
    if not readings:
        return None
    readings.sort()
    return readings[len(readings) // 2]


def read_front_distance_cm(hw):
    return median_distance_cm(hw['front_us'])


def read_side_distance_cm(hw):
    return median_distance_cm(hw['side_us'])


def read_color(hw):
    return hw['color'].get_color()


def _drive_cancelled_by_front_sensor(hw):
    if EMERGENCY_FRONT_STOP_CM is None or EMERGENCY_FRONT_STOP_CM <= 0:
        return False

    if '_front_stop_streak' not in hw:
        hw['_front_stop_streak'] = 0

    dist = hw['front_us'].get_distance_cm()
    if dist is not None and 0 < dist <= EMERGENCY_FRONT_STOP_CM:
        hw['_front_stop_streak'] += 1
    else:
        hw['_front_stop_streak'] = 0

    if hw['_front_stop_streak'] >= EMERGENCY_FRONT_STOP_CONFIRMATIONS:
        hw['motors'].stop()
        print('Emergency stop: while moving forward, the front sensor repeatedly detected an obstacle about {} cm away, so the robot cancelled the move to avoid touching a box.'.format(EMERGENCY_FRONT_STOP_CM))
        hw['_front_stop_streak'] = 0
        return True

    return False


def _drive_pd(hw, state):
    hub = hw["hub"]

    KP = 4.0
    KI = 0.0
    KD = 1.2
    BASE_SPEED = 40

    current = hub.motion_sensor.get_yaw_angle()
    error = -current

    state["integral"] += error
    derivative = error - state["last_error"]

    correction = KP * error + KI * state["integral"] + KD * derivative

    state["last_error"] = error

    left_speed = BASE_SPEED + correction
    right_speed = BASE_SPEED - correction

    # clamp
    left_speed = max(min(int(left_speed), 100), -100)
    right_speed = max(min(int(right_speed), 100), -100)

    return left_speed, right_speed



def move_forward_one_cell(hw):
    t = Timer()
    hw['_front_stop_streak'] = 0

    hub = hw["hub"]
    motors = hw['motors']
    hub.motion_sensor.reset_yaw_angle()

    #PD state
    state = {
        "integral": 0,
        "last_error": 0
    }

    motors.start_tank(DRIVE_SPEED, DRIVE_SPEED)

    t.reset()
    while hw['color'].get_color() == BORDER_COLOR_LABEL or t.now() < 1.0:
        left_speed, right_speed = _drive_pd(hw, state)
        motors.start_tank(left_speed, right_speed)
        if _drive_cancelled_by_front_sensor(hw):
            return False
        wait_for_seconds(0.01)

    if hw['color'].get_color() == BORDER_COLOR_LABEL:
        hw['motors'].stop()
        print('The robot could not leave the current border line, so this cell move was cancelled.')
        return False

    t.reset()
    while hw['color'].get_color() != BORDER_COLOR_LABEL and t.now() < MAX_CELL_TRAVEL_SECONDS:
        left_speed, right_speed = _drive_pd(hw, state)
        motors.start_tank(left_speed, right_speed)
        if _drive_cancelled_by_front_sensor(hw):
            return False
        wait_for_seconds(0.01)

    if hw['color'].get_color() != BORDER_COLOR_LABEL:
        hw['motors'].stop()
        print('The robot did not detect the next border within {:.1f} seconds, so this cell move was cancelled.'.format(MAX_CELL_TRAVEL_SECONDS))
        return False

    t.reset()
    while hw['color'].get_color() == BORDER_COLOR_LABEL and t.now() < 1.0:
        left_speed, right_speed = _drive_pd(hw, state)
        motors.start_tank(left_speed, right_speed)
        if _drive_cancelled_by_front_sensor(hw):
            return False
        wait_for_seconds(0.01)

    if hw['color'].get_color() == BORDER_COLOR_LABEL:
        hw['motors'].stop()
        print('The robot remained stuck on the border line, so this cell move was cancelled.')
        return False
    """
    t.reset()
    while t.now() < CELL_CENTER_DRIVE_SECONDS:
        left_speed, right_speed = _drive_pd(hw, state)
        motors.start_tank(left_speed, right_speed)
        if _drive_cancelled_by_front_sensor(hw):
            return False
        wait_for_seconds(0.01)
    """
    motors.stop()
    hw['_front_stop_streak'] = 0
    return True

# -----------------------------------------------------------------------------
# TURNING
# -----------------------------------------------------------------------------

def _gyro_turn(hw, target_angle):
    hub = hw['hub']
    motors = hw['motors']

    kp = 0.8
    ki = 0.012
    kd = 1.0
    min_speed = 1
    max_speed = 50
    required_stable_loops = 5

    integral = 0
    last_error = 0
    stable_count = 0

    hub.motion_sensor.reset_yaw_angle()

    while True:
        current = hub.motion_sensor.get_yaw_angle()
        error = target_angle - current

        if abs(error) <= HEADING_TOLERANCE_DEG:
            stable_count += 1
        else:
            stable_count = 0

        if stable_count >= required_stable_loops:
            break

        integral += error
        if integral > 100:
            integral = 100
        elif integral < -100:
            integral = -100

        derivative = error - last_error
        speed = int(kp * error + ki * integral + kd * derivative)
        last_error = error

        if speed > max_speed:
            speed = max_speed
        elif speed < -max_speed:
            speed = -max_speed

        if 0 < speed < min_speed:
            speed = min_speed
        elif -min_speed < speed < 0:
            speed = -min_speed

        motors.start_tank(speed, -speed)
        wait_for_seconds(0.01)

    motors.stop()
    wait_for_seconds(TURN_SETTLE_SECONDS)


def turn_left_90(hw):
    _gyro_turn(hw, -91)


def turn_right_90(hw):
    _gyro_turn(hw, 91)


def turn_180(hw):
    turn_right_90(hw)
    turn_right_90(hw)


# -----------------------------------------------------------------------------
# GEOMETRY / PATH HELPERS
# -----------------------------------------------------------------------------

def _facing_to_delta(facing):
    return {
        'E': (1, 0),
        'W': (-1, 0),
        'N': (0, 1),
        'S': (0, -1),
    }[facing]


def _turn_left_facing(facing):
    return {
        'E': 'N',
        'N': 'W',
        'W': 'S',
        'S': 'E',
    }[facing]


def _delta_to_facing(dx, dy):
    if dx == 1:
        return 'E'
    if dx == -1:
        return 'W'
    if dy == 1:
        return 'N'
    if dy == -1:
        return 'S'
    return 'E'


def _forward_cell(x, y, facing):
    dx, dy = _facing_to_delta(facing)
    return x + dx, y + dy


def _distance_to_cells(dist_cm):
    if dist_cm is None or dist_cm <= 0:
        return None
    if dist_cm <= DIRECT_NEIGHBOR_THRESHOLD_CM:
        return 1
    return int((dist_cm + (CELL_DIST_CM / 2.0)) / CELL_DIST_CM)


# -----------------------------------------------------------------------------
# DETECTION
# -----------------------------------------------------------------------------

def _classify_distance_reading(dist_cm, robot_x, robot_y, facing):
    cells_away = _distance_to_cells(dist_cm)
    if cells_away is None or cells_away < 1:
        return None

    dx, dy = _facing_to_delta(facing)
    tx = robot_x + dx * cells_away
    ty = robot_y + dy * cells_away

    if not (0 <= tx < GRID_COLS and 0 <= ty < GRID_ROWS):
        return None

    return (tx, ty)


def _mark_clear_cells_before_obstacle(grid_map, robot_x, robot_y, facing, dist_cm):
    cells_away = _distance_to_cells(dist_cm)
    if cells_away is None or cells_away <= 1:
        return

    dx, dy = _facing_to_delta(facing)
    for step in range(1, cells_away):
        cx = robot_x + dx * step
        cy = robot_y + dy * step
        if grid_map.in_bounds(cx, cy):
            grid_map.mark_empty(cx, cy)


def detect_from_sensors(grid_map, x, y, sensor_data, found_objects):
    facing = sensor_data.get('facing', 'E')
    front = sensor_data.get('front_dist_cm')
    side = sensor_data.get('side_dist_cm')
    color = sensor_data.get('color')

    if color is not None and color.lower() == RED_COLOR_LABEL.lower():
        if grid_map.mark_red(x, y):
            red_obj = (x, y, 'R')
            if red_obj not in found_objects:
                found_objects.append(red_obj)
            print('The robot found a red cell at coordinate ({}, {}).'.format(x, y))
    else:
        grid_map.set_empty_if_unknown(x, y)

    front_hit = _classify_distance_reading(front, x, y, facing)
    side_facing = _turn_left_facing(facing)
    side_hit = _classify_distance_reading(side, x, y, side_facing)

    if front_hit is not None:
        fx, fy = front_hit
        if grid_map.mark_box(fx, fy):
            box_obj = (fx, fy, 'B')
            if box_obj not in found_objects:
                found_objects.append(box_obj)
            print('The robot detected a box directly ahead at coordinate ({}, {}).'.format(fx, fy))
        _mark_clear_cells_before_obstacle(grid_map, x, y, facing, front)

    if side_hit is not None:
        sx, sy = side_hit
        if grid_map.mark_box(sx, sy):
            box_obj = (sx, sy, 'B')
            if box_obj not in found_objects:
                found_objects.append(box_obj)
            print('The robot detected a box on its left side at coordinate ({}, {}).'.format(sx, sy))
        _mark_clear_cells_before_obstacle(grid_map, x, y, side_facing, side)

    if front_hit is not None and side_hit is not None and front_hit != side_hit:
        print('The robot detected two different boxes in this scan: one ahead at ({}, {}) and one on the left at ({}, {}).'.format(
            front_hit[0], front_hit[1], side_hit[0], side_hit[1]
        ))


# -----------------------------------------------------------------------------
# BFS
# -----------------------------------------------------------------------------

def bfs_path(grid_map, sx, sy, gx, gy):
    queue = [(sx, sy)]
    came_from = {(sx, sy): None}

    while queue:
        cx, cy = queue.pop(0)
        if (cx, cy) == (gx, gy):
            path = []
            node = (gx, gy)
            while node is not None:
                path.append(node)
                node = came_from[node]
            path.reverse()
            return path

        for nx, ny in ((cx + 1, cy), (cx - 1, cy), (cx, cy + 1), (cx, cy - 1)):
            if not grid_map.in_bounds(nx, ny):
                continue
            if grid_map.is_obstacle(nx, ny):
                continue
            if (nx, ny) in came_from:
                continue
            came_from[(nx, ny)] = (cx, cy)
            queue.append((nx, ny))

    return []


# -----------------------------------------------------------------------------
# SIMPLE FULL-GRID TRAVERSAL
# -----------------------------------------------------------------------------

class FullGridTraversal:
    NAME = 'Simple full-grid boustrophedon'

    def __init__(self):
        self._waypoints = []
        self._index = 0
        self._visited_targets = {}

    def start(self, sx, sy):
        path = []

        if (sx, sy) == (0, 0):
            for y in range(GRID_ROWS):
                if y % 2 == 0:
                    xs = range(GRID_COLS)
                else:
                    xs = range(GRID_COLS - 1, -1, -1)
                for x in xs:
                    path.append((x, y))
        else:
            row_counted_from_top = 0
            for y in range(GRID_ROWS - 1, -1, -1):
                if row_counted_from_top % 2 == 0:
                    xs = range(GRID_COLS - 1, -1, -1)
                else:
                    xs = range(GRID_COLS)
                for x in xs:
                    path.append((x, y))
                row_counted_from_top += 1

        self._waypoints = path
        self._index = 0
        self._visited_targets = {}

        while self._index < len(self._waypoints) and self._waypoints[self._index] == (sx, sy):
            self._index += 1

    def next_target(self, grid_map, current_x, current_y):
        # Still follows the planned order, but automatically skips targets that
        # are now known obstacles or unreachable from the current robot position.
        while self._index < len(self._waypoints):
            tx, ty = self._waypoints[self._index]
            self._index += 1

            if grid_map.is_obstacle(tx, ty):
                continue

            path = bfs_path(grid_map, current_x, current_y, tx, ty)
            if len(path) < 2 and (current_x, current_y) != (tx, ty):
                print('The coordinate ({}, {}) is currently unreachable, so it will be skipped.'.format(tx, ty))
                continue

            return (tx, ty)

        return None


# -----------------------------------------------------------------------------
# LOGGING
# -----------------------------------------------------------------------------

def _read_sensor_snapshot(hw, facing):
    return {
        'front_dist_cm': read_front_distance_cm(hw),
        'side_dist_cm': read_side_distance_cm(hw),
        'color': read_color(hw),
        'facing': facing,
    }


def log_robot_state(x, y, facing, sensor_data=None):
    print('The robot is currently at coordinate ({}, {}) and the front sensor points along {}.'.format(
        x, y, facing_axis_label(facing)
    ))
    if sensor_data is not None:
        print('Front sensor distance: {} cm. Side sensor distance: {} cm. Floor color: {}.'.format(
            sensor_data.get('front_dist_cm'),
            sensor_data.get('side_dist_cm'),
            sensor_data.get('color')
        ))


def log_target_counts(grid_map):
    print('The robot has currently found {} red cell(s) and {} box(es).'.format(
        len(grid_map.red_cells()),
        len(grid_map.box_cells())
    ))


# -----------------------------------------------------------------------------
# MOVEMENT / EXPLORATION
# -----------------------------------------------------------------------------

def _turn_to(hw, current_facing, target_facing):
    clockwise = ['E', 'S', 'W', 'N']
    facing = current_facing

    while facing != target_facing:
        ci = clockwise.index(facing)
        ti = clockwise.index(target_facing)
        diff = (ti - ci) % 4

        if diff == 1:
            turn_right_90(hw)
            facing = clockwise[(ci + 1) % 4]
        elif diff == 3:
            turn_left_90(hw)
            facing = clockwise[(ci - 1) % 4]
        elif diff == 2:
            turn_180(hw)
            facing = clockwise[(ci + 2) % 4]
        else:
            break

    return facing


def move_to(hw, grid_map, current_x, current_y, facing, target_x, target_y, found_objects):
    cx, cy, f = current_x, current_y, facing

    while (cx, cy) != (target_x, target_y):
        path = bfs_path(grid_map, cx, cy, target_x, target_y)
        if len(path) < 2:
            print('There is no safe path to coordinate ({}, {}).'.format(target_x, target_y))
            return cx, cy, f, False

        nx, ny = path[1]
        target_facing = _delta_to_facing(nx - cx, ny - cy)

        if target_facing != f:
            print('The robot will turn from {} to {}.'.format(
                facing_axis_label(f), facing_axis_label(target_facing)
            ))
        f = _turn_to(hw, f, target_facing)

        pre_move_sensor_data = _read_sensor_snapshot(hw, f)
        detect_from_sensors(grid_map, cx, cy, pre_move_sensor_data, found_objects)
        log_robot_state(cx, cy, f, pre_move_sensor_data)
        log_target_counts(grid_map)

        fx, fy = _forward_cell(cx, cy, f)
        if (fx, fy) != (nx, ny):
            print('Safety stop: the internal forward step does not match the planned next coordinate.')
            return cx, cy, f, False
        if not grid_map.in_bounds(fx, fy):
            print('Safety stop: moving forward from ({}, {}) while facing {} would leave the grid.'.format(
                cx, cy, facing_axis_label(f)
            ))
            return cx, cy, f, False

        if grid_map.is_obstacle(nx, ny):
            print('The next coordinate ({}, {}) is blocked by a box, so the robot will replan.'.format(nx, ny))
            continue

        print('The robot will move from ({}, {}) to ({}, {}).'.format(cx, cy, nx, ny))
        if not move_forward_one_cell(hw):
            fail_sensor_data = _read_sensor_snapshot(hw, f)
            detect_from_sensors(grid_map, cx, cy, fail_sensor_data, found_objects)
            log_robot_state(cx, cy, f, fail_sensor_data)
            log_target_counts(grid_map)
            return cx, cy, f, False

        cx, cy = nx, ny
        print('[CELL] Now at ({}, {})'.format(cx, cy))
        print_grid_map(grid_map, cx, cy)

        post_move_sensor_data = _read_sensor_snapshot(hw, f)
        detect_from_sensors(grid_map, cx, cy, post_move_sensor_data, found_objects)
        log_robot_state(cx, cy, f, post_move_sensor_data)
        log_target_counts(grid_map)

        if grid_map.targets_complete():
            return cx, cy, f, True

    return cx, cy, f, True


def return_to_start(hw, grid_map, current_x, current_y, facing, start_x, start_y, found_objects):
    print('The robot will now return to the starting coordinate ({}, {}).'.format(start_x, start_y))
    cx, cy, f = current_x, current_y, facing

    while (cx, cy) != (start_x, start_y):
        path = bfs_path(grid_map, cx, cy, start_x, start_y)
        if len(path) < 2:
            print('The robot could not find a safe path back to the start.')
            return f, False

        nx, ny = path[1]
        target_facing = _delta_to_facing(nx - cx, ny - cy)
        if target_facing != f:
            print('The robot will turn from {} to {} while returning home.'.format(
                facing_axis_label(f), facing_axis_label(target_facing)
            ))
        f = _turn_to(hw, f, target_facing)

        pre_move_sensor_data = _read_sensor_snapshot(hw, f)
        detect_from_sensors(grid_map, cx, cy, pre_move_sensor_data, found_objects)
        log_robot_state(cx, cy, f, pre_move_sensor_data)

        fx, fy = _forward_cell(cx, cy, f)
        if (fx, fy) != (nx, ny):
            print('Safety stop: the internal forward step does not match the planned return coordinate.')
            return f, False
        if not grid_map.in_bounds(fx, fy):
            print('Safety stop: the return move would leave the grid.')
            return f, False
        if grid_map.is_obstacle(nx, ny):
            print('The return path is blocked at ({}, {}), so the robot will replan.'.format(nx, ny))
            continue

        print('The robot will return from ({}, {}) to ({}, {}).'.format(cx, cy, nx, ny))
        if not move_forward_one_cell(hw):
            fail_sensor_data = _read_sensor_snapshot(hw, f)
            detect_from_sensors(grid_map, cx, cy, fail_sensor_data, found_objects)
            log_robot_state(cx, cy, f, fail_sensor_data)
            return f, False

        cx, cy = nx, ny
        print('[CELL] Now at ({}, {})'.format(cx, cy))
        print_grid_map(grid_map, cx, cy)

        post_move_sensor_data = _read_sensor_snapshot(hw, f)
        detect_from_sensors(grid_map, cx, cy, post_move_sensor_data, found_objects)
        log_robot_state(cx, cy, f, post_move_sensor_data)

    print('The robot successfully returned to the starting coordinate.')
    return f, True


def run_exploration(hw, start_x, start_y, start_facing):
    global found_objects
    found_objects = []
    grid_map = GridMap()
    traversal = FullGridTraversal()
    traversal.start(start_x, start_y)

    cx, cy, facing = start_x, start_y, start_facing

    timer = Timer()
    timer.reset()
    shown_start_state = False

    while True:
        sensor_data = _read_sensor_snapshot(hw, facing)
        detect_from_sensors(grid_map, cx, cy, sensor_data, found_objects)
        if not shown_start_state:
            print('[CELL] Now at ({}, {})'.format(cx, cy))
            print_grid_map(grid_map, cx, cy)
            shown_start_state = True
        log_robot_state(cx, cy, facing, sensor_data)
        log_target_counts(grid_map)

        if grid_map.targets_complete():
            print('All required targets have been found. The robot will now return home.')
            break

        elapsed = timer.now()
        print('Elapsed exploration time: {:.1f} seconds.'.format(elapsed))
        if TIME_BUDGET_SECONDS is not None and elapsed >= TIME_BUDGET_SECONDS:
            print('The time budget has been reached, so exploration will stop and the robot will return home.')
            break

        next_target = traversal.next_target(grid_map, cx, cy)
        if next_target is None:
            print('There are no more reachable planned coordinates to visit.')
            break

        tx, ty = next_target
        print('The next planned coordinate is ({}, {}).'.format(tx, ty))

        cx, cy, facing, reached = move_to(hw, grid_map, cx, cy, facing, tx, ty, found_objects)

        if grid_map.targets_complete():
            print('All required targets have been found. The robot will now return home.')
            break

        if not reached:
            print('The robot could not safely reach coordinate ({}, {}), so that coordinate will be skipped and exploration will continue.'.format(tx, ty))
            continue

    facing, returned_home = return_to_start(hw, grid_map, cx, cy, facing, start_x, start_y, found_objects)
    if not returned_home:
        print('The run ended before the robot could return all the way to the start.')

    print('\n--- Final Findings (required format) ---')
    grid_map.report_findings()
    print('')
    print('=== FOUND OBJECTS SUMMARY ===')
    for obj in found_objects:
        print('({}, {}, {})'.format(obj[0], obj[1], obj[2]))
    reds = sum(1 for obj in found_objects if obj[2] == 'R')
    boxes = sum(1 for obj in found_objects if obj[2] == 'B')
    print('Total: {} red cell(s), {} box(es)'.format(reds, boxes))


# -----------------------------------------------------------------------------
# ENTRY POINT
# -----------------------------------------------------------------------------

def main():
    hw = init_hardware()
    start_x, start_y, start_facing = START_PROFILES[START_PROFILE]

    print('=== Grid Explorer ===')
    print('Traversal method: Simple full-grid boustrophedon with reachable-waypoint skipping.')
    print(start_profile_description(start_x, start_y, start_facing))
    print('The robot will return home as soon as it finds {} red cells and {} boxes.'.format(
        REQUIRED_RED_CELLS,
        REQUIRED_BOX_CELLS
    ))
    print('Press the LEFT button to begin.')

    hw['hub'].light_matrix.write(str(START_PROFILE))
    hw['hub'].left_button.wait_until_pressed()
    hw['hub'].light_matrix.off()

    run_exploration(hw, start_x, start_y, start_facing)


main()
