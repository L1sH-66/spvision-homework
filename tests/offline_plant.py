"""Supplied simulator/plant + the exact production C++ cores, without ROS.

Uses the unchanged SimRobot / sim_map code and the closed-source _dyn_core plant.
Navigation parameters are read from src/sp_nav_bringup/config/nav_params.yaml, so the
test always exercises the configuration that ROS will load.  Timing mirrors the real
stack: simulator/odometry 30 Hz, controller 50 Hz, re-planning 10 Hz; the arrival
judgement is the one in default_nav_with_fallback.xml (0.05 m, < 0.06 m/s, held 0.3 s),
after which the controller is disabled and the rest position is measured 2 s later.

Usage: python3 tests/offline_plant.py [max_speed ...]   (no argument: value from the yaml)
       python3 tests/offline_plant.py --mode pursuit
       python3 tests/offline_plant.py --latency 0.025 --set dead_time=0.32
         --latency L : extra odometry/TF age and command transport delay of L seconds each
         --set k=v   : override a controller parameter (robustness / mismatch tests)
Python 3.10 loads the plant directly; on other versions a python3.10 interpreter
(env PLANT_PYTHON, default "python3.10") serves it through tests/plant_server.py.
It does not run ROS, DDS, TF, the BT engine or RViz: times are simulated, not official.
"""
import ctypes as C
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import types
import numpy as np
import cv2
import yaml
sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[1]
SIM = ROOT / 'src/sp_nav_sim'
sys.path.insert(0, str(SIM))

if sys.version_info[:2] != (3, 10):
    import itertools
    _proc = subprocess.Popen([os.environ.get('PLANT_PYTHON', 'python3.10'), '-I', str(ROOT / 'tests/plant_server.py'),
                              str(SIM / 'sp_nav_sim/sim')], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)
    _ids = itertools.count()

    def _call(line):
        _proc.stdin.write(line + '\n'); _proc.stdin.flush()
        return _proc.stdout.readline().strip()

    class VelocityFilter:
        def __init__(self, dt): self.i = str(next(_ids)); _call(f'N {self.i} {float(dt)!r}')
        def reset(self): _call(f'R {self.i}')
        def stopping_lag(self): return float(_call(f'L {self.i}'))
        def set_vel(self, vx, vy): _call(f'V {self.i} {float(vx)!r} {float(vy)!r}')
        def step(self, *a):
            x, y = _call('S ' + self.i + ' ' + ' '.join(repr(float(v)) for v in a)).split()
            return float(x), float(y)
    import sp_nav_sim.sim
    sp_nav_sim.sim._dyn = sys.modules['sp_nav_sim.sim._dyn'] = types.SimpleNamespace(VelocityFilter=VelocityFilter)

from sp_nav_sim.sim.sim_robot import SimRobot
from sp_nav_sim.sim.sim_map import load_sim_map

out = ROOT / 'test-output'; out.mkdir(exist_ok=True)
subprocess.run(['g++', '-std=c++17', '-O2', '-shared', '-fPIC', str(ROOT / 'tests/core_bridge.cpp'),
                '-I' + str(ROOT / 'src/sp_controller_server/include'), '-I' + str(ROOT / 'src/sp_global_planner/include'),
                '-o', str(out / 'core.so')], check=True)
lib = C.CDLL(str(out / 'core.so')); D = C.POINTER(C.c_double)
lib.make_plan.argtypes = [C.POINTER(C.c_int8), C.c_int, C.c_int, *([C.c_double] * 4), C.c_int, C.c_double, C.c_double, C.c_int, C.c_int, C.c_double, D, C.c_int]
lib.set_param.argtypes = [C.c_char_p, C.c_double]; lib.set_param.restype = C.c_int
lib.set_plan.argtypes = [D, C.c_int]; lib.compute.argtypes = [*([C.c_double] * 7), D]

params = yaml.safe_load((ROOT / 'src/sp_nav_bringup/config/nav_params.yaml').read_text())
esdf = params['esdf_map_publisher']['ros__parameters']
astar = params['planner_server']['ros__parameters']['AStar']
ctrl = params['controller_server']['ros__parameters']['PidController']
sim = yaml.safe_load((SIM / 'config/sim_robot.yaml').read_text())['sim_robot_node']['ros__parameters']

_, occ, meta, _ = load_sim_map(str(ROOT / 'src/sp_nav_bringup/map/maze_map.yaml'), False)
h, w = occ.shape
res = meta.resolution


def inflate(d, radius, d_safe, gain):
    """sp_map_server / sim local map formula: lethal 100, smooth decay to 0 at d_safe."""
    soft = np.minimum(99, np.rint(100 * (1 - np.exp(-gain * (d_safe - d) ** 2))))
    return np.where(d < radius, 100, np.where(d >= d_safe, 0, soft)).astype(np.int8)


dist = cv2.distanceTransform((~occ).astype(np.uint8) * 255, cv2.DIST_L2, 5) * res
cost = np.ascontiguousarray(np.flipud(inflate(dist, esdf['robot_radius'] + esdf['margin'], esdf['d_safe'], esdf['w'])))
PEERS = []
TIMING = {'plan_ms': [], 'control_us': []}


def fused_cost(pos):
    # Reproduce the simulator's 4 m local map (walls + peer disks) and the planner's max-fusion.
    ox = max(0., math.floor(max(0., pos[0] - 2) / res + 1e-9) * res)
    oy = max(0., math.floor(max(0., pos[1] - 2) / res + 1e-9) * res)
    lw = int(math.floor((min(15., pos[0] + 2) - ox) / res + 1e-9))
    lh = int(math.floor((min(15., pos[1] + 2) - oy) / res + 1e-9))
    yy, xx = np.mgrid[:lh, :lw]; wx = ox + (xx + .5) * res; wy = oy + (yy + .5) * res
    px = np.rint(wx / res).astype(int); py = np.rint((h - 1) - wy / res).astype(int)
    valid = (px >= 0) & (py >= 0) & (px < w) & (py < h)
    mask = np.zeros((lh, lw), bool); mask[valid] = occ[py[valid], px[valid]]
    for bx, by in [p.pos for p in PEERS]:
        mask |= (wx - bx) ** 2 + (wy - by) ** 2 <= sim['robot_radius'] ** 2
    ld = cv2.distanceTransform((~mask).astype(np.uint8) * 255, cv2.DIST_L2, 5) * res
    lc = inflate(ld, sim['robot_radius'] + sim['margin'], sim['d_safe'], sim['w'])
    merged = cost.copy(); ix = int(round(ox / res)); iy = int(round(oy / res))
    merged[iy:iy + lh, ix:ix + lw] = np.maximum(merged[iy:iy + lh, ix:ix + lw], lc)
    return merged


def plan(pos):
    buf = np.zeros((w * h, 2), dtype=np.float64)
    merged = fused_cost(pos)
    t0 = time.perf_counter()
    n = lib.make_plan(merged.ctypes.data_as(C.POINTER(C.c_int8)), w, h, *[float(a / res) for a in (*pos, 14.1, 14.1)],
                      int(astar['lethal_cost']), float(astar['cost_weight']), float(astar.get('smooth_spacing', .1)) / res,
                      int(astar.get('smooth_max_cost', 40)), int(astar.get('curvature_iterations', 0)), float(astar.get('curvature_weight', 0.04)),
                      buf.ctypes.data_as(D), len(buf))
    TIMING['plan_ms'].append((time.perf_counter() - t0) * 1e3)
    if n <= 0:
        raise RuntimeError('no path')
    return np.ascontiguousarray(buf[:n] * res)


def track_error(pos, path):
    a = path[:-1]; v = path[1:] - a; den = (v * v).sum(axis=1)
    t = np.clip(((pos - a) * v).sum(axis=1) / np.maximum(den, 1e-20), 0, 1)
    return float(np.min(np.linalg.norm(pos - a - v * t[:, None], axis=1)))


OVERRIDES = {}
LATENCY = 0.0


def configure(vmax, mode):
    lib.clear()
    for key, value in {**ctrl, **OVERRIDES}.items():
        if isinstance(value, (int, float)) and not isinstance(value, bool):
            lib.set_param(key.encode(), float(value))
    lib.set_param(b'use_mpc', 1.0 if (mode or ctrl.get('controller_mode', 'mpc')) == 'mpc' else 0.0)
    if vmax is not None:
        lib.set_param(b'max_speed', vmax)
    lib.validate()


def run(vmax=None, mode=None):
    global PEERS
    configure(vmax, mode)
    bot = SimRobot(1, np.array([sim['init_x'], sim['init_y']]), sim['v_max'], sim['a_max'], sim['robot_radius'], occ, meta, w, h,
                   sim_hz=sim['publish_hz'], scan_speed=sim['scan_speed'])
    peers = [SimRobot(i, np.array(p), sim['v_max'], sim['a_max'], sim['robot_radius'], occ, meta, w, h)
             for i, p in [(2, [3.90, 7.20]), (3, [9.34, 3.92])]]
    bot.peers = peers; PEERS = peers
    for peer in peers:
        peer.peers = [bot] + [p for p in peers if p is not peer]
    path = plan(bot.pos); reference = path.copy()
    lib.set_plan(path.ctypes.data_as(D), len(path))
    command = np.zeros(2); errors = []; rows = []; contacts = 0; stable = 0
    t03 = None; arrived = None; enabled = True
    pos, vel, yaw, stamp = bot.pos.copy(), bot.vel.copy(), bot.yaw, 0.0   # latest odometry / TF
    odom_queue, cmd_queue = [], []
    for k in range(150 * 150):
        t = k / 150
        while odom_queue and odom_queue[0][0] + LATENCY <= t + 1e-9:
            stamp, (pos, vel, yaw) = odom_queue.pop(0)
        while cmd_queue and cmd_queue[0][0] <= t + 1e-9:
            _, cx, cy = cmd_queue.pop(0)
            bot.on_cmd_vel(cx, cy, int(t * 1e9))
        if arrived is None and k % 15 == 0:
            path = plan(pos); lib.set_plan(path.ctypes.data_as(D), len(path))
        if enabled and k % 3 == 0:
            t0 = time.perf_counter()
            lib.compute(t, stamp, *pos, *vel, yaw, command.ctypes.data_as(D))
            TIMING['control_us'].append((time.perf_counter() - t0) * 1e6)
            cmd_queue.append((t + LATENCY, command[0], command[1]))
        if k % 5:
            continue
        bot.step(int(t * 1e9))
        for peer in peers:
            peer.step(int(t * 1e9))
        odom_queue.append((t, (bot.pos.copy(), bot.vel.copy(), bot.yaw)))
        contacts += int(bot._wall_contact)
        if bot._wall_contact:
            print('contact', t, bot.pos, flush=True)
        goal = float(np.linalg.norm(bot.pos - [14.1, 14.1]))
        if arrived is None:
            e = track_error(bot.pos, reference); errors.append(e)
            rows.append([t, *bot.pos, *bot.vel, e, goal])
            if t03 is None and goal < .3:
                t03 = t
            stable = stable + 1 if goal < .05 and np.linalg.norm(bot.vel) < .06 else 0
            if stable >= 9:   # 0.3 s at 30 Hz, same as the BT
                arrived = t; enabled = False; cmd_queue.clear(); bot.on_cmd_vel(0., 0., int(t * 1e9))
        elif t > arrived + 2.0:
            break
    result = dict(latency_s=LATENCY, overrides=OVERRIDES, speed_limit=vmax if vmax is not None else ctrl['max_speed'], mode=mode or ctrl.get('controller_mode', 'mpc'),
                  reached=arrived is not None, time_to_0p3m_s=t03, arrival_s=arrived,
                  goal_error_at_arrival_m=rows[-1][-1] if rows else None, rest_goal_error_m=goal,
                  initial_path_mean_m=float(np.mean(errors)), initial_path_rmse_m=float(np.sqrt(np.mean(np.square(errors)))),
                  initial_path_max_error_m=max(errors), path_length_m=float(np.linalg.norm(np.diff(reference, axis=0), axis=1).sum()),
                  wall_contact_steps=contacts,
                  # CPU time of the production cores on this machine (incl. ctypes call overhead)
                  plan_ms_mean=float(np.mean(TIMING['plan_ms'])), plan_ms_max=float(np.max(TIMING['plan_ms'])),
                  control_us_mean=float(np.mean(TIMING['control_us'])), control_us_p99=float(np.percentile(TIMING['control_us'], 99)))
    TIMING['plan_ms'].clear(); TIMING['control_us'].clear()
    tag = f"{result['mode']}_{result['speed_limit']}"
    np.savetxt(out / f'trajectory_{tag}.csv', rows, delimiter=',', header='time,x,y,vx,vy,initial_path_error,goal_error', comments='')
    np.savetxt(out / f'reference_{tag}.csv', reference, delimiter=',', header='x,y', comments='')
    print(json.dumps(result), flush=True)
    return result


if __name__ == '__main__':
    args = sys.argv[1:]
    mode = None
    if '--mode' in args:
        i = args.index('--mode'); mode = args[i + 1]; del args[i:i + 2]
    if '--latency' in args:
        i = args.index('--latency'); LATENCY = float(args[i + 1]); del args[i:i + 2]
    while '--set' in args:
        i = args.index('--set'); key, value = args[i + 1].split('='); OVERRIDES[key] = float(value); del args[i:i + 2]
    speeds = [float(v) for v in args] or [None]
    results = [run(v, mode) for v in speeds]
    (out / 'offline_results.json').write_text(json.dumps(results, indent=2))
    if not all(r['reached'] and r['wall_contact_steps'] == 0 for r in results):
        sys.exit(1)
