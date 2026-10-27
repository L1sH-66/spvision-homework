"""Only used when the test runs on a Python other than 3.10: the supplied _dyn_core
binary is cp310-only, so `python3.10 plant_server.py <dir of _dyn_core>` serves
Plant.step() over a pipe (see offline_plant.py).  Nothing in the simulator is changed."""
import sys
sys.path.insert(0, sys.argv[1])
import _dyn_core
plants = {}
out = sys.stdout
for line in sys.stdin:
    p = line.split()
    c = p[0]
    if c == "N":
        plants[p[1]] = _dyn_core.Plant(float(p[2])); out.write("ok\n")
    elif c == "R":
        plants[p[1]].reset(); out.write("ok\n")
    elif c == "L":
        out.write(repr(float(plants[p[1]].stopping_lag())) + "\n")
    elif c == "V":
        plants[p[1]].set_vel(float(p[2]), float(p[3])); out.write("ok\n")
    elif c == "S":
        a = [float(x) for x in p[2:]]
        vx, vy = plants[p[1]].step(*a)
        out.write(repr(float(vx)) + " " + repr(float(vy)) + "\n")
    elif c == "Q":
        break
    out.flush()
