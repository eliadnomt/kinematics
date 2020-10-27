import modern_robotics as mr
import numpy as np

L = 220
xdist = L/2
ydist = np.sqrt(L**2 - xdist**2)

x = 420
y = 410
z = 0
p = np.array([x, y, z])

R = np.array([
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1]
])

M = np.array(
    [[1, 0, 0, 450],
    [0, 1, 0, 400],
    [0, 0, 1, 0],
    [0, 0, 0, 1]]
)

Blist = np.array(
    [[0, 0, 0],
    [0, 0, 0],
    [1, 1, 1],
    [2*ydist, ydist, ydist],
    [0 ,-xdist, xdist],
    [0, 0, 0]]
)

thetalist0 = np.array([np.pi/4, np.pi/8, -np.pi/4])

Tsb = mr.RpToTrans(R, p)

eomg = 0.1
ev = 0.1

[thetalist, success] = mr.IKinBody(Blist, M, Tsb, thetalist0, eomg, ev)
print(thetalist)
print(success)

