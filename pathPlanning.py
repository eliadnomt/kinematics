import modern_robotics as mr
import numpy as np
# ============= Trajectory generation ============== #
x0 = 400
xf = 450
y0 = 380
yf = 410

start = np.array([x0, y0, 0])
end = np.array([xf, yf, 0])

xdiff = xf - x0
ydiff = yf - y0

dx = xdiff/59
dy = ydiff/59

waypoints = []

for i in range(60):
    x0 = x0 + dx
    y0 = y0 + dy
    p = [int(x0), int(y0), 0]
    waypoints.append(p)

waypoints = np.array(waypoints)
#print(waypoints)

# ============= Path planning ============== #

L = 220
eomg = 0.1
ev = 0.1
homexdist = L / 2
homeydist = np.sqrt(L ** 2 - homexdist ** 2)

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
    [2 * homeydist, homeydist, homeydist],
    [0, -homexdist, homexdist],
    [0, 0, 0]]
)

thetalist0 = np.array([np.pi/4, np.pi/8, -np.pi/4])
thetalist = []

for i in range(len(waypoints)):
    p = waypoints[i]
    Tsb = mr.RpToTrans(R, p)
    [th, success] = mr.IKinBody(Blist, M, Tsb, thetalist0, eomg, ev)
    thetalist.append(th)

f = open("joint_angles.txt", "w")
for i in range(len(thetalist)):
    angle = thetalist[i]
    f.write(str(angle)+"\n")
f.close()

# ============= Search ============== #


def find_available(node):
    available = []
    if (node[0] + 0 and node[1]+50) not in available:
        available.append([node[0] + 0, node[1]+50])
    if (node[0] + 50 and node[1]+0) not in available:
        available.append([node[0] + 50, node[1]+0])
    if (node[0] + 50 and node[1]+50) not in available:
        available.append([node[0] + 50, node[1]+50])
    if (node[0] + 0 and node[1]-50) not in available:
        available.append([node[0] + 0, node[1]-50])
    if (node[0] -50 and node[1]+0) not in available:
        available.append([node[0] -50, node[1]+0])
    if (node[0] -50 and node[1]-50) not in available:
        available.append([node[0] -50, node[1]-50])
    if (node[0] -50 and node[1]+50) not in available:
        available.append([node[0] -50, node[1]+50])
    if (node[0] +50 and node[1]-50) not in available:
        available.append([node[0] +50, node[1]-50])
    return available


node = [450,400]
#
#print(find_available(node))


def iterate_adjacent(node, traversed, paths):
    children = []
    paths = []
    for pos in find_available(node):
        children.append(find_available(pos))
    for i in range(len(children)):
        if children[i] in traversed:
            children.remove(children[i])
    for pos in find_available(node):
        traversed.append(pos)
    print(children)
        # for i in children:
        #     iterate_adjacent(i)
    return children

print(iterate_adjacent(node))
