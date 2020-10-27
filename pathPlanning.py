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

dx = xdiff/10
dy = ydiff/10

waypoints = []

for i in range(11):
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

for i in range(len(waypoints)):
    p = waypoints[i]
    Tsb = mr.RpToTrans(R, p)
    [thetalist, success] = mr.IKinBody(Blist, M, Tsb, thetalist0, eomg, ev)

# ============= Search ============== #
available = []
traversed = []
def find_available(node):
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
#print(find_available(node))


def iterate_adjacent(node):
    for pos in find_available(node):
        traversed.append(pos)
        children = find_available(pos)
        for i in range(len(traversed)):
            if traversed[i] in children:
                children.remove(traversed[i])
        print(children)
        # for i in children:
        #     iterate_adjacent(i)
    return children

print(iterate_adjacent(node))
