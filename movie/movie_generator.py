#!/usr/bin/python3.6

import sys
args = sys.argv

if len(args) < 2:
    print("引数たりない")

target = args[1]

route = []
for line in open(target,"r"):
    e = line.split()
    if len(e) < 9:
        continue
    else:
        route.append([float(x) for x in e[1:9:2]])

import matplotlib.pyplot as plt

import numpy as np
import matplotlib.animation as animation
import matplotlib.image as mpimg
import numpy.linalg as LA
import math
import matplotlib.patches as patches

import config

machine_dx = np.array(config.machine_dx)
if target in 'kz':
    machine_dx += np.array([-0.05,0.05])

fig = plt.figure()

dt = 0.1
t = 0.0
index = 0

def draw_circle(center, radius):
    c = np.array(center)
    pos = [c + np.array([radius,0])]
    N = 20
    for i in range(1,N):
        theta = 2.0 * math.pi * i / N
        pos.append(c + radius * np.array([math.cos(theta), math.sin(theta)]))
    pos.append(c+np.array([radius,0]))
    x,y = np.array(pos).transpose()
    plt.plot(x,y)

def update(i):
    plt.cla()
    ax = fig.add_subplot(111)
    global index,t
    while route[index][0] < t and index >= 0:
        index += 1
        if index == len(route):
            print("out ouf index")
            index = -1
            break
    machine = route[index][1:]
    t += dt
    
    fence = [[0,0],[10,0],[10,2.575],
        [8.41,2.575],[10,2.575],
        [10,6.65],[0,6.65],[0,0],
        [0,2.575],[3.09,2.575]]
    x,y = 1e3 * np.array(fence).transpose()
    plt.plot(x,y)
    polls = [[8.41 ,2.575],[7.08 ,4.09],[5.75  ,2.575],[4.42 ,4.09],[3.09  ,2.575]]
    for p in polls:
        draw_circle(np.array(p)*1e3, 107)
        draw_circle(np.array(p)*1e3, 107 + 250)

    x,y,th = machine
    plt.plot(x,y,marker="x")
    p1 = np.array([machine_dx[0],config.machine_dy[0]])
    p2 = np.array([machine_dx[0],config.machine_dy[1]])
    p3 = np.array([machine_dx[1],config.machine_dy[1]])
    p4 = np.array([machine_dx[1],config.machine_dy[0]])
    R = np.array([[math.cos(th), -math.sin(th)],
                [math.sin(th), math.cos(th)]])
    center = np.array([x,y])
    edges = [(1e3*R@p)+center for p in [p1,p2,p3,p4]]
    edges.append(edges[0])
    X,Y = np.array(edges).transpose()
    plt.plot(X,Y)
    D = 5
    if index > D:
        r0, r1, r2 = [np.array(route[index+i][1:3]) for i in [-D,0,D]]
        t0, t1, t2 = [route[index+i][0]             for i in [-D,0,D]]
        v0, v1 = (r1-r0)/(t1-t0), (r2-r1)/(t2-t1)
        v = (r2-r0)/(t2-t0)
        a = (v1-v0)/(np.mean([t1,t2])-np.mean([t0,t1]))
        r = r1
        ax.annotate("", xy=r, xytext=r+v,
            arrowprops=dict(shrink=0,width=0.5,headwidth=0.4,
                                headlength=10, connectionstyle='arc3',
                                facecolor='red', edgecolor='red')
                 )
        ax.annotate("", xy=r, xytext=r+a,
            arrowprops=dict(shrink=0,width=0.5,headwidth=0.4,
                                headlength=10, connectionstyle='arc3',
                                facecolor='red', edgecolor='red')
                 )
        print(t,r[0],r[1],v[0],v[1],a[0],a[1])

T = route[-1][0] - route[0][0]
ani = animation.FuncAnimation(fig, update, interval = int(dt/2 * 1e3),  frames=int(T/dt))
ani.save('route.mp4',writer='ffmpeg')

plt.show()


