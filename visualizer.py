#!/usr/bin/python3.6
import matplotlib.pyplot as plt

import numpy as np
import matplotlib.animation as animation
import numpy.linalg as LA
import math
import matplotlib.patches as patches
import opt_calc

from params import *


fig = plt.figure()

def draw_circle(center, radius):
    c = np.array(center)
    pos = [c + np.array([radius,0])]
    N = 30
    for i in range(1,N):
        theta = 2.0 * math.pi * i / N
        pos.append(c + radius * np.array([math.cos(theta), math.sin(theta)]))
    pos.append(c+np.array([radius,0]))
    x,y = np.array(pos).transpose()
    plt.plot(x,y)

def draw_field(mergin):
    fence = [[0,0],[10,0],[10,2.575],
        [8.41,2.575],[10,2.575],
        [10,6.65],[0,6.65],[0,0],
        [0,2.575],[3.09,2.575]]
    x,y = np.array(fence).transpose()
    plt.plot(x,y)
    for p in polls:
        c = np.array(p)
        draw_circle(c, 0.107)
        draw_circle(c, mergin)

def draw_machine(r,v):
    x,y,th = r
    center = np.array([x,y])
    p = 0.37
    R = np.array([[math.cos(th), -math.sin(th)],[math.sin(th), math.cos(th)]])
    edges = [center + R @ (np.array(a)*machine_size) for a in [[1,1],[-1,1],[-1,-1],[1,-1],[1,1]]]
    X,Y = np.array(edges).transpose()
    plt.plot(X,Y)
    t = 1.0
    plt.plot([x,x+v[0]*t], [y,y+v[1]*t])

def draw_route(init,route_data,mergin=0.357):
    plt.axes().set_aspect('equal', 'datalim')
    S = np.diag([1.,1.,0.3])
    pS = np.diag([1.,1.,1/0.3])
    draw_field(mergin)
    r_ = S@np.array(init)
    v_ = S@np.array([0,0,0],dtype=float)

    txy = []
    tth = []
    tv = []
    tom = []
    t = 0
    draw_machine(pS@r_,pS@v_)
    for route in route_data:
        tf,raw_px,raw_pv0 = route
        px_ = pS@np.array(raw_px)
        pv0_= pS@np.array(raw_pv0)
        points = []
        txy.append([])
        tth.append([])
        tv.append([])
        tom.append([])
        N = 100
        for i in range(N):
            odom = opt_calc.opt_route(i*tf/N, px_, pv0_, r_, v_)
            points.append(odom[0][:2])
            txy[-1].append([t+i*tf/N, odom[0][0], odom[0][1]])
            tth[-1].append([t+i*tf/N, (pS@odom[0])[2]])
            tv[-1].append([t+i*tf/N, odom[1][0], odom[1][1]])
            tom[-1].append([t+i*tf/N, (pS@odom[1])[2]])
        x,y = np.array(points).transpose()
        plt.plot(x,y)
        r_,v_ = opt_calc.opt_route(tf, px_, pv0_, r_, v_)
        r,v = pS@r_, pS@v_
        draw_machine(r,v)
        t += tf
    plt.plot([r[0],10],[r[1],0])
    plt.show()

    for lis in txy:
        t,x,y = np.array(lis).transpose()
        plt.plot(t,x)
        plt.plot(t,y)
    plt.show()

    for t_th in tth:
        t,th = np.array(t_th).transpose()
        plt.plot(t,th)
    plt.show()
    
    for lis in tv:
        t,vx,vy = np.array(lis).transpose()
        plt.plot(t,vx)
        plt.plot(t,vy)
    plt.show()

    for t_om in tom:
        t,om = np.array(t_om).transpose()
        plt.plot(t,om)
    plt.show()


def write_route(init, route_data, filename="pyroute.py", dt=0.01):
    f = open(filename,"w")
    f.write("route = [\n")
    global_time = 0.0
    t_total = 0.0
    S = np.diag([1.,1.,0.3])
    pS = np.diag([1.,1.,1/0.3])
    scaling = np.diag([1e3,1e3,1/0.3])
    r_ = S@np.array(init)
    v_ = S@np.array([0,0,0],dtype=float)

    conv = lambda t,r:"[ " + str(round(t,2)) + " , " + str(r[0])+ " , " + str(r[1]) + " , " + str(r[2]) + " ]"
    for route in route_data:
        tf,raw_px,raw_pv0 = route
        px_ = pS@np.array(raw_px)
        pv0_ = pS@np.array(raw_pv0)
        while (global_time-t_total) < tf:
            t = global_time-t_total
            odom = opt_calc.opt_route(t, px_, pv0_, r_, v_)
            r,v = scaling@odom[0], scaling@odom[1]
            f.write("    "+conv(global_time,r)+",\n")
            global_time += dt
        odom = opt_calc.opt_route(t,px_,pv0_,r_,v_)
        r_,v_ = odom
        t_total += tf
        print(global_time,t_total,r_,v_)
    
    tf,raw_px,raw_pv0 = route_data[-1]
    px_ = pS@np.array(raw_px)
    pv0_ = pS@np.array(raw_pv0)
    r,v = scaling@odom[0], scaling@odom[1]
    f.write("    "+conv(t_total,r)+",\n")
    f.write("]")


if __name__ == '__main__':
    init_r = [ 0.5 ,2.05 ,3.141592653589793] 

    vias_data2 = [
        [2.675690485658338, [1.5737490263219245, -0.7839645946782798, 0.06138472814837909], [2.8647837164774783, -1.0724125008504934, 0.13018844753063932]],
        [1.3669067918021878, [1.388528581398728, 0.24430481457290298, 0.12277119083818751], [-0.29066766477274425, 0.38934961043576755, -0.015585885164889863]],
        [0.8554509309247698, [-0.42769057582493253, 2.3796653804148375, 1.9053535252422183e-05], [0.28106057870577483, -1.5648336924298833, 1.3392411282774955e-05]],
    ]



    vias_data3 = [
        [2.5663430227623825, [2.7805736298432424, -1.1180010278266395, -0.10273215763938712], [4.919210930116046, -1.536459155543523, -0.11430334525661108]],
        [1.9578681832777851, [-0.023283601486773116, 1.163336324845221, 0.09803947911330867], [-2.348238746594717, 1.6084880586800012, 0.08918902949947534]],
        [0.8591333122528396, [-3.7051019114739536, -0.07684905007121265, -0.14037928210773962], [-2.487733170101678, -0.6595651247976403, -0.17046306060911162]],
        [1.1999999999999, [-1.4826913082164301, 0.29156250631521263, 6.0216598284559163e-05], [1.7623138431941774, -0.3465177955349603, 2.7422127212838045e-05]],
    ]

    vias_data = vias_data3

    write_route(init_r, vias_data)
    draw_route(init_r, vias_data, 0.107 + 0.15)

