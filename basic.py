#!/usr/bin/python3.6
import opt_calc
import numpy as np
import numpy.linalg as LA
import math
from scipy.optimize import minimize
from scipy.optimize import minimize_scalar
import subprocess as sp
import visualizer
from params import *

th_scale = 0.3
S = np.diag([1.,1.,th_scale])
pS = np.diag([1.,1.,1/th_scale])

##### その場で使うような簡単な関数 #####
### 経路周りの関数
# 初期位置と経路データから経路を計算する
def calc_route(init, route_data):
    r_ = S@np.array(init)
    v_ = S@np.array([0,0,0],dtype=float)
    vias = []
    
    for route in route_data:
        tf,raw_px,raw_pv0 = route
        px_ = pS@np.array(raw_px)
        pv0_= pS@np.array(raw_pv0)
        r_,v_ = opt_calc.opt_route(tf, px_, pv0_, r_, v_)
        r,v = pS@r_, pS@v_
        vias.append([r,v])
    return vias

# 1本のベクトルからデータを整える
def shape(x):
    routes = []
    for i in range(0, len(x), 7):
        routes.append([x[i], x[i+1:i+4], x[i+4:i+7]])
    return routes

# フルデータから経路に変換する
def fulldata_to_route(opt_route):
    ans = []
    for route in opt_route:
        ans.append([route[0], route[-1][0], route[-1][1]])
    return ans

# データを1本のベクトルに展開する
def expand(routes):
    x = []
    for route in routes:
        x.append(route[0])
        for i in range(3):
            x.append(route[1][i])
        for i in range(3):
            x.append(route[2][i])
    return x

# 初期位置と1本のベクトルから経路データから経路を計算する
def calc_route_fromx(init, x):
    return calc_route(init, shape(x))

# 経路データから時間を抽出する
def calc_time_fromx(x):
    return sum([route[0] for route in shape(x)])

# vias dataを出力する
def str_route_data(routes):
    moji = "vias_data = [\n"
    for route in routes:
        moji += "    "
        moji += str([route[0], list(route[1]), list(route[2])]) + ",\n"
    moji += "]"
    return moji

# わーい
def str_full_route(init, routes):
    vias = calc_route(init, routes)
    moji = "  {x = " + str(list(init)) + ", v = [0.0, 0.0, 0.0]},\n"
    for i in range(len(vias)-1):
        x,v = vias[i]
        moji += "  {type= \"PosFixed\", x = " + str(list(x))
        moji += ",v = " + str(list(v)) + "},\n"
    x,v = vias[-1]
    moji += "  {x = " + str(list(x)) + ", v = [0.0, 0.0, 0.0]},\n"
    return moji

### 計算するあたり
# 回転行列
def Rot(th):
    c,s = np.cos(th),np.sin(th)
    return np.array([[c,-s],[s,c]])

# n回転したθを取る
def theta_remainder(theta):
    twopi = 2.0 * math.pi
    theta = theta % twopi
    if theta > math.pi:
        theta -= twopi
    return theta

# ポールとの距離
def poll_distance(r0, poll_pos, machine_size, mergin):
    r = np.array(r0)
    R = Rot(r[-1])
    relative = np.array(poll_pos) - np.array(r[:2])
    poll = np.array([abs(e) for e in R.transpose() @ relative]) - machine_size
    if poll[0] < 0 and poll[1] < 0:
        return min(poll) - mergin
    elif poll[0] < 0:
        return poll[1] - mergin
    elif poll[1] < 0:
        return poll[0] - mergin
    else:
        return LA.norm(poll)-mergin

# 経路とポールの距離
def poll_route_strict(init, x, area_num, poll, machine_size, mergin, N):
    routes = shape(x)

    r_,v_ = [S@np.array(init), S@np.array([0.,0.,0.])]
    i = 0
    while True:
        tf,raw_px,raw_pv0 = routes[i]
        px_,pv0_ = [pS@np.array(raw_px), pS@np.array(raw_pv0)]
        if i < area_num:
            r_,v_ = opt_calc.opt_route(tf, px_, pv0_, r_, v_)
            i += 1
        else:
            break

    ans = float('inf')
    for i in range(N):
        t = i * tf / N
        r = pS@(opt_calc.opt_route(t, px_, pv0_, r_, v_)[0])
        ans = min(ans, poll_distance(r, poll, machine_size, mergin))
    return ans

# 経路に沿って黄金探索する
def poll_route_search(init, x, via_left, machine_size, mergin, N):
    print("おいおいおいおいおいおいおいおいおいおい")
    exit()
    routes = shape(x)

    start = calc_route_fromx(init, x)[via_left]
    r_ = S@np.array(start[0])
    v_ = S@np.array(start[0])
    tf = routes[via_left][0]
    px_ = pS@np.array(routes[via_left+1][1])
    pv0_ = pS@np.array(routes[via_left+1][2])

    f = lambda ratio:poll_distance(pS@(opt_calc.opt_route(tf*ratio, px_, pv0_, r_, v_)[0]),
        p5, machine_size, mergin)
    result = minimize_scalar(f, bounds=(0., 1.), method='bounded')
    return f(result.x)

# ゾーンに入っているかを見る
def in_zone(r0, machine_size, mergin, Xrange, Yrange):
    r = np.array(r0)
    ans = float('inf')
    for i in [[1,1],[-1,1],[-1,-1],[1,-1]]:
        pos = r[:2] + Rot(r[-1])@( machine_size*np.array((i)) )
        ans = min([ans,
            pos[0]-Xrange[0], Xrange[1]-pos[0],
            pos[1]-Yrange[0], Yrange[1]-pos[1]  ])
    return ans - mergin

# マシン中心がエリアに入ってるかを見る
def center_in_area(r, stricts):
    hoges = [[r[i]-stricts[i][0], stricts[i][1]-r[i]] for i in range(len(r))]
    ans = 0.0
    for p in hoges:
        for x in p:
            if x < 0:
                ans -= (x**2)
    return ans

# キックできる方向にあるのかを見る
def kickable(r0, dest):
    r = np.array(r0[:2])
    dr = np.array(dest) - r
    theta0 = np.arctan2(dr[1], dr[0])
    return theta_remainder(r0[-1]-theta0)

def kick_ball_reachable(r0, dest, poll, ball_mergin):
    r = np.array(r0[:2])
    ball_direction = np.array(dest) - r
    poll_direction = np.array(poll) - r
    ball_dir_projection = ((ball_direction @ poll_direction) / (ball_direction @ ball_direction)) * ball_direction
    minimum_mergin = poll_direction - ball_dir_projection
    ans = LA.norm(minimum_mergin)
    if (Rot(0.5*math.pi)@ball_direction)@minimum_mergin < 0:
        ans *= -1
    return ans - ball_mergin

def last_route_not_rotate(x):
    vias = calc_route_fromx([0., 0., 0.], x)
    dtheta = vias[-2][0][2] - vias[-1][0][2]
    domega = vias[-2][1][2] - vias[-1][1][2]
    return (dtheta**2) + (domega**2)

def last_route_const_acc(x):
    vias = calc_route_fromx([0., 0., 0.], x)
    x0 = np.array(vias[-2][0][:2])
    x1 = np.array(vias[-1][0][:2])
    dx = x1 - x0
    e = dx / LA.norm(dx)
    vertical = np.array([[0,-1],[1,0]]) @ e
    v = np.array(vias[-2][1][:2])
    return v@vertical

# 各経路の時間
def forward_check(x, num):
    return x[7*num]

# 停止しているかチェック
def stop(v0):
    v = np.array(v0)
    return v@v

# コールバック
iteration_count = 0
def basic_callback(x, suffix):
    global iteration_count
    print(iteration_count,calc_time_fromx(x))
    iteration_count += 1
    tmp_file = "tmp/tmp" + suffix + ".txt"
    save_file = "tmp/last" + suffix + ".txt"
    f = open(tmp_file,"w")
    f.write(str_route_data(shape(x)))
    f.close()
    sp.run(" ".join(["cp",tmp_file,save_file]),shell=True)

# 最適化計算を行う
def optimize(init_r, mergin, x0, cons, suffix="", maxiter=100, visualize=True):
    result = minimize(
        calc_time_fromx,
        x0 = x0,
        constraints = cons,
        method = "SLSQP",
        options = {"maxiter":maxiter},
        callback = lambda x:basic_callback(x, suffix),
    )
    
    opt_x = result["x"]
    opt_route = shape(opt_x)
    init_cost = calc_time_fromx(x0)
    final_cost = calc_time_fromx(opt_x)

    print("=====END====")
    print(result)
    
    print("cost:", init_cost, "->", final_cost)

    print("~~~ for ROP ~~~")
    print(str_full_route(init_r, opt_route))
    print("~~~~~~")

    print("~~~ copy&paste below~~~")
    print(str_route_data(opt_route))
    print("~~~~~~")
    
    if visualize:
        visualizer.draw_route(init_r, opt_route, mergin)


##### 関数を作るやつ #####
# pollの制約を考慮するやつ
def gen_poll_strict(init, via_num, poll, machine_size, mergin, mode='ineq'):
    return {
        'type':mode,
        'fun':lambda x:poll_distance(calc_route_fromx(init, x)[via_num][0], poll, machine_size, mergin)
    }

# ゾーンを考えるやつ
def gen_in_zone(init, via_num, machine_size, mergin, XRange, YRange):
    return {
        'type':'ineq',
        'fun':lambda x:in_zone(calc_route_fromx(init, x)[via_num][0], machine_size, mergin, 
            XRange, YRange)
    }

# kz2の中にいるようにするやつ
def gen_in_kz2(init, via_num, machine_size, mergin):
    return gen_in_zone(init, via_num, machine_size, mergin, [0.0, 5.0], [2.6, 5.0])
    
# kz3の中にいるようにするやつ
def gen_in_kz3(init, via_num, machine_size, mergin):
    return gen_in_zone(init, via_num, machine_size, mergin, [0.0, 2.5], [2.6, 5.0])

# マシン中心のエリアを考えるやつ
def gen_center_in_area(init, via_num, stricts):
    return {
        'type':'ineq',
        'fun':lambda x:center_in_area(calc_route_fromx(init, x)[via_num][0], stricts)
    }

# キックできる方向にあるのかを見る
def gen_kickable(init, via_num, dest):
    return {
        'type':'eq',
        'fun':lambda x:kickable(calc_route_fromx(init, x)[via_num][0], dest)
    }

# 経路に沿って確認する
def gen_poll_route_strict(init, via_left, poll, machine_size, mergin, N=100):
    return {
        'type':'ineq',
        'fun':lambda x:poll_route_strict(init, x, via_left, poll, machine_size, mergin, N)
    }

# 経路上で黄金探索して最短の点を取るわよ
def gen_poll_route_search_strict(init, via_left, poll, machine_size, mergin):
    return {
        'type':'ineq',
        'fun':lambda x:poll_route_search(init, x, via_left, poll, machine_size, mergin)
    }

# 時間を逆回ししていないことを確認
def gen_forward_check(num, minimum_value=0.0):
    return {
        'type':'ineq',
        'fun':lambda x:forward_check(x, num) - minimum_value
    }

# 停止しているかチェック
def gen_stop_check(init, via_num):
    return {
        'type':'eq',
        'fun':lambda x:stop(calc_route_fromx(init, x)[via_num][1])
    }

# キックボールがぶつからないか確認
def gen_kick_ball_reachable(init, goll_post, poll, ball_mergin):
    return {
        'type':'ineq',
        'fun':lambda x:kick_ball_reachable(calc_route_fromx(init, x)[-1][0], goll_post, poll, ball_mergin)
    }

# 最後のrouteで回転しない制約
def gen_last_route_not_rotate_strict():
    return {
        'type':'eq',
        'fun':lambda x:last_route_not_rotate(x)
    }

# 最後のrouteで等加速度運動をする制約
def gen_last_route_const_acc_strict():
    return {
        'type':'eq',
        'fun':lambda x:last_route_const_acc(x)
    }

