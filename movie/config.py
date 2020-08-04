import math
import numpy as np
import numpy.linalg as LA
import matplotlib.pyplot as plt

trth = -0.5 * math.pi
trsz = [9.50, 6.150]

alpha0x = 3.570
alphay = 5.425
alpha = [[alpha0x + 0.27*i, alphay] for i in range(6)]

ts_x = [8.410, 7.080, 5.750, 4.420, 3.090]
ts_y = 0.500 + 0.390 + 0.02
ts = [[ts_x[i], ts_y] for i in range(5)]
try_order = [4, 5, 2, 1, 3]

poll = [[8.410 , 2.575],
        [7.080 , 4.090],
        [5.750 , 2.575],
        [4.420 , 4.090],
        [3.090 , 2.575]]

mergin = 0.107 + 0.2
machine_dx = np.array([ 0.45629, -0.400])
machine_dy = np.array([ 0.350  , -0.350])
# m:x -> f:-y, m:y -> f:x
def conv_m_to_f(pos):
    return np.array([max(pos)+mergin, min(pos)-mergin])
field_dx = conv_m_to_f(machine_dy)
field_dy = conv_m_to_f(-machine_dx)

def poll_relative(num,i,j):
    return np.array(poll[num]) + np.array([field_dx[i],field_dy[j]]) 
def lu(num):
    return poll_relative(num,1,0)
def ru(num):
    return poll_relative(num,0,0)
def rd(num):
    return poll_relative(num,0,1)
def ld(num):
    return poll_relative(num,1,1)

# マシンは回転してるのでdxとdyが逆になることに注意
ABedgex = []
AByabaix = []
for i in [1,3]:
    edge = poll[i][0] + np.array(field_dx)
    ABedgex += list(edge)
    AByabaix.append(edge)
CDedgex = []
CDyabaix = []
for i in [0,2,4]:
    edge = poll[i][0] + np.array(field_dx)
    CDedgex += list(edge)
    CDyabaix.append(edge)
CDedgex = CDedgex[1:-1]
CDyabaix[0][0] = 10.0
CDyabaix[-1][-1] = 0.0
ABCDedgex = [ABedgex, ABedgex, CDedgex, CDedgex]
ABCDyabaix = [AByabaix, AByabaix, CDyabaix, CDyabaix]

# マシンは回転してるのでdxとdyが逆になることに注意
ABy = poll[1][1] + np.array(field_dy)  # [+,-]
CDy = poll[0][1] + np.array(field_dy)  # [+,-]
ABCDy = list(ABy) + list(CDy)

def oresen(vias, y):
    for i in range(len(vias)-1):
        if y > vias[i+1][1]:
            return vias[i+1][0] + (y-vias[i+1][1]) * (vias[i][0] - vias[i+1][0]) / (vias[i][1] - vias[i+1][1])
    return vias[-1][0] + (y-vias[-1][1]) * (vias[-2][0] - vias[-1][0]) / (vias[-2][1] - vias[-1][1])

def calc_cost(vias,debug=False):
    if debug:
        plt.cla()
        for p in poll:
            dy,dx = field_dy,field_dx
            pos = np.array(p)
            R = []
            R.append(pos + np.array([dx[0],dy[0]]))
            R.append(pos + np.array([dx[1],dy[0]]))
            R.append(pos + np.array([dx[1],dy[1]]))
            R.append(pos + np.array([dx[0],dy[1]]))
            R.append(pos + np.array([dx[0],dy[0]]))
            X,Y = np.array(R).transpose()
            plt.plot(X,Y)
        x,y = np.array(vias).transpose()
        plt.plot(x,y)
        for v in vias:
            plt.plot(v[0],v[1],marker="x")
        for p in poll:
            plt.plot(p[0],p[1],marker="x")
        plt.pause(1.0)
    for i in [0,2]:
        N = 20
        for j in range(0,N+1):
            x = oresen(vias, (ABCDy[i]*(N-j)+ABCDy[i+1]*j)/N)
            value = min([(X[1]-x)*(X[0]-x) for X in ABCDyabaix[i]])
            if value < 0:
                if debug:
                    print('inf')
                return float('inf')
    cost = 0
    for i in range(len(vias)-1):
        cost += LA.norm(np.array(vias[i+1]) - np.array(vias[i]))
    if debug:
        print(cost, vias)
    return cost

def gen_tr_vias(s0,e0,debug=False):
    best_cost = float('inf')
    best_vias = None
    best_ind = None

    if s0[1] < e0[1]: # もしstartのほうがy座標低かったら
        start,end = e0,s0
        ts_to_rz = True
    else:
        start,end = s0,e0
        ts_to_rz = False

    if start[1] < 4.090 or end[1] > 2.575:
        return []

    search_range = [[a,b,c,d] 
        for a in range(len(ABedgex)+1)
        for b in range(len(ABedgex)+1)
        for c in range(len(CDedgex)+1)
        for d in range(len(CDedgex)+1)]
    
    for ind in search_range:
        vias = [start]
        for j in range(len(ind)):
            i = ind[j] - 1
            if i >= 0:
                vias.append([ ABCDedgex[j][i], ABCDy[j] ])
        vias.append(end)

        cost = calc_cost(vias,debug)

        if cost < best_cost:
            best_cost = cost
            best_vias = vias
            best_ind = ind
    if best_vias == None:
        print("あ、終わった")
    if ts_to_rz:
        best_vias.reverse()
    print("trの最高のviaを手に入れたぜ❗")
    print("start:",s0,"end:",e0,"index:",best_ind)
    return best_vias[1:-1]


