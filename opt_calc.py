import numpy as np
import numpy.linalg as LA
import matplotlib.pyplot as plt
from math import sqrt
from math import log

def S(t,a,b,c):
    return sqrt(a*(t**2) + b*t + c)

def non_linear_route(t,a,b,c):
    st = S(t,a,b,c)
    s0 = S(0,a,b,c)
    st0 = st - s0
    tst = t * st

    sqa = sqrt(a)
    re_sqa = 1 / sqa

    I = lambda x,s:re_sqa*log(abs(2*a*x+b+2*sqa*s))
    it = I(t,st)
    i0 = I(0,s0)
    it0 = it - i0
    tit = t * it

    q1 = it0
    p1 = (st0 - b * it0 / 2) / a
    q2 = -st0 / a + (b/(2*a))*it0 + tit - t * i0
    p2 = (3*b)/(4*(a*a)) * st0 + (4*a*c-3*(b*b))/(8*(a*a))*it0 + \
        1/(2*a) * tst - b/(2*a) * tit - (1/a) * t * s0 + \
        b / (2*a) * t * i0
    return [q1, p1, q2, p2]

def calc_integration(t,a,b,c):
    # 例外処理をしない
    return non_linear_route(t,a,b,c)

def opt_route(tf,px_,pv0_,x0_,v0_):
    p = -np.array(px_)
    q = np.array(pv0_)
    x0 = np.array(x0_)
    v0 = np.array(v0_)

    a = p@p
    b = 2.0*p@q
    c = q@q
    
    if (b*b) == (4*a*c):
        a0 =  q / sqrt(c)
        if a >= 0:
            # 一定加速度
            vtf = a0 * tf + v0
            xtf = 0.5 * a0 * (tf*tf) + v0 * tf + x0
        else:
            # 前半
            ta = -b/(2*a)
            vh = a0 * ta + v0
            xh = 0.5 * a0 * (ta*ta) + v0 * ta + x0
            # 後半
            tb = tf - ta
            vtf = -a0 * tb + vh
            xtf = -0.5 * a0 * (tb*tb) + vh * tb + xh
    else:
        q1,p1,q2,p2 = calc_integration(tf,a,b,c)
        vtf = (q1 * q + p1 * p) + v0
        xtf = (q2 * q + p2 * p) + v0 * tf + x0
    return [xtf, vtf]

