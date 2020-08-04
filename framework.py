#!/usr/bin/python3.6
from basic import *

maxiter = 200

p5 = np.array([3.09  ,2.575])
mergin = 0.107 + 0.2
zone_mergin = 0.05
machine_size = np.array([0.472,0.388])

goll_post = [10.0, 0.0]

init_r =  [ 0.5 ,2.05 ,3.141592653589793] 

t0 = time.time()

vias_data = [[2.589682441211634, [2.1699235309734177, -1.20969454220854742, -0.12172327379210329], [4.782017022806075, -1.5134958962001008, -0.14532742376332747]], [1.3377600808143457, [-0.3500639119038344, 1.1035549768113249, 0.10693980005959916], [-2.19923640758037, 1.4028651181645522, 0.09125622579415117]], [0.5551663925534844, [-3.029511859759594, 0.008925948876890941, -0.039570942164643796], [-2.3455467799939633, -0.2871950204608332, -0.09287238244922649]]]

x0 = expand(vias_data)
init_cost = calc_time_fromx(x0)

cons = (
    gen_poll_strict(init_r, 0, p5, machine_size, mergin),
    gen_poll_strict(init_r, 1, p5, machine_size, mergin),
    gen_in_zone(init_r, 0, machine_size, mergin, [3.0, 10.0], [0.0, 2.575]),
    gen_in_zone(init_r, 1, machine_size, mergin, [3.0, 10.0], [2.575, 100]),
    gen_kickable(init_r, -1, goll_post),
    gen_poll_route_strict(init_r, 0, p5, machine_size, mergin),
    gen_poll_route_strict(init_r, 1, p5, machine_size, mergin),
    gen_poll_route_strict(init_r, 2, p5, machine_size, mergin),
    gen_in_kz2(init_r, -1, machine_size, zone_mergin),
    gen_forward_check(0),
    gen_forward_check(1),
    gen_forward_check(2),
    gen_stop_check(init_r,-1),
)

iter_count = 0

def cbk(x):
    global iter_count
    print(iter_count)
    iter_count += 1
    f = open("tmp_two.txt","w")
    f.write(str_route_data(shape(x)))
    f.close()
    sp.run(" ".join(["cp","tmp_two.txt","last_two.txt"]),shell=True)

result = minimize(
    calc_time_fromx, 
    x0=x0, 
    constraints=cons, 
    method="SLSQP", 
    options={"maxiter":maxiter},
    callback=cbk
)

opt_x = result["x"]
opt_route = shape(opt_x)

def print_vias_data(x):
    print("vias_data = [")
    for route in shape(x):
        print("    ", [route[0], list(route[1]), list(route[2])], sep="", end=",")
    print("]")

print("=====END====")
print(str_route_data(opt_route))
print_vias_data(result["x"])

print("cost:",init_cost,"->",calc_time_fromx(opt_x))

import visualizer as vs

vs.draw_route(init_r, opt_route, mergin)



