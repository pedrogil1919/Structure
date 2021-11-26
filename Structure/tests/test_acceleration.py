'''
Created on 25 nov. 2021

@author: pedrogil
'''


from dynamics.simple_acceleration import compute_simple_acceleration, draw_distance

v_ini = 2.0
v_fin = 4.0
d_tot = 20.0
t_tot = 8.0

k = 2.0

a1, t1, v1 = compute_simple_acceleration(v_ini, v_fin, d_tot, t_tot, k)
draw_distance(v_ini, t1, t_tot, a1, -k * a1)
print("end")
