'''
Created on 25 nov. 2021

@author: pedrogil
'''


from dynamics.simple_acceleration \
    import compute_two_sections, compute_one_section, plot_dynamics

v_ini = 4.0
v_fin = 2.0
d_tot = 3.0
t_tot = 1.0

k = 10.0
a1, a2, t1, v1 = compute_one_section(v_ini, v_fin, d_tot, t_tot, k)
# a1, a2, t1, v1 = compute_two_sections(v_ini, v_fin, d_tot, t_tot, k)
print("Aceleration 1:", a1)
print("Aceleration 2:", a2)
print("Time:", t1)
print("Speed:", v1)
plot_dynamics(v_ini, [a1, a2], [t1, t_tot - t1], 0.001, True)

# t, v, p = plot_dynamics(1, [1.0, 0.5, -1.5], [0.5, 0.5, 0.4], 0.1, True)
# t, v, p = plot_dynamics(1, [1.0, 0.5, -1.5], [0.5, 0.5, 0.4], 0.07, True)


# for t1, v1, p1 in zip(t, v, p):
#     print("%.1f, %.5f" % (t1, p1))
# print("·····································")
