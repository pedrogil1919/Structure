'''
Created on 25 nov. 2021

@author: pedrogil
'''


from dynamics.profiles \
    import compute_three_sections, compute_two_sections, compute_one_section, \
    plot_dynamics

v_ini = 10.0
v_fin = 12.0
d_tot = 80.0
t_tot = 10.0

# a, t, v = compute_one_section(v_ini, v_fin, d_tot, t_tot)
# plot_dynamics(v_ini, a, t, 0.001, True, False)
# a, t, v = compute_two_sections(v_ini, v_fin, d_tot, t_tot)
# plot_dynamics(v_ini, a, t, 0.001, True, False)
a, t, v = compute_three_sections(v_ini, v_fin, d_tot, t_tot)
# a = [2, 0, -3]
# t = [2, 1, 1]
# v_ini = 0
plot_dynamics(v_ini, a, t, 0.001, True, True)


# t, v, p = plot_dynamics(1, [1.0, 0.5, -1.5], [0.5, 0.5, 0.4], 0.1, True)
# t, v, p = plot_dynamics(1, [1.0, 0.5, -1.5], [0.5, 0.5, 0.4], 0.07, True)


# for t1, v1, p1 in zip(t, v, p):
#     print("%.1f, %.5f" % (t1, p1))
# print("·····································")
