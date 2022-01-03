'''
Created on 25 nov. 2021

@author: pedrogil
'''


from dynamics.profiles import SpeedProfile

v_ini = 2.0
v_end = 4.0
d_tot = 14.0
t_tot = 6.0

data = {
    'speed': 5.0,
    'acceleration': 10.0,
    'decceleration': 20.0}


profile = SpeedProfile(data, v_ini)

# while t_tot < 20:
min_a, min_t, v_max = profile.profile_two_sections(v_end, d_tot, t_tot)
print(d_tot, min_a, min_t, v_max)
profile.plot_dynamics(v_ini, min_a, min_t, 0.001, True, True)
print(profile.two_sections_time_limits(v_end, d_tot))
# while d_tot > 0:
#     min_a, min_t, v_max = profile.profile_two_sections(v_end, d_tot, t_tot)
#     print(d_tot, min_a, min_t, v_max)
#     profile.plot_dynamics(v_ini, min_a, min_t, 0.001, True, True)
# #     except ValueError as er:
# #         print(er.description())
#     t_tot += 0.5
# #


# try:
#     t_min, t_max = profile.two_sections_time_limits(v_end, d_tot)
# except ValueError:
#     continue
# print("%#4.2f - %0.4f | %0.4f" % (d_tot, t_min, t_max))

# a, t, v = compute_one_section(v_ini, v_fin, d_tot, t_tot)
# plot_dynamics(v_ini, a, t, 0.001, True, False)
# a, t, v = compute_two_sections(v_ini, v_fin, d_tot, t_tot)
# plot_dynamics(v_ini, a, t, 0.001, True, False)
# a, t, v = compute_two_sections(v_ini, v_fin, d_tot, t_tot)
# a = [2, 0, -3]
# t = [2, 1, 1]
# v_ini = 0
# plot_dynamics(v_ini, a, t, 0.001, True, True)


# t, v, p = plot_dynamics(1, [1.0, 0.5, -1.5], [0.5, 0.5, 0.4], 0.1, True)
# t, v, p = plot_dynamics(1, [1.0, 0.5, -1.5], [0.5, 0.5, 0.4], 0.07, True)


# for t1, v1, p1 in zip(t, v, p):
#     print("%.1f, %.5f" % (t1, p1))
# print("·····································")
