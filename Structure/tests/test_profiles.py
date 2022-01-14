'''
Created on 25 nov. 2021

@author: pedrogil
'''


from dynamics.profiles import SpeedProfile

v_ini = 5.0
d_tot = 40.0
t_lim = 10.0

data = {
    'speed': 13.0,
    'acceleration': 2.0,
    'decceleration': 4.0}

profile = SpeedProfile(data)

v_min, v_max = profile.speed_range(v_ini, d_tot)

a, t, v = profile.compute_profile(8.0, 13.0, d_tot, 3.5)
res = profile.plot_dynamics(8.0, a, t, 0.001)
profile.draw_dynamics(res[0], res[1], res[2], None, True)


# v_aux = 10.0
# while (v_aux < v_max):
#     print("Velocidad:", v_aux)
#     t_min, t_max = profile.two_sections_time_limits(v_ini, v_aux, d_tot)
#     if t_max == float('inf'):
#         t_max = t_lim
#     t_aux = t_min
#     profile.clear_figures()
#     while (True):
#         print("Tiempo:", t_aux)
#         a, t, v = profile.compute_profile(v_ini, v_aux, d_tot, t_aux)
#         res = profile.plot_dynamics(v_ini, a, t, 0.001)
#         if (t_aux < t_max):
#             profile.draw_dynamics(res[0], res[1], res[2])
#         else:
#             profile.draw_dynamics(res[0], res[1], res[2], None, True)
#             break
#         t_aux += 0.1
#     v_aux += 0.25


# a, t, v = profile.compute_profile(v_ini, (v1 + v2) / 2, d_tot, (t31 + t32) / 2)
# ts, ss, ps = profile.plot_dynamics(v_ini, a, t, 0.1)
# profile.draw_dynamics(ts, ss, ps, 1.0, False)
# a, t, v = profile.compute_profile(
#     v_ini, (v1 + v2) / 2, d_tot, (t31 + t32) / 2.5)
print("fin")

# while t_tot < 20:
# min_a, min_t, v_max = profile.profile_two_sections(v_end, d_tot, t_tot)
# print(d_tot, min_a, min_t, v_max)
# profile.plot_dynamics(v_ini, min_a, min_t, 0.001, True, True)
# print(profile.two_sections_time_limits(v_end, d_tot))
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
