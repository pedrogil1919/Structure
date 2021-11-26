'''
Created on 25 nov. 2021

@author: pedrogil
'''

from math import sqrt
import numpy
import matplotlib.pyplot as plt
import matplotlib.axes as axes
from youtube_dl.extractor import tnaflix


def compute_simple_acceleration(v_ini, v_end, d_tot, t_tot, k):

    # Compute mean velocity.
    v_mean = d_tot / t_tot - v_ini
    v_end -= v_ini

    # Compute intermediate terms.
    a0 = v_end / t_tot
    v0 = 2 * v_mean - v_end
    t2 = t_tot
    v2 = v_end
    # Comtupe second order equation terms
    c = -a0 * v_end
    b = v2 - k * a0 * t2 - v0 * (1 + k)
    a = k * t2

    d = sqrt(b**2 - 4 * a * c)
    ap = (-b + d) / (2 * a)
    an = (-b - d) / (2 * a)

    tp = v0 / (ap - a0)
    tn = v0 / (an - a0)
    if tp > 0 and tp < t_tot:
        a1 = ap
        t1 = tp
    elif tn > 0 and tn < t_tot:
        a1 = an
        t1 = tn

    v1 = a1 * t1

    return a1, t1, v1


def draw_distance(v0, t1, t2, a1, a2):
    n1 = numpy.arange(0, t1, t2 / 100, numpy.float64)
    n2 = numpy.arange(t1, t2, t2 / 100, numpy.float64)
    p1 = v0 * n1 + 0.5 * a1 * n1**2
    v1 = v0 + a1 * n1

    d1 = v0 * t1 + 0.5 * a1 * t1**2
    s1 = v0 + a1 * t1
    p2 = d1 + s1 * (n2 - t1) + 0.5 * a2 * (n2 - t1)**2
    v2 = s1 + a2 * (n2 - t1)

    d2 = d1 + s1 * (t2 - t1) + 0.5 * a2 * (t2 - t1)**2
    s2 = s1 + a2 * (t2 - t1)

    n = numpy.hstack((n1, n2))
    v = numpy.hstack((v1, v2))
    p = numpy.hstack((p1, p2))

    plt.figure(figsize=(8, 6), dpi=100)

    plt.subplot(2, 1, 1)
    plt.plot(n, v, 'b')
    plt.title("velocidad")
    plt.grid()
    plt.axis([0, t2, 0, max([v0, s1, s2])])

    plt.subplot(2, 1, 2)
    plt.plot(n, p, 'b')
    plt.title("espacio")
    plt.grid()
    plt.axis([0, t2, 0, d2])
    plt.show()
