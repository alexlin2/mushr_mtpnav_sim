from math import cos, sin, pi
import numpy as np

curve_radius = 1.5

def gen_manhattan_path(x, y):
    (x1, x2) = x
    (y1, y2) = y

    print('(' + str(x1) + ' ' + str(x2) + ')', end=',')

    if y2 > x2:
        if y1 < x1:
            gen_semi_circle(x1-curve_radius, y2-curve_radius, 'nw')
        elif y1 > x1:
            gen_semi_circle(x1+curve_radius, y2-curve_radius, 'ne')
    elif y2 < x2: 
        if y1 < x1:
            gen_semi_circle(x1-curve_radius, y2+curve_radius, 'sw')
        elif y1 > x1:
            gen_semi_circle(x1+curve_radius, y2+curve_radius, 'se')

    print('('+str(y1) + ' ' + str(y2)+')', end=',')


def gen_semi_circle(x, y, direction):

    if direction == 'nw':
        start, end = 0, pi/2
    elif direction == 'ne':
        start, end = pi, pi/2
    elif direction == 'sw':
        start, end = 0, - pi /2
    elif direction == 'se':
        start, end = pi, 3 * pi /2

    for i in np.linspace(start, end, 6):
        x_float = "{:.3f}".format(x + curve_radius * cos(i))
        y_float = "{:.3f}".format(y + curve_radius * sin(i))
        print('(' + str(x_float)+' '+str(y_float) + ')', end=',')
        
if __name__ == "__main__":
    gen_manhattan_path((5,6), (0, 3))
