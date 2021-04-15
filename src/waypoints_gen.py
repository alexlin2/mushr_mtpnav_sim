from math import cos, sin, pi
import numpy as np

def gen_manhattan_path(direction):
    
    if direction[0] == 'n':
        x1, y1 = 2.2, 0 
    elif direction[0] == 's':
        x1, y1 = 1.55, 4.8
    elif direction[0] == 'e':
        x1, y1 = 0, 2.15
    elif direction[0] == 'w':
        x1, y1 = 3.6, 2.7
    
    if direction[1] == 'n':
        x2, y2 = 2.2, 4.8
    elif direction[1] == 's':
        x2, y2 = 1.55, 0
    elif direction[1] == 'e':
        x2, y2 = 3.6, 2.15
    elif direction[1] == 'w':
        x2, y2 = 0, 2.7

    print('(' + str(x1) + ' ' + str(y1) + ')', end=',')

    if direction == 'nw':
        gen_semi_circle(x1-0.9, y2-0.9, direction, 0.9)
    elif direction == 'ne':
        gen_semi_circle(x1+0.3, y2-0.3, direction, 0.3)
    elif direction == 'sw':
        gen_semi_circle(x1-0.3, y2+0.3, direction, 0.3)
    elif direction == 'se':
        gen_semi_circle(x1+0.9, y2+0.9, direction, 0.9)
    elif direction == 'es':
        gen_semi_circle(x2-0.3, y1-0.3, direction, 0.3)
    elif direction == 'ws':
        gen_semi_circle(x2+0.9, y1-0.9, direction, 0.9)
    elif direction == 'en':
        gen_semi_circle(x2-0.9, y1+0.9, direction, 0.9)
    elif direction == 'wn':
        gen_semi_circle(x2+0.3, y1+0.3, direction, 0.3)

    print('('+str(x2) + ' ' + str(y2)+')', end=',')


def gen_semi_circle(x, y, direction, curve_radius):

    if direction == 'nw':
        start, end = 0, pi/2
    elif direction == 'ne':
        start, end = pi, pi/2
    elif direction == 'sw':
        start, end = 0, - pi /2
    elif direction == 'se':
        start, end = pi, 3 * pi /2
    elif direction == 'es':
        start, end = pi/2, 0 
    elif direction == 'ws':
        start, end = pi/2, pi
    elif direction == 'en':
        start, end = - pi / 2, 0
    elif direction == 'wn':
        start, end = 3 * pi/2, pi


    for i in np.linspace(start, end, int(curve_radius / 0.2 + 1)):
        x_float = "{:.3f}".format(x + curve_radius * cos(i))
        y_float = "{:.3f}".format(y + curve_radius * sin(i))
        print('(' + x_float+' '+y_float + ')', end=',')
        
if __name__ == "__main__":
    gen_manhattan_path("es")
