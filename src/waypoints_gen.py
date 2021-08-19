from math import cos, sin, pi, sqrt
import numpy as np

turning_speed = 0.5
speed = 0.5

def gen_manhattan_path(direction):
    
    if direction[0] == 'n':
        x1, y1 = 2.1, 0 
    elif direction[0] == 's':
        x1, y1 = 1.5, 4.8
    elif direction[0] == 'e':
        x1, y1 = -0.6, 2.1
    elif direction[0] == 'w':
        x1, y1 = 4.2, 2.7
    
    if direction[1] == 'n':
        x2, y2 = 2.1, 4.8
    elif direction[1] == 's':
        x2, y2 = 1.5, 0
    elif direction[1] == 'e':
        x2, y2 = 4.2, 2.1
    elif direction[1] == 'w':
        x2, y2 = -0.6, 2.7

    if direction == 'nw':
        gen_straight_line((x1,y1), (x1,y2-0.9))
        gen_semi_circle(x1-0.9, y2-0.9, direction, 0.9)
        gen_straight_line((x1-0.9,y2), (x2,y2))
    elif direction == 'ne':
        gen_straight_line((x1,y1), (x1,y2-0.3))
        gen_semi_circle(x1+0.3, y2-0.3, direction, 0.3)
        gen_straight_line((x1+0.3,y2), (x2,y2))
    elif direction == 'sw':
        gen_straight_line((x1,y1), (x1,y2+0.3))
        gen_semi_circle(x1-0.3, y2+0.3, direction, 0.3)
        gen_straight_line((x1-0.3,y2), (x2,y2))
    elif direction == 'se':
        gen_straight_line((x1,y1), (x1,y2+0.9))
        gen_semi_circle(x1+0.9, y2+0.9, direction, 0.9)
        gen_straight_line((x1+0.9,y2), (x2,y2))
    elif direction == 'es':
        gen_straight_line((x1,y1), (x2-0.3,y1))
        gen_semi_circle(x2-0.3, y1-0.3, direction, 0.3)
        gen_straight_line((x2,y1-0.3), (x2,y2))
    elif direction == 'ws':
        gen_straight_line((x1,y1), (x2+0.9,y1))
        gen_semi_circle(x2+0.9, y1-0.9, direction, 0.9)
        gen_straight_line((x2,y1-0.9), (x2,y2))
    elif direction == 'en':
        gen_straight_line((x1,y1), (x2-0.9,y1))
        gen_semi_circle(x2-0.9, y1+0.9, direction, 0.9)
        gen_straight_line((x2,y1+0.9), (x2,y2))
    elif direction == 'wn':
        gen_straight_line((x1,y1), (x2+0.3,y1))
        gen_semi_circle(x2+0.3, y1+0.3, direction, 0.3)
        gen_straight_line((x2,y1+0.3), (x2,y2))



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

    circle = np.linspace(start, end, int(curve_radius / 0.2 + 1))

    for i in circle[1:-1]:
        x_float = "{:.3f}".format(x + curve_radius * cos(i))
        y_float = "{:.3f}".format(y + curve_radius * sin(i))
        print('(' + x_float+' '+y_float +' '+ str(turning_speed) + ')', end=',')

def gen_straight_line(waypoint1, waypoint2):

    def dist(x1, x2):
            return sqrt( ((x1[0]-x2[0])**2) + ((x1[1]-x2[1])**2) )
    
    d = dist(waypoint1, waypoint2)

    path_x = np.linspace(waypoint1[0], waypoint2[0], int(d * 3))
    path_y = np.linspace(waypoint1[1], waypoint2[1], int(d * 3))

    for i in range(int(d * 3)):
        x_float = "{:.3f}".format(path_x[i])
        y_float = "{:.3f}".format(path_y[i])
        print('(' + x_float+' '+y_float+' '+ str(speed) + ')', end=',')
        
if __name__ == "__main__":
    #gen_straight_line((2.1,0.0),(2.1,4.8))
    gen_manhattan_path("ws")
