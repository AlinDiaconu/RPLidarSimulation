import os
from math import *
import pygame

from rplidar import *
import threading


pygame.init()
lcd = pygame.display.set_mode((640,640))
#pygame.mouse.set_visible(False)
pygame.display.flip()
lcd.fill((0,0,0))
pygame.display.update()

PORT_NAME ='COM3'
lidar = RPLidar(PORT_NAME,115200)
health = lidar.get_health()
print(health)



max_distance = 1300
#DMAX = 4000
#IMIN = 0
#IMAX = 50

#def update_line(num, iterator, line):
#    scan = next(iterator)
#    offsets = np.array([(np.radians(meas[1]), meas[2] ) for meas in scan])
#    line.set_offsets(offsets)
#    intens = np.array([meas[0] for meas in scan])
#    line.set_array(intens)
#    return line 


def process_data(data):
    global max_distance
    for angle in range(0,361):
        distance = data[angle]
        if distance > 0:                  
            #max_distance = max([min([5000, distance]), max_distance])
            radians = angle * pi / 180.0
            x = distance * sin(radians)
            y = distance * cos(radians)

            point = (320 + int(x / max_distance * 119), 320 + int(y / max_distance * 119))
            pygame.draw.line(lcd,pygame.Color(255,0,0),Long_Line(point),point,3)
            pygame.draw.line(lcd,pygame.Color(100,100,100),point,(320,320),3)
            pygame.draw.line(lcd,pygame.Color(0,0,255),point,point,3)
            #lcd.set_at(point, pygame.Color(0,0,255))
    pygame.display.flip()

def Long_Line(init_point):
    mid_y = 640 // 2
    mid_x = 640 // 2
    if init_point[0] == mid_x:
        return init_point
    slope = (init_point[1] - mid_y)/(init_point[0] - mid_x)
    if slope  == 0:
        return init_point
    b = mid_y - slope * mid_x
    a = (0,b)
    e = (640, 640 * slope + b)
    c = (-b/slope,0)
    d = ((640-b)/slope,640)
    dis_a = pow(a[0] - init_point[0], 2) + pow(a[1] - init_point[1],2)
    dis_b = pow(e[0] - init_point[0], 2) + pow(e[1] - init_point[1],2)
    dis_c = pow(c[0] - init_point[0], 2) + pow(c[1] - init_point[1],2)
    dis_d = pow(d[0] - init_point[0], 2) + pow(d[1] - init_point[1],2)
    minim = min(dis_a,dis_b,dis_c,dis_d)
    if minim == dis_a:
        return a
    elif minim == dis_b:
        return e
    elif minim == dis_c:
        return c
    elif minim == dis_d:
        return d

scan_data = [0]*361



def lidar_thread():
    lcd.fill((0,0,0))
    for scan in lidar.iter_scans():
        for _,angle, distance in scan:
            scan_data[min([360, floor(angle)])] = distance
        process_data(scan_data)



lidar_thread = threading.Thread(target=lidar_thread)
lidar_thread.start()

done = False
while not done:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = True

print("Stoping.")
lidar.clear_input()
lidar.stop()
lidar.stop_motor()
lidar.disconnect()