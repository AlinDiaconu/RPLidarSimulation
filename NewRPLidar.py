from rplidar import RPLidar
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
from matplotlib.collections import LineCollection
import cv2 as cv
import unittest
import time
import pyrplidar_protocol
from pyrplidar_protocol import *
from pyrplidar import PyRPlidar



PORT_NAME = 'COM3'
DMAX = 4000
IMIN = 0
IMAX = 50

def update_line(num, iterator, line,lines,back_l):
    scan = next(iterator)
    offsets = np.array([(np.radians(meas[1]), meas[2]) for meas in scan])
    line.set_offsets(offsets)
    intens = np.array([meas[0] for meas in scan])
    line.set_array(intens)
    for i in range(len(scan)):
        lines[i].set_xdata([0, offsets[i][0]])
        lines[i].set_ydata([0, offsets[i][1]])
    back_lines = [[(angle, radius), (angle, DMAX)] for angle, radius in zip(offsets[:, 0], offsets[:, 1])]
    back_l.set_segments(back_lines)
    return line, *lines, back_l

""" 
#Gasirea culorilor si colorarea pe pixeli 2D in functie de distanta - Pe imagine -> Obj:update pentru date in timp real
#Tips -> Colorarea distantei in functie de Calculul Distantei Euclidiene
def FindColorIn(r,g,b, xmin, xmax, ymin, ymax):
    image = ImageGrab.grab()
    for x in range(xmin, xmax):
        for y in range(ymin,ymax):
            px = image.getpixel((x, y))
            if px[0] == r and px[1] == g and px[2] == b:
                line = x
                back_l = y
                return line,back_l

def FindColor(r,g,b):
    image = ImageGrab.grab()
    size = image.size
    pos = FindColorIn(r,g,b, 1, size[0], 1, size[1])
    return pos
"""

def run():
    lidar = RPLidar(PORT_NAME,115200)
    fig = plt.figure()
    ax  = plt.subplot(111, projection='polar')
    line = ax.scatter([0, 0], [0, 0], s=5, c=[IMIN, IMAX],
                           cmap=plt.cm.Blues_r, lw=0)
    lines = [ax.plot([0, 0], [0, 0], color='green', lw=1)[0] for i in range(360)]
    ax.set_rmax(DMAX)
    ax.grid(True)
    back_lines = ax.add_collection(LineCollection([], colors='red', linewidth=1))
    iterator = lidar.iter_scans()
    ani = animation.FuncAnimation(fig, update_line,
        fargs=(iterator, line, lines,back_lines), interval=50)
    plt.show()
    lidar.stop()
    lidar.disconnect()

#Clasa pentru teste (procesarea raw_data/bytes)
class RPlidarTest(unittest.TestCase):

    def test_stop_req_packet(self):
        cmd = PyRPlidarCommand(RPLIDAR_CMD_STOP)
        self.assertEqual(cmd.raw_bytes, b"\xA5\x25")

    def test_reset_req_packet(self):
        cmd = PyRPlidarCommand(RPLIDAR_CMD_RESET)
        self.assertEqual(cmd.raw_bytes, b"\xA5\x40")

    def test_get_info_req_packet(self):
        cmd = PyRPlidarCommand(RPLIDAR_CMD_GET_INFO)
        self.assertEqual(cmd.raw_bytes, b"\xA5\x50")

    def test_parse_descriptor_01(self):
        descriptor = PyRPlidarResponse(b"\xA5\x5A\x04\x00\x00\x00\x15")
        self.assertEqual(descriptor.data_length, 0x04)
        self.assertEqual(descriptor.send_mode, 0)
        self.assertEqual(descriptor.data_type, 0x15)

    def test_parse_descriptor_02(self):
        descriptor = PyRPlidarResponse(b"\xA5\x5A\x84\x00\x00\x40\x84")
        self.assertEqual(descriptor.data_length, 0x84)
        self.assertEqual(descriptor.send_mode, 1)
        self.assertEqual(descriptor.data_type, 0x84)

    
    def test_varbitscale_decode(self):

        dist_major_input = [0x1E0, 0x20B, 0x219, 0x504, 0x507, 0x51E]
        dist_major_output = [0x1E0, 0x216, 0x232, 0x810, 0x81C, 0x878]
        scalelvl_output = [0, 1, 1 ,2, 2, 2]
    
        for i in range(len(dist_major_input)):
            dist_major, scalelvl = PyRPlidarScanUltraCapsule._varbitscale_decode(dist_major_input[i])
            self.assertEqual(dist_major, dist_major_output[i])
            self.assertEqual(scalelvl, scalelvl_output[i])
    
    def test_dense_capsule_parsing(self):
        pass

#Verificare conexiune port ->testarea citirii datelor     
def check_connection():

    lidar = PyRPlidar()
    lidar.connect(port="COM3", baudrate=115200, timeout=10)
                  
    info = lidar.get_info()
    print("info :", info)
    
    health = lidar.get_health()
    print("health :", health)
    
    samplerate = lidar.get_samplerate()
    print("samplerate :", samplerate)
    

    scan_modes = lidar.get_scan_modes()
    print("scan modes :")
    for scan_mode in scan_modes:
        print(scan_mode)

    lidar.disconnect()
    
if __name__ == '__main__':
    run() & check_connection() 
    start_time = time.time()
    unittest.main()
    end_time = time.time()
    print("WorkingTime: {} sec".format(end_time-start_time))
   
    