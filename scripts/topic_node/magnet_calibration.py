"""
Read data from a QMC5883L magnetic sensor, covering a full turn
around the Z axis (i.e. on the X-Y plane). During the acquiring
phase, it shows a curses interface to give a feedback on how
many points were acquired and at what turning angle. When enough
data is acquired (or when the "Q" key is pressed), it saves a
text file with the raw "X Y Z" coordinates.
"""

import py_qmc5883l

import numpy as np
from numpy.linalg import eig, inv
import yaml

import signal

import curses
import math
import time

import textwrap
import subprocess
import sys

import rospy
from aelos_smart_ros.msg import sensor_topic
from rospy.exceptions import ROSException, ROSInterruptException


class Mag_Clibration:
    restart_rosnode = "rosnode kill /sensor_service_node"
    YAML_VERSION = 1

    # Subdivide the entire circle in sectors, to group samples.
    SECTORS_COUNT = 36
    # How many samples to get per each sector.
    SAMPLES_PER_SECTOR = 50

    # Size of dial, in screen characters.
    DIAL_WIDTH = 37
    DIAL_HEIGHT = 19
    BORDER_X = 4
    BORDER_Y = 2

    # Measured values range
    SENSOR_MIN_VAL = -32768
    SENSOR_MAX_VAL = 32767

    # RAW_DATA_FILE = "magnet-data_%s.txt" % (datetime.datetime.now().strftime('%Y%m%d_%H%M'),)
    # ------------------------------------------------------------------------
    # Calculate the size of screen objects.
    # ------------------------------------------------------------------------
    DIAL_RADIUS_X = float((DIAL_WIDTH - 1) / 2.0)
    DIAL_RADIUS_Y = float((DIAL_HEIGHT - 1) / 2.0)
    SECTOR_WIDTH = (2 * math.pi) / SECTORS_COUNT
    TOTAL_WIDTH = DIAL_WIDTH + BORDER_X * 2
    TOTAL_HEIGHT = DIAL_HEIGHT + BORDER_Y * 2

    Clibration_stop_flag = False

    SAMPLES = {}

    CALIBRATION_FINISH = 0
    CALIBRATION_STOP = 1
    CALIBRATION_ERROR = 2

    def __init__(self):
        self.Clibration_stop_flag = False
        pass

    def stop_Calibration(self):
        self.Clibration_stop_flag = True

    def calibration(self, view_in_terminal=False):
        # 声明收集磁铁.Inizialize samples dictionary
        self.SAMPLES = {}
        for i in range(0, self.SECTORS_COUNT):
            self.SAMPLES[i] = []

        if view_in_terminal:

            self.stdscr = curses.initscr()
            # Hide the cursor and make getch() non-blocking.
            curses.curs_set(0)
            curses.noecho()
            self.stdscr.nodelay(1)
            self.stdscr.refresh()

            # Draw a box.
            self.print_at(0, 0, "-" * self.TOTAL_WIDTH)
            self.print_at(0, self.TOTAL_HEIGHT - 1, "-" * self.TOTAL_WIDTH)
            for i in range(1, self.TOTAL_HEIGHT-1):
                self.print_at(0, i, "|")
                self.print_at(self.TOTAL_WIDTH-1, i, "|")
            msg = '请缓慢旋转机器人，采集地磁信息，直到罗盘所有的 "." 都变成 "#" ，如果想提前结束采样（采样不完整校正效果不佳）可以按下键盘 "q"\
                    Do a complete rotation of the sensor on the XY plane. When enough samples are acquired, each sector will be marked with an "#".'
            self.print_at(0, self.TOTAL_HEIGHT+2,
                          textwrap.fill(msg, self.TOTAL_WIDTH))

            # print dial on screen.
            for i in range(0, self.SECTORS_COUNT):
                angle = self.SECTOR_WIDTH * i
                DOT_X = self.BORDER_X + int(self.DIAL_RADIUS_X +
                                            self.DIAL_RADIUS_X * math.sin(angle))
                DOT_Y = self.BORDER_Y + int(self.DIAL_RADIUS_Y -
                                            self.DIAL_RADIUS_Y * math.cos(angle))
                self.print_at(DOT_X, DOT_Y, ".")
            self.print_at(self.BORDER_X + int(self.DIAL_RADIUS_X),
                          self.BORDER_Y + int(self.DIAL_RADIUS_Y), '+')
            self.print_at(self.BORDER_X + int(self.DIAL_RADIUS_X),
                          self.BORDER_Y - 1, 'N')
            self.print_at(self.BORDER_X + int(self.DIAL_RADIUS_X),
                          self.BORDER_Y + self.DIAL_HEIGHT, 'S')
            self.print_at(self.BORDER_X + self.DIAL_WIDTH, self.BORDER_Y +
                          int(self.DIAL_RADIUS_Y),  'E')
            self.print_at(self.BORDER_X - 1, self.BORDER_Y +
                          int(self.DIAL_RADIUS_Y), 'W')

        # Loop to acquire data for the entire circumference.
        completed_sectors = 0
        NEEDLE_X = NEEDLE_Y = 1
        while True:
            try:
                sensor_msg = rospy.wait_for_message('/sensor_info',sensor_topic, timeout = 20)
            except (ROSException, ROSInterruptException) as e:
                rospy.logerr("magnet_calibration timeout error occurred: %s", str(e))
                return self.CALIBRATION_ERROR

            (x, y, z) = (sensor_msg.mag_x, sensor_msg.mag_y , sensor_msg.mag_z)
            if x is not None and y is not None:
                # Angle on the XY plane from magnetic sensor.
                angle = math.atan2(y, x)
                if angle < 0:
                    angle += 2 * math.pi
                sector = int(angle / self.SECTOR_WIDTH)
                sampled = len(self.SAMPLES[sector])
                # Needle angle, rounded to sector center.
                needle_angle = ((2 * math.pi) / self.SECTORS_COUNT) * sector

                if view_in_terminal:
                    # Hide compass needle at previous position.
                    self.print_at(NEEDLE_X, NEEDLE_Y, " ")

                    # Print compass needle.
                    NEEDLE_X = self.BORDER_X + \
                        int(self.DIAL_RADIUS_X + self.DIAL_RADIUS_X *
                            0.8 * math.sin(needle_angle))
                    NEEDLE_Y = self.BORDER_Y + \
                        int(self.DIAL_RADIUS_Y - self.DIAL_RADIUS_Y *
                            0.8 * math.cos(needle_angle))
                    self.print_at(NEEDLE_X, NEEDLE_Y, "O", curses.A_REVERSE)
                    self.print_at(0, self.TOTAL_HEIGHT, "(X, Y) = (%s, %s), Compass: %s deg"
                                  % ("{:6d}".format(x), "{:6d}".format(y), "{:5.1f}".format(math.degrees(angle))))

                if sampled < self.SAMPLES_PER_SECTOR:

                    self.SAMPLES[sector].append([x, y, z])
                    sampled += 1

                    if view_in_terminal:
                        completed = int(
                            10 * (float(sampled) / self.SAMPLES_PER_SECTOR))
                        if completed < 10:
                            completed = str(completed)
                            attr = curses.A_NORMAL
                        else:
                            completed = '#'
                            attr = curses.A_REVERSE

                        DOT_X = self.BORDER_X + \
                            int(self.DIAL_RADIUS_X + self.DIAL_RADIUS_X *
                                math.sin(needle_angle))
                        DOT_Y = self.BORDER_Y + \
                            int(self.DIAL_RADIUS_Y - self.DIAL_RADIUS_Y *
                                math.cos(needle_angle))
                        self.print_at(DOT_X, DOT_Y, completed, attr)

                    if sampled >= self.SAMPLES_PER_SECTOR:
                        completed_sectors += 1
                    if completed_sectors >= self.SECTORS_COUNT:
                        break
                    time.sleep(0.10)
            time.sleep(0.05)

            if view_in_terminal:
                # 通过信号退出.
                key = self.stdscr.getch()
                if key == ord('q'):
                    curses.endwin()
                    return self.CALIBRATION_STOP
            
            elif self.Clibration_stop_flag:
                return self.CALIBRATION_STOP
        
        if view_in_terminal:
            curses.endwin()
        # 返回完成态
        return self.CALIBRATION_FINISH

    def statistics_form_samples(self, samples):
        """ with "x y" data lines. Return two lists with x and
        y values, plus the min/max values for x and y."""
        # global SENSOR_MAX_VAL, SENSOR_MIN_VAL
        min_x = min_y = self.SENSOR_MAX_VAL
        max_x = max_y = self.SENSOR_MIN_VAL
        x = []
        y = []

        for sample_key, sample_value in samples.items():
            for sample_data in sample_value:
                data_x = sample_data[0]
                data_y = sample_data[1]
                x.append(data_x)
                y.append(data_y)
                if data_x < min_x:
                    min_x = data_x
                if data_x > max_x:
                    max_x = data_x
                if data_y < min_y:
                    min_y = data_y
                if data_y > max_y:
                    max_y = data_y
        return x, y, min_x, min_y, max_x, max_y

    def fit_ellipse(self, x, y, use_abs=True):
        """Return the best fit ellipse from two numpy.ndarray
        (multidimensional arrays) of vertices."""
        x = x[:, np.newaxis]
        y = y[:, np.newaxis]
        D = np.hstack((x*x, x*y, y*y, x, y, np.ones_like(x)))
        S = np.dot(D.T, D)
        C = np.zeros([6, 6])
        C[0, 2] = C[2, 0] = 2
        C[1, 1] = -1
        E, V = eig(np.dot(inv(S), C))
        if use_abs:
            n = np.argmax(np.abs(E))
        else:
            # Use this if semi axes are invalid (sqrt of negative).
            n = np.argmax(E)
        a = V[:, n]
        return a

    def ellipse_center(self, a):
        """Return the coordinates of the ellipse center."""
        b, c, d, f, g, a = a[1]/2, a[2], a[3]/2, a[4]/2, a[5], a[0]
        num = b*b-a*c
        x0 = (c*d-b*f)/num
        y0 = (a*f-b*d)/num
        return np.array([x0, y0])

    def ellipse_semi_axes_length(self, a):
        """Return the lenght of both semi-axes of the ellipse."""
        b, c, d, f, g, a = a[1]/2, a[2], a[3]/2, a[4]/2, a[5], a[0]
        up = 2*(a*f*f+c*d*d+g*b*b-2*b*d*f-a*c*g)
        down1 = (b*b-a*c)*((c-a)*np.sqrt(1+4*b*b/((a-c)*(a-c)))-(c+a))
        down2 = (b*b-a*c)*((a-c)*np.sqrt(1+4*b*b/((a-c)*(a-c)))-(c+a))
        if (up/down1) >= 0 and (up/down2) >= 0:
            res1 = np.sqrt(up/down1)
            res2 = np.sqrt(up/down2)
        else:
            res1 = None
            res2 = None
        return np.array([res1, res2])

    def ellipse_angle_of_rotation(self, a):
        """Return the rotation angle (in radians) of the ellipse axes.
        A positive angle means counter-clockwise rotation."""
        b, c, d, f, g, a = a[1]/2, a[2], a[3]/2, a[4]/2, a[5], a[0]
        return 0.5*np.arctan(2*b/(a-c))

    def affine_matrix(self, cx, cy, a, b, phi, to_origin=False):
        """Matrix for affine transformation from ellipse to circle."""
        if a >= b:
            # Affine transformation to circle with R = A (major axis).
            ab_ratio = float(a) / float(b)
            cos_phi = np.cos(phi)
            sin_phi = np.sin(phi)
        else:
            # Swap A and B axis: transformation to circle with R = B (major axis).
            ab_ratio = float(b) / float(a)
            cos_phi = np.cos(phi+np.pi/2)
            sin_phi = np.sin(phi+np.pi/2)
        # R1 and R2: matrix to rotate the ellipse orthogonal to the axes and back.
        # T1 and T2: matrix to translate the ellipse to the origin and back.
        # D: matrix to scale ellipse to circle.
        R1 = np.array([[cos_phi,  sin_phi, 0], [-sin_phi,
                      cos_phi, 0], [0, 0, 1]], dtype=float)
        R2 = np.array(
            [[cos_phi, -sin_phi, 0], [sin_phi,  cos_phi, 0], [0, 0, 1]], dtype=float)
        T1 = np.array(
            [[1,          0,   -cx], [0,        1,     -cy], [0, 0, 1]], dtype=float)
        T2 = np.array([[1,          0,    cx], [
                      0,        1,      cy], [0, 0, 1]], dtype=float)
        D = np.array([[1,          0,     0], [
                     0, ab_ratio,       0], [0, 0, 1]], dtype=float)
        if to_origin:
            # Transformation shifted to axes origin.
            return np.matmul(np.matmul(np.matmul(R2, D), R1), T1)
        else:
            # Transformation centered with the ellipse.
            return np.matmul(np.matmul(np.matmul(np.matmul(T2, R2), D), R1), T1)

    def calibration_info_to_yaml(self, yaml_path):
        # Read data from file.
        if len(self.SAMPLES) == 0:
            return False
        x, y, min_x, min_y, max_x, max_y = self.statistics_form_samples(
            self.SAMPLES)
        # print("{0} {1} {2} {3} {4} {5}" .format(x, y, min_x, min_y, max_x, max_y))
        MAX_SCALE = int(max(abs(min_x), abs(max_x), abs(
            min_y), abs(max_y)) / 500.0 * 750.0)

        # Convert lists x and y into Numpy N-dimensional arrays.
        x_arr = np.fromiter(x, float)
        y_arr = np.fromiter(y, float)

        # Calculate the ellipse which best fits the data.
        warning = ''
        ellipse = self.fit_ellipse(x_arr, y_arr)
        [cx, cy] = self.ellipse_center(ellipse)
        [a, b] = self.ellipse_semi_axes_length(ellipse)
        phi = self.ellipse_angle_of_rotation(ellipse)
        # If semi axes are invalid, try a different method.
        if a == None or b == None:
            warning = "Invalid semi axes detected: using fit_ellipse() without np.abs()."
            ellipse = self.fit_ellipse(x_arr, y_arr, use_abs=False)
            [cx, cy] = self.ellipse_center(ellipse)
            [a, b] = self.ellipse_semi_axes_length(ellipse)
            phi = self.ellipse_angle_of_rotation(ellipse)

        # Calculate the coordinates of semi-axes vertices.
        # ax = cx + a * np.cos(phi)
        # ay = cy + a * np.sin(phi)
        # bx = cx + b * np.cos(phi + np.pi/2)
        # by = cy + b * np.sin(phi + np.pi/2)

        # # Calculate the affine transformation matrix:
        # # centered with the best fitting ellipse...
        # M = affine_matrix(a, b, phi, to_origin=False)
        # centered on the origin...
        M1 = self.affine_matrix(cx, cy, a, b, phi, to_origin=True)

        data = {'yaml_version': self.YAML_VERSION,  'clibration': M1.tolist()}

        # print("mag Clibration:")
        # print(str(M1))

        with open(str(yaml_path), 'w') as f:
            yaml.dump(data, f, sort_keys=False)
        
        return M1

    # ------------------------------------------------------------------------
    # ------------------------------------------------------------------------
    def print_at(self, x, y, string, attr=curses.A_NORMAL):
        # global stdscr
        try:
            self.stdscr.addstr(y, x, string, attr)
            self.stdscr.refresh()
        except:
            pass


def terminate_handler(sig, frame):
    curses.endwin()
    sys.exit(1)


if __name__ == '__main__':
    rospy.init_node('magnet_calibration_shell', anonymous=True)
    # ------------------------------------------------------------------------
    # Initialize the magnetic sensor and screen curses.
    # ------------------------------------------------------------------------

    signal.signal(signal.SIGINT, terminate_handler)

    mag_calibrate = Mag_Clibration()
    if mag_calibrate.calibration(True) == mag_calibrate.CALIBRATION_ERROR:
        print("calibration error")
        curses.endwin()
        sys.exit(1)
    # mag_samples = mag_calibrate.calibration(False)
    m1 = mag_calibrate.calibration_info_to_yaml('/mnt/leju_data/magnet/mag.yaml')

    try:
        result = subprocess.check_output(mag_calibrate.restart_rosnode, shell=True)
        print("校正完成")
        print(m1)
    except Exception as err:
        print("err =",err)
