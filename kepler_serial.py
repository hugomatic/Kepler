# TODO 
#
# instead of min, max... use zero, min and max  
#


import math
import time
import random

import hugomatic.toolkit # UI and cmdline stuff
import hugomatic.code    # gcode routines

import kepler_sim

from fuzzy_controller import get_angle_controller
import pid_controller
import kalman
import serial
import struct

import logging


log2memory = True

show_serial_port_settings = True
show_table_settings = True
show_elctronics_params = True
show_pid_settings = True

# create logger
logging.basicConfig(filename='kepler.log', filemode='w', level=logging.DEBUG,)
# create console handler and set level to debug
ch = logging.StreamHandler()
ch.setLevel(logging.DEBUG)
# create formatter
#formatter = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
# add formatter to ch
# add ch to logger

logger = logging.getLogger("fuzzy")
#ch.setFormatter(formatter)
logger.addHandler(ch)
logger.setLevel(logging.DEBUG)


def msgerr(msg):
    logger.error(msg)

def msgonce(msg):
    logger.critical(msg)
    
def msgloop(msg):
    """
    Logs a recursive log message 
    """
    logger.debug(msg)



def is_windows():
    try:
        import win32file
        return True
    except ImportError:
        return False
    return True

def take_a_break():
    """
    This function is called when the debug line is printed. Add a breakpoint here
    but don't call print, because the program will go into an infinite loop.
    """
    a = 42

params = hugomatic.toolkit.Parameters('Kepler', 
                                      'Control framework', 
                                      picture_file="kepler.gif",
                                      debug_callback=take_a_break)

interface_name = 'serial'
params.addArgument(interface_name,'Interface to the outside ', choices = ('serial', 'serialASAP','simulator', 'echo'), group = 'setup')
controller_name = 'keyboard'
params.addArgument(controller_name, 'Control strategy',  choices=('keyboard','fuzzy_v1','servo','servo_pid','pid','pid_pid', 'sine'), group='setup')
filter_name = 'mean'
params.addArgument(filter_name, 'Position data filter',  choices=('none','mean','kalman'), group='setup')

serial_port_name = '/dev/ttyACM0' #  '/dev/ttyUSB0'
bauds = 115200
format_tx = "BBBBBBBxxxxxxxxB" # added 2 bytes for motor directions
format_rx = "BBBBBBBBBBBBBBBB" #
timeout = 1

if show_serial_port_settings:
    params.addArgument(timeout , 'Serial read/write timeout in sec', group='serial')
    params.addArgument(format_rx , 'Binary response message format', group='serial')
    params.addArgument(format_tx , 'Binary command message format', group='serial')
    params.addArgument(bauds , 'Bauds (115200)', group='serial')
    params.addArgument(serial_port_name , 'Port name', group='serial')


servo_min = -1024.
params.addArgument(servo_min , 'Servo min speed', group='servo')
servo_max = 1024.
params.addArgument(servo_max , 'Servo max speed', group='servo')



table_distance_from_center = 0.2794 # 11 inches
table_min_rot_speed = -0.0615 # rads / sec
table_max_rot_speed = 0.0615
table_max_height = 0.005 #mm #0.01875
table_min_height = -0.005#-0.01875

if show_table_settings:
    params.addArgument(table_distance_from_center, 'Table center to actuator distance in m', group='table')
    params.addArgument(table_min_height, 'Table min deviation from horiz (neg)', group='table')
    params.addArgument(table_max_height, 'Table max deviation from horiz', group='table')
    params.addArgument(table_min_rot_speed , 'Table min rot speed in rad/sec', group='table')
    params.addArgument(table_max_rot_speed , 'Table max rot speed in rad/sec', group='table')


encoder1_min = 440. - 50. #2048.
encoder1_max = 440. + 50. #6144.

encoder2_min = 348. - 50. #2048.
encoder2_max = 348 + 50 #6144.

load_cell1_min = 159 #830
load_cell1_max = 721 #550

load_cell2_min = 260
load_cell2_max = 700

ball_pos_min = -0.3#0.
ball_pos_max = 0.3#1.0

if show_elctronics_params:
    
    params.addArgument(encoder1_min , 'Min motor1 ADC', group='encoder')
    params.addArgument(encoder1_max , 'Max motor1 ADC', group='encoder')
    
    params.addArgument(encoder2_min , 'Min motor2 ADC', group='encoder')
    params.addArgument(encoder2_max , 'Max motor2 ADC', group='encoder')
    
    params.addArgument(load_cell1_min, 'Load cell 1 min', group='Load cell')
    params.addArgument(load_cell1_max, 'Load cell 1 max', group='Load cell')
    params.addArgument(load_cell2_min, 'Load cell 2 min', group='Load cell')
    params.addArgument(load_cell2_max, 'Load cell 2 max', group='Load cell')
    
    params.addArgument(ball_pos_min, 'Position min (1 and 2) in m', group='Load cell')
    params.addArgument(ball_pos_max, 'Position max (1 and 2) in m', group='Load cell')


kp_s1 = -5. 
ki_s1 = 0.
kd_s1 = -1.
kp_s2 = 0.5 #2.
ki_s2 = 0. #2.
kd_s2 = 0. #0.05  
kp_p1 = -0.5
ki_p1 = 0.
kd_p1 = -0.04
kp_p2 = -0.0713
ki_p2 = 0.
kd_p2 = -0.065

if show_pid_settings:
    params.addArgument(kp_s1, 'P angle 1', group='Angle')
    params.addArgument(ki_s1, 'I angle 1', group='Angle')
    params.addArgument(kd_s1, 'D angle 1', group='Angle')
       
    params.addArgument(kp_s2, 'P angle 2', group='Angle')
    params.addArgument(ki_s2, 'I angle 2', group='Angle')
    params.addArgument(kd_s2, 'D angle 2', group='Angle')
    
    params.addArgument(kp_p1, 'P postion 1', group='Position')
    params.addArgument(ki_p1, 'I postion 1', group='Position')
    params.addArgument(kd_p1, 'D postion 1', group='Position')
    
    params.addArgument(kp_p2, 'P postion 2', group='Position')
    params.addArgument(ki_p2, 'I postion 2', group='Position')
    params.addArgument(kd_p2, 'D postion 2', group='Position')

# Some of the Format codes
#    x     pad byte       no value      
#    c     char           string of length 1     
#    b     signed char    integer     
#    B     unsigned char  integer     
#    h     short          integer     
#    H     unsigned short integer



def split_14bit_number(number):
    low = number & 127
    high = number >> 7
    return low, high

def assemble_14bit_number(high, low):
    number = high << 7
    number += low
    return number

def print_binary_data(pack):
    l = len(pack)
    print "data size: %s" % l 
    print "data: ", repr(pack)
    pos = 0
    while pos <= l-1:
        s = repr(pack[pos])
        print "%02d) %s" % (pos,s)
        pos += 1


def read_response(port, size):
    #print "Reading response..."
    resp = ""
    
    port_read_count = 0
    read_byte_count = 0
    while len(resp) < size:
        #port_read_count += 1
        c = port.read()
        #print "RX: %s" % c 
        resp += c
        read_byte_count += len(resp)
        #if port_read_count > 10:
        #    port_read_count = 0
        #    print "10 port read tries without response"
    return resp

def makeError(mean, sigma):
    def error_func():
         # arguments are "closed" in the definition of error_func
        e = random.gauss(mean, sigma)
        return e
    
    return error_func

class Controller(object):
    
    def __init__(self):
        self.interface = None
        
    def set_interface(self, interface, parent):
        self.interface = interface
        self.parent =  parent
    
    def stop(self):
        pass
            

    def get_display_data(self):
        return {}
    
    def key_up(self):
        pass
    
    def key_down(self):
        pass
    
    def key_right(self):  
        pass
    
    def key_left(self): 
        pass   
    def key_l(self): 
        pass
    def key_s(self): 
        pass
    def key_a(self): 
        pass
    def key_d(self): 
        pass
    def key_q(self):    
        pass
    def key_w(self):    
        pass
    def key_z(self):    
        pass
    def key_x(self):    
        pass
    def key_c(self):    
        pass
    def key_v(self):    
        pass
    def key_b(self):    
        pass  
    def key_n(self):    
        pass
    def key_g(self):
        pass
    
    def key_f(self):
        pass 
    
class PidController(Controller):
    
    def __init__(self, axis, kp, ki, kd, kc, filter_error = True):
        Controller.__init__(self) 
        self.filter_error = filter_error
        #self.pid = pid_controller.Pid(kp, ki, kd)        
        self.kc = kc
        self.kp = kp
        self.ki = ki
        self.kd = kd  
        self.op = 0
        self.oi = 0
        self.od = 0 
        self.axis = axis
        self.pid_out = 0.
        print "Position PID: "  

    def get_display_data(self):
        
        axis = str(self.axis)
        
        return {'pid_out'+axis : self.pid_out,
                'pid_op'+axis :self.op,
                'pid_oi'+axis :self.oi,
                'pid_od'+axis :self.od,
                
                # 'pid_error'+axis : self.pid.error,
                # 'pid_oi'+axis : self.pid.oi,
               'pid_derivative'+axis : self.derivative,
              'pid_integral'+axis : self.integral,
                
                }
        
    
    def loop(self, step, sim_time, delta_time):

        position = None
        error = None 
        derivative = None
        integral = 0.
        
        if self.axis == 1 :
            if self.filter_error:
                error = self.parent.error_pos1
            else:
                error = self.parent.target_position1 - self.parent.current_pos1
            derivative = self.parent.pos_derivative1
            integral = self.parent.pos_integral1
            
        if self.axis == 2 :
            if self.filter_error:
                error = self.parent.error_pos2
            else:
                error = self.parent.target_position2 - self.parent.current_pos2
            derivative = self.parent.pos_derivative2            
            integral = self.parent.pos_integral2
            
        self.op = self.kp * error
        self.oi = self.ki * integral
        self.od = self.kd * derivative
        self.pid_out = self.kc * ( self.op + self.oi  + self.od )  #self.pid.eval( position, target_position, delta_time)
        self.angle_out =self.pid_out
        self.derivative = derivative
        self.integral = integral
        #self.angle1_out = self.parent.target_angle1
        if self.angle_out > table_max_angle:
            #print "oops1"
            self.angle_out = table_max_angle
            #return
            
        if self.angle_out < table_min_angle:
            #print "oops2"
            self.angle_out = table_min_angle
        
        if self.axis == 1:
            self.parent.target_angle1 = self.angle_out
            
        if self.axis == 2:
            self.parent.target_angle2 =  self.angle_out
            
    def key_z(self): 
        self.kp -= 0.01
        print "kp %f" %  self.kp
               
    def key_x(self):
        self.kp += 0.01
        print "kp %f" %  self.kp
        
    def key_c(self): 
        self.ki -= 0.0005
        print "ki %f" %  self.ki
        
    def key_v(self):
        self.ki += 0.0005
        print "ki %f" %  self.ki

    def key_b(self): 
        self.kd -= 0.01
        print "kd %f" %  self.kd
        
    def key_n(self):
        self.kd += 0.01
        print "kd %f" %  self.kd
       
    def key_left(self):  
        m = ""
        self.parent.target_position1 -= ball_pos_max / 10.0
        if self.parent.target_position1 < ball_pos_min:
            m = 'MIN!'
            self.parent.target_position1 = ball_pos_min
        print "Target position is %f %s" % (self.parent.target_position1, m)
    
    def key_right(self): 
        m = ""
        self.parent.target_position1 += ball_pos_max / 10.0
        if self.parent.target_position1 > ball_pos_max:
            m = 'MAX!'
            self.parent.target_position1 = ball_pos_max
        print "Target position is %f %s" % (self.parent.target_position1, m) 

class Derivator(Controller):
    
    def __init__(self, axis, prefix = 'pos', deriv_pv = False): 
        Controller.__init__(self)
        self.deriv_pv = deriv_pv
        self.prefix = 'pos'
        self.axis = axis
        self.error_history = [(0,0.01),(0,0.01),(0,0.01),(0,0.01),(0,0.01)]
        self.error = 0
        self.derivative = 0
        self.integral = 0
        
    def get_display_data(self):
        sa = str(self.axis)
        data = {
                #'pos_error'+sa:self.error,
                #'pos_derivative'+sa:self.derivative,
                #'pos_integral'+sa:self.integral
                }
        return data
        
    def loop(self, step, sim_time, delta_time):
        self.error = None
        self.position = None
        if self.axis == 1:
            self.error = self.parent.target_position1 - self.parent.filtered_pos1
            self.position = self.parent.filtered_pos1
        if self.axis == 2:
            self.error = self.parent.target_position2 - self.parent.filtered_pos2   
            self.position =  self.parent.filtered_pos2
        # remove first elem
        self.error_history = self.error_history[1:]
        # add new elem
        if self.deriv_pv:
            self.error_history.append( (self.error, delta_time) )
        else:
            self.error_history.append( (self.position, delta_time) )   
        self.integral += self.error * delta_time
        
        # http://en.wikipedia.org/wiki/Five-point_stencil
        # get step size
        h = 0.
        for d in self.error_history:
            h += d[1] 
        h = h / len(self.error_history)
        
        f_x_plus_2h = self.error_history[4][0]
        f_x_plus_h  = self.error_history[3][0]
        # f_x  = self.error_history[2][0]
        f_x_minus_h  = self.error_history[1][0]
        f_x_minus_2h  = self.error_history[0][0]
        
        self.derivative = 0.
        if h > 0:
            self.derivative = (-f_x_plus_2h + 8.*f_x_plus_h -8*f_x_plus_h + f_x_minus_2h) / (12. * h)
        if self.axis == 1:
            self.parent.error_pos1 = self.error
            self.parent.pos_derivative1 = self.derivative
            self.parent.pos_integral1 = self.integral
        if self.axis == 2:
            self.parent.error_pos2 = self.error
            self.parent.pos_derivative2 = self.derivative
            self.parent.pos_integral2 = self.integral

class ServoMeanFilter(Controller):

    def __init__(self, axis, band):   
        self.axis = axis
        self.band = band
        self.count_down = -1
        self.history = []
        for i in range(self.band):
            self.history.insert(0,0.)
            
    def eval(self, history):
        total = 0.
        count = len(history)
        for v in history:
            total += v
        value = total / count
        return value
         
    def loop(self, step, sim_time, delta_time):
        position = None
        if self.axis == 1:
            position = self.parent.servo1
        if self.axis == 2:
            position = self.parent.servo2
            
        self.history.insert(0,position)
        self.history.pop()
        self.count_down -= 1
        if self.count_down < 0:
            count_down = self.band
            filtered_out = self.eval(self.history)
            if self.axis == 1:
                self.parent.filtered_servo1 = filtered_out 
            if self.axis == 2:
                self.parent.filtered_servo2 = filtered_out

class Filter(Controller):

    def __init__(self, axis, band):   
        self.axis = axis
        self.band = band
        self.count_down = -1
        self.history = []
        for i in range(self.band):
            self.history.insert(0,0.)
            
    def eval(self, history):
        value = history[-1]
        print "Filter len(history) %s" % len(history)
        return value
         
    def loop(self, step, sim_time, delta_time):
        position = None
        if self.axis == 1:
            position = self.parent.current_pos1
        if self.axis == 2:
            position = self.parent.current_pos2
            
        self.history.insert(0,position)
        self.history.pop()
        self.count_down -= 1
        if self.count_down < 0:
            count_down = self.band
            filtered_pos = self.eval(self.history)
            if self.axis == 1:
                self.parent.filtered_pos1 = filtered_pos 
            if self.axis == 2:
                self.parent.filtered_pos2 = filtered_pos
          
                
class MeanFilter(Filter):
     
     def __init__(self, axis, band):
         Filter.__init__(self, axis, band)
        
     def eval(self, history):
        count = len(history)
        total = 0.
        
        for v in history: 
            total += v
        value = total / count
        return value
    
       
            

        
class ServoPid(Controller):
    
    def __init__(self, axis, kp, ki, kd, keyboard_enabled, display_data):
        self.display_data = display_data
        self.pid = pid_controller.Pid(kp , ki, kd ) #(-80., 0., 0.,) #(-55., -0.00001, 0. ) # kc -110. starts to oscillate
        self.axis = axis # 1 or 2, for x or y
        self.keyboard_enabled = keyboard_enabled
        self.pid_out = None
        self.pid.op = None
        self.pid.oi = None
        self.pid.od = None

        #self.pid.error = None
        #self.pid.derivative = None
        #self.pid.integral = None

    
    def _get_angle(self):
        if self.axis == 1:    
            v = self.parent.target_angle1
        if self.axis == 2:
            v = self.parent.target_angle2
        return v
               
    def _set_angle(self, v):
        m = ''
        value = v
        if value >= table_max_angle:
            m = 'MAX!'
            value = table_max_angle
        elif  value  <= table_min_angle:
            m = 'MIN!'
            value = table_min_angle
        
        if self.axis == 1:    
            self.parent.target_angle1 = value
        if self.axis == 2:
            self.parent.target_angle2 = value
        print "target angle %s is %f %s" % (self.axis, value, m)

    def key_q(self):
        self.keyboard_enabled = not self.keyboard_enabled
        print "servo pid %s keyboard = %s" % (self.axis, self.keyboard_enabled)
        
    def key_a(self):
        if self.keyboard_enabled: 
            self._set_angle(table_min_angle)

    def key_s(self):
        if self.keyboard_enabled:
            v  =  self._get_angle()  - (table_max_angle / 10.0)
            self._set_angle(v)
        
    def key_d(self):
        self._set_angle(0.)

    def key_f(self): 
        if self.keyboard_enabled:
            v  =  self._get_angle() + (table_max_angle / 10.0)
            self._set_angle(v)
         
    def key_g(self): 
        if self.keyboard_enabled:
            self._set_angle(table_max_angle)
 
        
        
    def get_display_data(self):
        data = {}
        if not self.display_data:
            return data
        name = '1'
        if self.axis == 2:
            name = '2'
            
        data['servo_pid_out'+name] = self.pid_out
        data['servo_pid_op'+name] = self.pid.op
        data['servo_pid_oi'+name] = self.pid.oi
        data['servo_pid_od'+name] = self.pid.od 

        data['servo_pid_error' + name] = self.pid.error 
        data['servo_pid_derivative' + name] = self.pid.derivative
        data['servo_pid_integral' + name] = self.pid.integral
        return data
    
    def loop(self, step, sim_time, delta_time):
        
        current_angle = 0.
        desired_angle = 0.
        
        if self.axis == 1:
            current_angle = encoder1_to_angle(self.parent.encoder1 )
            desired_angle = self.parent.target_angle1
        if self.axis == 2:
            current_angle = encoder2_to_angle(self.parent.encoder2 )
            desired_angle = self.parent.target_angle2
 
        self.pid_out = self.pid.eval(current_angle, desired_angle, delta_time)
        target_rot_speed = self.pid_out
        
        if self.pid_out < table_min_rot_speed:
            #print "ServoPid min"
            target_rot_speed = table_min_rot_speed
        if self.pid_out > table_max_rot_speed:
            #print "ServoPid max"
            target_rot_speed = table_max_rot_speed
                
        servo = rot_speed_to_servo(target_rot_speed)
        
        if servo < 0:
            if servo >  -100: #-150:
                servo = 0
        else:             
            if servo < 350: #550:
                servo = 0
            
        if self.parent:
            if self.axis == 1:
                self.parent.servo1 = servo
            if self.axis == 2:
                self.parent.servo2 = servo
            

class MultiController(Controller):
    
    def __init__(self):  
          
        self.interface = None
        self.controllers = []
        
        self.current_angle1 = 0.
        self.current_pos1 = 0.
        self.target_angle1 = 0.
        self.target_position1 =  0.
        self.encoder1 = (encoder1_min + encoder2_max) /2
        self.load_cell1 = (load_cell1_min + load_cell1_max) /2
        self.servo1 = (servo_min + servo_max) /2
        self.servo2 = (servo_min + servo_max) /2

        self.current_angle2 = 0.        
        self.current_pos2 = 0.         
        self.target_angle2 = 0.        
        self.target_position2 =  0.        
        self.encoder2 = (encoder2_min + encoder2_max) /2
        self.load_cell2 = (load_cell2_min + load_cell2_max) /2
        
        
        self.filtered_pos1 = 0.
        self.filtered_pos2 = 0.
        
        self.filtered_load_cell1 = 0.
        self.filtered_load_cell2 = 0.
        
        self.error_pos2 = 0.
        self.error_pos1 = 0.
        
        self.pos_derivative1 = 0.
        self.pos_derivative2 = 0.
        
        self.pos_integral1 = 0.
        self.pos_integral2 = 0.
        
        self.filtered_encoder1 = 0.
        self.filtered_encoder2 = 0.
        
        self.filtered_servo1 = None
        self.filtered_servo2 = None
        
    def key_right(self):  
        for controller_data in self.controllers:
            controller_data[0].key_right()
    
    def key_left(self): 
        for controller_data in self.controllers:
            controller_data[0].key_left()
    
    def key_l(self): 
        for controller_data in self.controllers:
            controller_data[0].key_l()

    def key_s(self): 
        for controller_data in self.controllers:
            controller_data[0].key_s()
            
    def key_a(self): 
        for controller_data in self.controllers:
            controller_data[0].key_a()

    def key_f(self): 
        for controller_data in self.controllers:
            controller_data[0].key_f()

    def key_d(self): 
        for controller_data in self.controllers:
            controller_data[0].key_d()

    def key_g(self): 
        for controller_data in self.controllers:
            controller_data[0].key_g()
    
    def key_up(self):
        for controller_data in self.controllers:
            controller_data[0].key_up()

    def key_down(self):
        for controller_data in self.controllers:
            controller_data[0].key_down()
            
    def key_q(self):    
        for controller_data in self.controllers:
            controller_data[0].key_q()
            
    def key_w(self):    
        for controller_data in self.controllers:
            controller_data[0].key_w()   
            
    def key_z(self):    
        for controller_data in self.controllers:
            controller_data[0].key_z ()   
            
    def key_x(self):    
        for controller_data in self.controllers:
            controller_data[0].key_x ()   
            
    def key_c(self):    
        for controller_data in self.controllers:
            controller_data[0].key_c ()   
            
    def key_v(self):    
        for controller_data in self.controllers:
            controller_data[0].key_v()  
            
    def key_b(self):   
        for controller_data in self.controllers:
            controller_data[0].key_b()    
            
    def key_n(self):    
        for controller_data in self.controllers:
            controller_data[0].key_n ()
    
    def set_interface(self, interface, parent):        
        self.interface = interface
        self.parent = parent
        for controller_data in self.controllers:
            controller = controller_data[0]
            controller.set_interface(interface, self)
      
    def get_display_data(self):

        data = {

#        'current_angle1':self.current_angle1,
#        'current_pos1':self.current_pos1,
#        'target_angle1':self.target_angle1,
#        'target_position1':self.target_position1,        
#        'encoder1':self.encoder1,
#        'load_cell1':self.load_cell1,
#        'servo1':self.servo1,
#        'filtered_pos1':self.filtered_pos1,
#        'filtered_load_cell1':self.filtered_load_cell1,
#        'filtered_servo1':self.filtered_servo1,

        'current_angle2':self.current_angle2,
        'current_pos2':self.current_pos2 ,
        'target_angle2':self.target_angle2,
        'target_position2':self.target_position2 ,       
        'encoder2':self.encoder2,
        'load_cell2':self.load_cell2,
        'servo2':self.servo2,
        'filtered_pos2':self.filtered_pos2,
        'filtered_load_cell2':self.filtered_load_cell2,
        'filtered_servo2': self.filtered_servo2,
        
        #'error_pos2':   self.error_pos2,
        #'error_change2':self.error_change2,
        #'integral2':self.integral2,
        
        #'pos_derivative1' : self.pos_derivative1,
        'pos_derivative2' : self.pos_derivative2,
        
        'pos_integral1': self.pos_integral1,
        'pos_integral2':self.pos_integral2,

        
        }
            
        for controller_data in self.controllers:
            controller = controller_data[0]
            values = controller.get_display_data()
            for k in values.keys():
                if data.has_key(k): continue
                else: data[k] = values[k]
        return data
                
    def add_controller(self, controller, band):
        count_down = band
        step = 0
        last_sim_time = 0.
        controller_data = [controller, count_down, band, step,last_sim_time]
        self.controllers.append(controller_data)
    
    def _get_servo_safely(self):
        
        servo1_out = None
        servo2_out = None
        if self.filtered_servo1:
            servo1_out = self.filtered_servo1
        else:
             servo1_out =  self.servo1
             
        if self.filtered_servo2:
            servo2_out = self.filtered_servo2
        else:
             servo2_out =  self.servo2 
                           
        angle1 =  encoder1_to_angle(self.encoder1)
        angle2 =  encoder2_to_angle(self.encoder2)
        
        max = table_max_angle * 1.5
        min = table_min_angle * 1.5

        if angle1 > max and servo1_out > 0:
            servo1_out = 0
            print "SERVO1 SAFETY CUTOFF"
        if angle2 > max and servo2_out > 0:
            servo2_out = 0
            print "SERVO2 SAFETY CUTOFF"
        if angle1 < min and servo1_out <0:
            servo1_out = 0
            print "SERVO1 SAFETY CUTOFF"
        if angle2 < min and servo2_out < 0:
            servo2_out = 0
            print "SERVO2 SAFETY CUTOFF"        
        
        servo_out1 = int(servo1_out) 
        servo_out2 = int(servo2_out)
        return servo_out1, servo_out2
    
    def stop(self):
        servo1_int = 0
        dir1 = 0
        servo2_int = 0
        dir2 = 0
        error_code, encoder1, encoder2, load_cell1, load_cell2, filtered_encoder1, filtered_encoder2, filtered_load_cell1, filtered_load_cell2 = self.interface.send_receive(0, servo1_int, dir1, servo2_int, dir2)
        
            
    def loop(self, step, sim_time, delta_time):
        
        servo1_int,servo2_int  = self._get_servo_safely()
        
        dir1 = 0 # motor direction
        dir2 = 0
        
        if servo1_int < 0:
            servo1_int = -servo1_int
            dir1 = 1
        if servo2_int < 0:
            servo2_int = -servo2_int
            dir2 = 1
            
        error_code, encoder1, encoder2, load_cell1, load_cell2, filtered_encoder1, filtered_encoder2, filtered_load_cell1, filtered_load_cell2 = self.interface.send_receive(0, servo1_int, dir1, servo2_int, dir2)
        
        self.error_code = error_code
        
        self.load_cell1 = load_cell1
        self.load_cell2 = load_cell2
        
        self.encoder1 = encoder1
        self.encoder2 = encoder2
        
        self.current_pos1 = load_cell1_to_position(load_cell1, self.servo1)
        self.current_pos2 = load_cell2_to_position(load_cell2, self.servo2)
        
        self.current_angle1 = encoder1_to_angle(encoder1)
        self.current_angle2 = encoder2_to_angle(encoder2)
        
        self.filtered_load_cell1 = filtered_load_cell1
        self.filtered_load_cell2 = filtered_load_cell2
        
        self.filtered_pos1 = load_cell1_to_position(filtered_load_cell1)
        self.filtered_pos2 = load_cell2_to_position(filtered_load_cell2)
        
        self.filtered_encoder1 = filtered_encoder1
        self.filtered_encoder2 = filtered_encoder2
        
        
        finished = False
        for controller_data in self.controllers:
            # decrement counter
            controller_data[1] -= 1
            if controller_data[1] <= 0:
                # reinitialise counter
                band = controller_data[2]
                previous_time = controller_data[4]
                # adjust last sim time
                controller_data[4] = sim_time
                # adjust delta time
                delta = sim_time - previous_time
                if delta == 0.:
                    delta = 0.001
                step = controller_data[3]
                # increment step
                controller_data[3] += 1
                # execute controller
                controller_data[1] = band
                # fire it up
                controller = controller_data[0]
                f = controller.loop(step, sim_time, delta)
                finished = f or finished # True if a controller returns True
        
        return finished
    
class KeyboardController(Controller):
    
    def __init__(self):
        self.error_code = 0
        self.load_cell1 = 0
        self.load_cell2 = 0
        self.encoder1 = 0
        self.encoder2 = 0
        self.dir1 = 0
        self.servo1 = 0
        self.dir2 = 0
        self.servo2 = 0
        self.step = 0
        self.filtered_load_cell2 = 0
        self.filtered_load_cell1 = 0
        self.pos1 = 0.
        self.pos2 = 0.
        self.filtered_pos1 = 0.
        self.filtered_pos2 = 0.

    def _set_servo2(self, v):
        m = ""
        if v > servo_max:
            m = "MAX!"
            v = servo_max
            
        if v < servo_min:
            v = servo_min
            m = "MIN!"
        
        self.servo2 = v
        print "Servo2 %f %s" % (v, m)
    
    def _set_servo1(self, v):
        m = ""
        if v > servo_max:
            m = "MAX!"
            v = servo_max
            
        if v < servo_min:
            v = servo_min
            m = "MIN!"
        
        self.servo1 = v
        print "Servo1 %f %s" % (v, m)

    def key_a(self):
        self._set_servo1(0.)
        self._set_servo2(0.)

        
    def key_z(self):
        v = self.servo1 - (servo_max - servo_min) /10.
        self._set_servo1(v)
        
    def key_x(self):
        v = self.servo1 + (servo_max - servo_min) /10.
        self._set_servo1(v)
        
    def key_c(self):
        v = self.servo2 - (servo_max - servo_min) /10.
        self._set_servo2(v)
        
    def key_v(self):
        v = self.servo2 + (servo_max - servo_min) /10.    
        self._set_servo2(v)
            
    def get_display_data(self):
        data = {}
        
        #data['error_code'] =  self.error_code
        data['load_cell1'] =  self.load_cell1
        data['load_cell2'] =  self.load_cell2    
        data['filtered_load_cell2'] = self.filtered_load_cell2    
        data['encoder1'] =  self.encoder1
        data['encoder2'] =  self.encoder2 
        data['servo1']   = self.servo1
        data['servo2']   = self.servo2
        data['angle1']   = encoder1_to_angle(self.encoder1)
        data['angle2']   = encoder2_to_angle(self.encoder2)
        
        data['current_pos1']   = self.pos1
        data['current_pos2']   = self.pos2
        
        data['filtered_pos2'] = self.filtered_pos1
        data['filtered_pos1'] = self.filtered_pos2
        
        data['step'] = self.step
        return data   

    #'encoder2'
    #'angle2'
        
    def loop(self, step, time, delta_time):
        self.step +=  1
        #print 'loop' 
        stop = False
        servo1_int = None
        servo2_int =  None
        
        if stop:
            print "type in ENTER to quit or CMD MOT1 MOT2 and ENTER"
            msg = raw_input()
            if len(msg) ==0:
            # finished
                return True
            else:
                toks = msg.split()
                cmd = int(toks[0])
                mot1 = int(toks[1])
                mot2 = int(toks[2])
                servo1_int = int(mot1)
                servo2_int = int(mot2)
        else:
             servo1_int = int(self.servo1)
             servo2_int = int(self.servo2)   
             
        dir1 = 0 # motor direction
        dir2 = 0
        
        if servo1_int < 0:
            servo1_int = -servo1_int
            dir1 = 1
        if servo2_int < 0:
            servo2_int = -servo2_int
            dir2 = 1
            
        error_code, encoder1, encoder2, load_cell1, load_cell2, filtered_encoder1, filtered_encoder2, filtered_load_cell1, filtered_load_cell2  = self.interface.send_receive(0, servo1_int, dir1, servo2_int, dir2)
        
        self.error_code = error_code
        self.load_cell1 = load_cell1
        self.load_cell2 = load_cell2
        self.filtered_load_cell1 = filtered_load_cell1
        self.filtered_load_cell2 = filtered_load_cell2
        self.encoder1 = encoder1
        self.encoder2 = encoder2
        
        self.pos1 = load_cell1_to_position(load_cell1, self.servo1)
        self.pos2 = load_cell2_to_position(load_cell2, self.servo2)
        self.filtered_pos1 = load_cell1_to_position(filtered_load_cell1, self.servo1)
        self.filtered_pos2 = load_cell2_to_position(filtered_load_cell1, self.servo2)

#        self.dir1 = dir1
#        self.servo1 = servo1_int
#        self.dir2 = dir2
#        self.servo2 = servo2_int
            
        return False

class SineAngleTarget(Controller):
    
    def __init__(self):
        self.period = 3.0# in seconds
        self.zero = 0.
        self.amplitude = 0.001
        self.max_amplitude = table_max_angle
        self.time = 0.
        
    def key_q(self):
        
        self.period += 1.0
        print "PositionSetter.q period %f" % self.period
        
    def key_w(self):

        self.period -= 1.0   
        print "PositionSetter.q period %f" % self.period
          
    def loop(self, step, sim_time, delta_time):
        self.time += delta_time
        # reset clock to zero after period
        if self.time > self.period:
            difference = self.time - self.period
            self.time = difference
            self.amplitude = self.amplitude + 0.001
            if self.amplitude > self.max_amplitude:
                self.amplitude = self.max_amplitude
            
        phase = self.time / self.period
        angle = phase * 2* math.pi
        pos  = math.sin(angle) 
        pos = self.zero + pos *  self.amplitude
        pos = pos * (ball_pos_max - ball_pos_min)
   
        if self.parent:
            self.parent.target_angle1 = pos

class PositionSetter(Controller):
    
    def __init__(self, axis):
        self.period = 30.0# in seconds
        self.zero = 0.
        self.amplitude = 0.05
        self.max_amplitude = 0.2
        self.time = 0.
        self.axis = axis
        
    def key_q(self):
        print "PositionSetter.k"
        self.period += 1.0
        
    def key_w(self):
        print "PositionSetter.w"
        self.period -= 1.0   
         
    def loop(self, step, sim_time, delta_time):
        self.time += delta_time
        # reset clock to zero after period
        if self.time > self.period:
            difference = self.time - self.period
            self.time = difference
            self.amplitude = self.amplitude + 0.05
            if self.amplitude > self.max_amplitude:
                self.amplitude = self.max_amplitude
            
        phase = self.time / self.period
        angle = phase * 2* math.pi
        pos  = math.sin(angle) 
        pos = self.zero + pos *  self.amplitude
        pos = pos * (ball_pos_max - ball_pos_min)
   
        if self.parent:
            if self.axis == 1:
                self.parent.target_position1 = pos
            if self.axis == 2:
                self.parent.target_position2 = pos
                
    def get_display_data(self):    
        return {}
        #angle = self.angle
        #return locals()

        
            

def scale(value, in1, in2, out1, out2):
    range_in = in2 - in1
    range_out = out2 - out1
    resp = out1 + range_out * (value - in1) / range_in 
    return resp

def rot_speed_to_servo(rot_speed):
    
    min = servo_min
    max = servo_max
    
    int_resp = 0
    if rot_speed > 0:
        min = 300.
        max = servo_max
        resp = scale(rot_speed, 0., table_max_rot_speed, min, max)
        int_resp =  int (resp)
        
    if rot_speed < 0:
        min = servo_min 
        max = 0.
        resp = scale(rot_speed, table_min_rot_speed, 0., min, max)
        int_resp =  int (resp)
    #resp = scale(rot_speed, table_min_rot_speed, table_max_rot_speed, min, max)
    #int_resp = int(resp) # convert to integer
    return int_resp
        
def servo_to_rot_speed(servo):
     resp = scale(servo, servo_min, servo_max, table_min_rot_speed, table_max_rot_speed)
     return resp
    
def encoder1_to_angle(encoder):
    resp = scale(encoder, encoder1_min, encoder1_max, table_min_angle, table_max_angle)
    return resp

def encoder2_to_angle(encoder):
    resp = scale(encoder, encoder2_min, encoder2_max, table_min_angle, table_max_angle)
    return resp

def angle1_to_encoder(angle):
    resp = scale(angle, table_min_angle, table_max_angle, encoder1_min, encoder1_max)
    return resp

def angle2_to_encoder(angle):
    resp = scale(angle, table_min_angle, table_max_angle, encoder2_min, encoder2_max)
    return resp
    
def load_cell1_to_position(load_cell, servo = 0):
    resp = scale(load_cell, load_cell1_min, load_cell1_max, ball_pos_min, ball_pos_max)
    return resp

def load_cell2_to_position(load_cell, servo = 0):
    resp = scale(load_cell, load_cell2_min, load_cell2_max, ball_pos_min, ball_pos_max)
    return resp

def position_to_load_cell1(position, servo = 0):
    resp = scale(position, ball_pos_min, ball_pos_max, load_cell1_min, load_cell1_max)
    return int(resp)

def position_to_load_cell2(position, servo = 0):
    resp = scale(position, ball_pos_min, ball_pos_max, load_cell2_min, load_cell2_max)
    return int(resp)


class KeplerSimInterface(object):
    
    def __init__(self, simulator):
        self.simulator = simulator
        
    def send_receive(self, cmd, *args):
        #servo_1 = args[0]
        #servo_2 = args[1]
        
        servo1_int = args[0]
        dir1       = args[1] 
        servo2_int = args[2] 
        dir2       = args[3]
        
        servo_1 = servo1_int
        if dir1 == 1:
            servo_1 = -servo_1
        servo_2 = servo2_int
        if dir2 == 1:
            servo_2 = -servo_2
        
        alpha_rot = servo_to_rot_speed(servo_1)
        beta_rot = servo_to_rot_speed(servo_2)
        
        self.simulator.set_table_rotation_speed(alpha_rot, beta_rot)
        
        alpha, beta = self.simulator.get_table_inclination()
        y,x = self.simulator.get_ball_position()
        
        x = x /4.5
        y = y / 4.5
        x = x * ball_pos_max
        y = y * ball_pos_max 
        error_code = 0
        encoder1 = angle1_to_encoder(alpha)
        encoder2 = angle2_to_encoder(beta)
        filtered_encoder1 = encoder1
        filtered_encoder2 = encoder2
        
        load_cell1 = position_to_load_cell1(x, servo_1)
        load_cell2 = position_to_load_cell2(y, servo_2)
        
        filtered_load_cell1 = load_cell1
        filtered_load_cell2 = load_cell2
        
        return error_code, encoder1, encoder2, load_cell1, load_cell2, filtered_encoder1, filtered_encoder2, filtered_load_cell1, filtered_load_cell2
            
class EchoInterface(object):
    
    def __init__(self, delay):
        print "EchoInterface init"
        self.delay = delay
        
        self.error_motor1 = None
        self.error_motor2 = None
        self.error_encoder = None
        self.error_load_cell1 = None
        self.error_load_cell2 = None
    

    
    def  send_receive(self, cmd, *args):
        print
        print 'EchoInterface.send_receive'
        print "cmd: ", cmd
        print "args: ", args
        mot_1 = args[0]
        mot_2 = args[1]
        
        if self.error_motor1:
            err = self.error_motor1()
            mot_1 += err
        
        if self.error_motor2:
            err = self.error_motor2()
            mot_2 += err    
        
        load_1 = 0
        load_2 = 0
        
        time.sleep(self.delay)
        return 0, mot_1, mot_2, load_1, load_2 
        # return error_code, encoder1, encoder2, load_cell1, load_cell2
    
        
            
class KeplerSerialComm(object):
    
    def __init__(self, serial_port_name, bauds, format_tx, format_rx, timeout):
        print 'Opening serial port: ', serial_port_name
        s1 = struct.calcsize(format_tx)
        print "TX format: %s nb of bytes %s " % (format_tx, s1)
        s2 = struct.calcsize(format_rx)
        print "RX format: %s nb of bytes %s " % (format_rx, s2)
        if is_windows():
            serial_port_name = int(serial_port_name)
        self.port = serial.Serial(serial_port_name, bauds, timeout=1)

    def  send_receive(self, cmd, *args):
        #print "cmd: ", cmd
        #print "args: ", args
        servo1 = args[0]
        dir1 = args[1]
        servo2 = args[2]
        dir2 = args[3]
        # print "Command: '%s', servo1 '%s' servo2 '%s' " % (cmd, servo1, servo2)

        mot1l, mot1h  = split_14bit_number(servo1)
        mot2l, mot2h  = split_14bit_number(servo2)
        
         
        pack = struct.pack(format_tx, cmd, mot1l, mot1h, dir1, mot2l, mot2h, dir2, 255)
        # print_binary_data(pack)
        
        self.port.write(pack)
        message_size = struct.calcsize(format_rx)
        response = read_response(self.port, message_size )
        #print
        #print "received packed data:"
        #print_binary_data(response)
        try:
            datas = struct.unpack(format_rx, response)
       
            error_code = datas[0]
            
            load_cell1_l = datas[1]
            load_cell1_h = datas[2]
            load_cell2_l = datas[3]
            load_cell2_h = datas[4]
            # load cell 3  position 5 and 6
            enc1_l = datas[7]
            enc1_h = datas[8]
            enc2_l = datas[9]
            enc2_h = datas[10]
            
            filtered_loadcell1_l = datas[5]
            filtered_loadcell1_h = datas[6]
            filtered_loadcell2_l = datas[11]
            filtered_loadcell2_h = datas[12]
            
            filtered_enc1_l = 0#datas[17]
            filtered_enc1_h = 0#datas[18]
            filtered_enc2_l = 0#datas[19]
            filtered_enc2_h = 0#datas[20]
#            
            term_byte = datas[15]
            
            #print "Term byte = %s" % term_byte
                
            encoder1 = assemble_14bit_number(enc1_h, enc1_l)
            encoder2 = assemble_14bit_number(enc2_h, enc2_l)             
            load_cell1 = assemble_14bit_number(load_cell1_h, load_cell1_l)
            load_cell2 = assemble_14bit_number(load_cell2_h, load_cell2_l)
            
            filtered_load_cell1 = assemble_14bit_number(filtered_loadcell1_h, filtered_loadcell1_l)
            filtered_load_cell2 = assemble_14bit_number(filtered_loadcell2_h, filtered_loadcell2_l)
            
            filtered_encoder1 = assemble_14bit_number(filtered_enc1_h, filtered_enc1_l)
            filtered_encoder2 = assemble_14bit_number(filtered_enc2_h, filtered_enc2_l)             
            
            # if error_code != 0:
            #    print "Error code: %d" % error_code
            
            # print "Encoder low: %d" % enc1_l
            # print "Encoder high: %d" % enc1_h
            # print "Load cell low: %d" % load_cell1_l
            # print "Load cell high: %d" % load_cell1_h
            
            # print "Encoder[1] %s, encoder[2] %s, load cell[1] %s load cell[2] %s" % (encoder1, encoder2, load_cell1, load_cell2)
            #print "error_code %s, encoder1 %s, encoder2 %s, load_cell1 %s, load_cell2 %s, filtered_encoder1 %s, filtered_encoder2 %s, filtered_load_cell1 %s, filtered_load_cell2 %s" % (error_code, encoder1, encoder2, load_cell1, load_cell2, filtered_encoder1, filtered_encoder2, filtered_load_cell1, filtered_load_cell2)
            return error_code, encoder1, encoder2, load_cell1, load_cell2, filtered_encoder1, filtered_encoder2, filtered_load_cell1, filtered_load_cell2
            
            
        except Exception, e:
            print "Error: ", e

class Loop(object):
    
    def __init__(self, controller):
        self.start_time = time.time()
        self.last_time = 0.
        self.time =0.
        self.log = KeplerLogger()
        self.step = 0.
        self.controller = controller
        self.logging = True
        self.text_band = 100
        self.text_count_down = 1
        self.text_time = time.time()
    
    def _refresh_text(self, txt):
        
        self.text_count_down -=1
        if self.text_count_down > 0:
            return
        
        lasttime = self.text_time
        self.text_time = time.time()
        delta = self.text_time - lasttime
        fps = self.text_band / delta
        print "fps %s" % fps    
        self.text_count_down = self.text_band
        print "time %s" % self.time
        
        if self.logging:
            return
        
        data = self.controller.get_display_data()
        for k,v in data.iteritems():
            print "%s %.10f" % (k,v)
        
        
    def loop(self):

        self.time = time.time() - self.start_time
        delta_time = self.time - self.last_time
        # avoid division by 0
        if delta_time < 0.0001:
            delta_time = 0.0001
        self.last_time = self.time
        self.step += 1
        txt = ""
        self._refresh_text(txt)
        self.controller.loop(step, self.time, delta_time)
        
        if self.logging:
            data = self.controller.get_display_data()
            self.log.snapshot(get_data_logger(), self.step, self.time, data, ('self'))        
        
        return False #Task.cont    


 
    
#        time.sleep(time_step)
#        now = time.time()
#        delta_time = now - epoch 
#        step += 1
#        sim_time += delta_time
#        finished = controller.loop(step, sim_time, delta_time)

#from direct.showbase.DirectObject        import DirectObject
#from direct.task                         import Task
#from direct.gui.OnscreenText             import OnscreenText, TextNode
#import direct.directbase.DirectStart 
#import sys


class AngleController(Controller):
    
    def __init__(self, angle_error_min, angle_error_max, rot_speed_error_min, rot_speed_error_max, out_rot_speed_min, out_rot_speed_max):
        self.target_angle1 = 0.
        self.target_angle2 = 0.
        self.encoder1 = angle_to_encoder(0.)
        self.encoder2 = angle_to_encoder(0.)
        self.servo1 = rot_speed_to_servo(0.)
        self.servo2 = rot_speed_to_servo(0.)
        self.fuzz = get_angle_controller(angle_error_min, angle_error_max, rot_speed_error_min, rot_speed_error_max, out_rot_speed_min, out_rot_speed_max)
    
    def get_display_data(self):    
      
        encoder1 = self.encoder1
        angle = self.current_angle1
        target_angle = self.target_angle1
        rot_speed =  self.current_rot_speed1
        
        angle_error1 = self.error
        rot_speed_error1= self.speed_error 
        
        fuzzy_in1 =  self.fuzz._sigs_in['err'].evaluation.values()
        fuzzy_in2 =  self.fuzz._sigs_in['speederr'].evaluation.values()
        rule_activations = []
        fuzzy_rules = self.fuzzy_rules
     
        fuzzy_rot_speed1 = self.fuzzy_rot_speed1  
        fuzzy_servo = self.servo1 
        return locals()
            
    def set_angle(self, target_angle):
        self.target_angle = target_angle
    
    def _calc_rot_speed(self, angle, rot_speed, target_angle ):
        error = target_angle - angle
        speed_error = 0 - rot_speed
        self.error = error
        self.speed_error = speed_error
        #print "Angle error: %.5f" % error
        #print "speed error: %.5f" % speed_error
        fuzzy_in = {'err':error,'speederr':speed_error}
        fuzzy = self.fuzz.evaluate(fuzzy_in)
        self.fuzzy_rules = []
        for rule in self.fuzz._rules:
            self.fuzzy_rules.append(rule.evaluation)
        # print "  fuzzy ctl: %.5f" % fuzzy
        self.fuzzy_rot_speed = fuzzy
        return fuzzy
        
    def loop(self, step, sim_time, delta_time):
        error_code, encoder1, encoder2, load_cell1, load_cell2 = self.interface.send_receive(0, self.servo1, self.servo2)
        if error_code > 0:
            print "Error %s while communicating with the interface" % error_code
            return
        
        self.encoder1 = encoder1
        self.current_angle1 = encoder1_to_angle(self.encoder1)
        self.current_rot_speed1 =  servo_to_rot_speed(self.servo1)
        self.fuzzy_rot_speed1 = self._calc_rot_speed(self.current_angle1, self.current_rot_speed1,  self.target_angle1)
        self.servo1 = rot_speed_to_servo(self.fuzzy_rot_speed1)
        
        
class AngleSetter(Controller):
    
    def __init__(self, angle_ctrl, steps):
        self.steps = steps
        self.angle_ctrl = angle_ctrl
        self.angle = 0.
    
    def key_up(self):
        self.angle += 0.001
        print "Angle is %f" % self.angle
    
    def key_down(self):
        self.angle -= 0.001
        print "Angle is %f" % self.angle
        
    def get_display_data(self):    
        angle = self.angle
        return locals()
        
    def loop(self, step, time, delta_time):
        self.angle_ctrl.set_angle(self.angle)

class KalmanFilter(Filter):
    
    def __init__(self, band):
        Filter.__init__(self, band)
        R = 0.1 #0.1**2 # estimate of measurement variance, change to see effect
        Q = 1e-5 # process variance
        self.calman = calman.Filter(R, Q)
    
    def eval(self, history):
        return self.calman.filter( history[-1])
    


class Application( kepler_sim.WorldBase ):   
    #function that initializes everything needed 
      
    def __init__( self, controller, log_2_memory ):
        kepler_sim.WorldBase.__init__(self, controller, log_2_memory)
        
             
        
    
    def _get_data(self):
        
        return data
            




#
# XXX yyy
#                
if params.loadParams():
    table_min_angle = math.asin(table_min_height/table_distance_from_center)
    table_max_angle = math.asin(table_max_height/table_distance_from_center)
    print "Min table angle in rads %.4f" % table_min_angle
    print "Max table angle in rads %.4f" % table_max_angle
    print "Table min rot speed  %f" % table_min_rot_speed
    print "Table max rot speed  %f" % table_max_rot_speed
    
    controller = None
    if  controller_name == 'keyboard':    
        controller = KeyboardController()
    if controller_name == 'servo':
        controller = ServoController()
        
    if controller_name == 'sine':
        controller = MultiController()
        if filter_name == 'mean':    
            filter_band = 10
            pos_filter = MeanFilter(filter_band)
        controller.add_controller(pos_filter, 1)  
        ang_controller = SineAngleTarget()
        ang_band = 10
        controller.add_controller( ang_controller, ang_band)
        angle_band = 1
        angle_controller = ServoPid(-0.01, 0., 0.) 
        controller.add_controller( angle_controller, angle_band)
        
    if controller_name == 'pid_pid':
        
        servo_pid1 = False
        pid1 = False
        position_setter1 = False
        
        servo_pid2 = True
        pid2 = True
        position_setter2 = False
        
        controller = MultiController()
        
        
        
        angle_band = 1
        filter_band = 1
        pid_band = filter_band
        
        servo_filter1 = ServoMeanFilter(1, 5)
        servo_filter2 = ServoMeanFilter(2, 5)
        controller.add_controller( servo_filter1, filter_band)
        controller.add_controller( servo_filter2, filter_band)
#        pos1_filter = None
#        pos2_filter = None
#        if filter_name == 'none':    
#            pos1_filter = Filter(1, filter_band)
#            pos2_filter = Filter(2, filter_band)
#        if filter_name == 'mean':    
#            pos1_filter = MeanFilter(1, filter_band)
#            pos2_filter = MeanFilter(2, filter_band)
#            
            
            
#        if filter_name == 'Kalman':
#            filter_band = 1
#            pos_filter = KalmanFilter(filter_band)
#        controller.add_controller(pos1_filter, 1)    
#        controller.add_controller(pos2_filter, 1)    
        
            
        deriv1 = Derivator(1)
        deriv2 = Derivator(2)
        controller.add_controller( deriv1, filter_band)
        controller.add_controller( deriv2, filter_band)    
    
        keyboard = False
        show_data = False
        if servo_pid1:
            angle_controller1 = ServoPid(1, kp_s1, ki_s1, kd_s1, keyboard, show_data)
            controller.add_controller( angle_controller1, angle_band)
            
        if servo_pid2:
            angle_controller2 = ServoPid(2, kp_s2, ki_s2, kd_s2, keyboard, show_data)
            controller.add_controller( angle_controller2, angle_band)
        
        
        filter_error = True # false to use raw error
        kc = 1.0
        if pid1:
            pid_controller1 = PidController(1,kp_p1, ki_p1, kd_p1, kc, filter_error) # split_pid 05
            controller.add_controller( pid_controller1, pid_band)
            
        if pid2:
            pid_controller2 = PidController(2,kp_p2, ki_p2, kd_p2, kc, filter_error) # split_pid 05
            controller.add_controller( pid_controller2, pid_band)
        
        #(-0.025,0,-0.5) #(-0.015, 0., 0.002)
        #(-0.01,0., 0.)#-0.0000000125,-0.01175)
        
        #(-0.001, -0.000000000001, -0.003) #13(-0.008, -0.000000000001, -0.032)
        #mean11(-0.0165, -0.00000000005, -0.064)
        #(-0.009,-0.000000018,-0.02)
        #mean09(-0.0075,-0.0000000125,-0.01175)
        #mean08(-0.015,-0.000000025,-0.035)
        #mean07(-0.035,-0.00000005,-0.075)
        #mean 05(-0.01,-0.0000005,-0.01)
        #(-0.008,-0.0005,-0.0115)
        #calman(-0.01,-0.0000000,-0.005)
        
        # calman(-0.02,-0.0000001,-0.05)
        #pidpid6(-0.04,-0.0000001,-0.1)
        #pidpid05(-0.07,-0.0000001,-0.15)
        #(-0.14,-0.0000001,-0.3)
        #pidpid_02(-0.12,-0.0000001,-0.3)
        # pidpid_01(-0.24,-0.000,-0.8)
        #pidpid8.log(-0.025,-0.001,-2.)
        #pipid7.log(-0.05,0,-2.5)
        #pidpid6(-0.05,0,-1.)
        #(-0.025, 0,-0.32)
        #(-0.025, 0,-0.16) #(-0.025, 0., -10.)
        pid_band = 1
        #controller.add_controller( pid_controller1, pid_band)
        
        if position_setter1: 
            pos_controller1 = PositionSetter(1)
            pos_band = 1
            controller.add_controller( pos_controller1, pos_band)
        if position_setter2: 
            pos_controller2 = PositionSetter(2)
            pos_band = 1
            controller.add_controller( pos_controller2, pos_band)
        
        
    if controller_name == 'pid':
        controller = MultiController()
        servo_band = 1
        servo_controller = ServoController() 
        controller.add_controller( servo_controller, servo_band)
        pid_controller = PidController(-0.02, -0.0000000005, -0.8)
        pid_band = 1
        controller.add_controller( pid_controller, pid_band)
        
    if controller_name == 'servo_pid':
        controller = MultiController() 
        #servo_pid05 -8,0,0
        #servo_pid06 -6, 0, 0 
        #servo_pid07 -6, 0, 0 
        pid1 = True
        show_data = True
        if pid1:
            servo1 = ServoPid(1, kp_s1, ki_s1, kd_s1, False, show_data)
            controller.add_controller( servo1, 1)
        servo2 = ServoPid(2, kp_s2, ki_s2, kd_s2, True, show_data)
        controller.add_controller( servo2, 1) 
         
    if controller_name == 'fuzzy_v1':
        
        controller = MultiController()
        # angle_controller = KeyboardController()
        angle_error_min = table_min_angle
        angle_error_max = table_max_angle 
        rot_speed_error_min = table_min_rot_speed
        rot_speed_error_max = table_max_rot_speed
        out_min = math.sin(table_min_rot_speed)
        out_max = math.sin(table_max_rot_speed)
        angle_controller = AngleController(angle_error_min, angle_error_max, rot_speed_error_min, rot_speed_error_max, out_min, out_max)
        band = 1
        controller.add_controller( angle_controller, band)
        governer = AngleSetter(angle_controller,[(5.,0.), (10.,table_max_angle /2 ), (15.,table_max_angle )])
        controller.add_controller( governer, band)
   
    interface = None
    if interface_name == 'echo':
        interface = EchoInterface(0.)
        app = Application(controller, log2memory)
    if interface_name == 'serial':  
        interface = KeplerSerialComm(serial_port_name, bauds, format_tx, format_rx, 1)
        app = Application(controller, log2memory)
    if interface_name ==  'serialASAP':    
        interface = KeplerSerialComm(serial_port_name, bauds, format_tx, format_rx, 1)
        app = Loop(controller)
            
    if interface_name == 'simulator':
        app = kepler_sim.World(controller,log2memory)
        interface = KeplerSimInterface(app)  
    interface.error_motor1 = makeError(2,1) 
    controller.set_interface(interface,None)
  
    epoch = time.time()
    sim_time = 0.
    step = 0
    time_step = 0.01
    delta_time = 0.
    
    import Tkinter

    
    Tkinter.tkinter.quit()


# first time?
#controller.loop(step, sim_time, delta_time)

no = True
if no:
    if type(app) == type(Loop):
        finished = False
        while not finished:
            #time.sleep(0.0)
            app.loop()
    
    
 
# Panda    
run()     
print "QUITTING"
    

