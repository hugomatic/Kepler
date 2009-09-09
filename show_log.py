#!/usr/bin/python

import commands
import sys



file_name = sys.argv[1]
 
#print commands.getoutput("mv kepler.log kepler2.log")


import pylab

class KeplerLogger(object):
        
    def snapshot(self, logger, step, sim_time, data, exclude_list = ()):
        logger.debug("SIM_STEP %s SIM_TIME %s" % (step, sim_time) )
        for k,value in data.iteritems():
            if k not in exclude_list:
                s = "%s = %s " %  (k, value)
            logger.debug(s)
    
    def load_from_file(self, file_name):
        self.history = {'step':[], 'time':[]}
        logfile = open(file_name,'r')
        try:
            lines = logfile.readlines()
        finally:
            logfile.close()
            
        for line in lines:
            line = line.strip() # remove \n
            key_eq_val = line.split(':')[-1]
            key_eq_val = key_eq_val.strip()
            #print " key_eq_val : [" + key_eq_val + "]"
            if key_eq_val.startswith( 'SIM_STEP'):
                toks = key_eq_val.split()
                step = toks[1]
                time = toks[3]
                step = int(step)
                time = float(time)
                self.history['step'].append(step)
                self.history['time'].append(time)
            else:
                # set one of the data values
                key,val = key_eq_val.split('=')
                key = key.strip()
                val = val.strip()
                if val == "None":
                    print "None data for %s" % key
                else:
                    if not val.startswith('['):
                        try:
                            val = float(val)
                        except:
                            val = 0.
                            #import sys, traceback
                            #traceback.print_stack( file = sys.stdout)
                            print "***** key  %s, val %s" % (key,val)
                            
                            
                    if not self.history.has_key(key):
                        self.history[key] = []    
                    self.history[key].append(val)



data_log = KeplerLogger()
data_log.load_from_file(file_name)

data = data_log.history



min_max_values = {}

print
print "data available"

for k,v in data.iteritems():
    
    lo = v[0]
    hi = v[0]
    for o in v:
        if o < lo:
            lo = o
        if o > hi :
            hi = o
    min_max_values[k] = (lo,hi)
    print " %s = range [%s, %s] %s samples" % (k, lo,hi, len(v) )

def norm(data, scale = 1.0, offset = 0.):
    norm_data = []
    v = data
    lo = v[0]
    hi = v[0]
    for o in v:
        if o < lo:
            lo = o
        if o > hi :
            hi = o
    delta = hi - lo
    for o in v:
       if delta == 0 : # constant
            normalized_value = 0.5
       else:
            normalized_value =  (o - lo) / delta
            #normalized_value = 2. * normalized_value - 1.
            norm_data.append(normalized_value * scale + offset)
    return norm_data
            
steps = data['step']
times = data['time']            
stepcount = steps[-1] - steps[0]
duration = times[-1] - times[0]
average = duration / stepcount
hz = 1. / average
print "Hz %s steps %d duration %f average %s " % (hz, stepcount, duration, average)

pylab.figure()

def enc_normalise(v):
    return (v - 2000. )/ 4000.

def get_label(name):
    lo,hi = min_max_values[name]
    label = "%s [%s, %s]" % (name, lo,hi)
    return label


angle_target = None
angle = None
err = None


def add_plot(name, normalise = False, scale = 1.0, offset = 0.0):
    print "add_plot ", name
    list_of_values = data[name]
    if normalise:
        list_of_values = norm(list_of_values, scale, offset)
    label = get_label(name)
    pylab.plot(times, list_of_values, label = label)


plot_all = False
servo_pid_2 = True
servo_pid_1 = False
servo_pid_1_pid = False
positions_2 = False
derivative2 = False
filter2 = False
rotation_speed = False





if plot_all:
    for name, points in data.iteritems():
        if name not in ["time", "step"]:
            if name[-1] != "1":
                add_plot(name, True)

if rotation_speed:
    add_plot('servo2', True)
    #add_plot('servo2')
    add_plot('angle2', True)
    
if filter2:
    add_plot('load_cell2')
    add_plot('filterd_load_cell2')
    add_plot('current_pos2')
    add_plot('filtered_pos2')
    
if derivative2:
    add_plot('current_pos2')
    add_plot('filtered_pos2')
    add_plot('error_pos2')
    add_plot('error_delta_pos2')
    add_plot('pid_out2')
    add_plot('pid_op2')
    add_plot('pid_od2')
    
if positions_2:  
    #add_plot('load_cell2')
    add_plot('current_angle2')
    #add_plot('current_pos2')
    add_plot('filtered_pos2')
    add_plot('target_position2')
    #add_plot('servo2')
    #add_plot('pos2_error')
    add_plot('target_angle2')
    #add_plot('pid_out2')
    #add_plot('pid_op2')
    #add_plot('pid_od2')

if servo_pid_2:
#    add_plot('target_angle2')
    add_plot('servo_pid_error2')
    add_plot('servo_pid_out2')
#    add_plot('servo_pid_op2')
#    add_plot('servo_pid_oi2')
#    add_plot('servo_pid_od2')
    
    #add_plot('current_angle2')
    #add_plot('angle2_error')
    add_plot('servo2', True)   
    add_plot('encoder2', True)
    
if servo_pid_1:
    add_plot('target_angle1')    
    add_plot('current_angle1')
    add_plot('angle1_error')
    add_plot('servo_pid_out1')
    
if servo_pid_1_pid:
    add_plot('angle1_error')
    #add_plot('current_angle1')
    add_plot('servo_pid_op1')
    add_plot('servo_pid_oi1')
    add_plot('servo_pid_od1')    
    

#pylab.plot(times, norm(data['motor1'], 0.1, ), 'r--', label='motor1')
#pylab.plot(times, norm(data['servo1'], 0.1), 'b--', label='servo1')
#pylab.plot(times, pid_out, 'r-', label='servo pid1')


#pylab.plot(norm_data['pid_error'], 'g-', label='op')
#pylab.plot(norm_data['pid_derivative'], 'b-', label='od')

#pylab.plot(pos,'r-',label='position')
#pylab.plot(filtered_pos,'b-',label='filtered pos')
#pylab.plot(pos_dif,'y-',label='pos - filter')
#pylab.plot(target_pos,'r-',label='target')

pylab.legend()
pylab.show()
