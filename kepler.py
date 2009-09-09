#!/usr/bin/python

'''
Created on 2009-06-16

@author: hugo
'''
import math
import hugomatic.toolkit # UI and cmdline stuff
import hugomatic.code    # gcode routines
import hugomatic.metrics as metrics

def take_a_break():
    """
    This function is called when the debug line is printed. Add a breakpoint here
    but don't call print, because the program will go into an infinite loop.
    """
    a = 42

params = hugomatic.toolkit.Parameters('Kepler', 
                                      'Characterisation', 
                                      picture_file="kepler.gif",
                                      debug_callback=take_a_break)

#units = 'Inches'  
#params.addArgument( units, 'Units... 1 inch = 25.4 millimeters', choices=('Inches', 'mm'), group='setup')
ball_radius = 0.02
params.addArgument(ball_radius , 'Ball radius in m', group='dimensions')

ball_density = 7850. # 7.850 g/cm-3 mild steel (0.284 lb/in3)
params.addArgument(ball_density, 'Ball density in kg/m^3', group='dimensions')

table_x = metrics.inches_to_mm(24.) / 1000.
params.addArgument(table_x, 'Table dimension along x in m', group='dimensions')

table_y = metrics.inches_to_mm(24.) / 1000.
params.addArgument(table_y, 'Table dimension along y in m', group='dimensions')

table_z = metrics.inches_to_mm(3./8.) / 1000.
params.addArgument(table_z, 'Table dimension along z in m(thickness)', group='dimensions')
                   
table_density = 2700. # 2.70 g/cm^3 aluminium
params.addArgument(table_density , 'Table density in kg/m^3', group='dimensions')

actuator_force = 10. 
params.addArgument(actuator_force , 'Actuator max force in N', group='actuators')
actuator_lever = 10.
params.addArgument(actuator_lever, 'Actuator lever (distance from center) in m') 
actuator_dx = 0.1                    
params.addArgument(actuator_dx, 'Actuator displacement in m')
                   
#Moment of inertia:
#- solid sphere, radius r: 
#  I = m(2/5)*r^2 (along any diameter)
#
#- thin rod, length l: 
#  I = ml^2/12 (normal to length, at center)
#
#- solid cylinder, radius r, length l:
#  I = mr^2 (along length)
#  I = m/12 (3r^2 + h^2)
#

    
    


def fisiks():
    print """
references

    1 inch = 0.0254 meters
    1 meter = 3.2808399 feet

    F = dP / dt Total force is derivation of linear momentum over time
    N = dL / dt Total torque is derivation of angular momentum over time
    Torque unit: N*m

    P = MV Linear momentum is mass * velocity
    L = Iw angular momentum is moment of inertia tensor * rotational velocity
    
    http://en.wikipedia.org/wiki/List_of_moments_of_inertia
    http://en.wikipedia.org/wiki/List_of_moment_of_inertia_tensors
"""
    print
    print "Sphere volume and mass (radius=%s m, density=%s kg/m**3)" % (ball_radius, ball_density)
    ball_volume = metrics.sphere_volume(ball_radius)
    ball_mass = ball_density * ball_volume 
      
    print "Volume of sphere = %s m**3" % (ball_volume)
    print "Mass of sphere  = %s kg" % (ball_mass) 
    gravity = -9.81
    ball_gravity = ball_mass * gravity
    print "Gravity force on sphere = m * g = %f" % (ball_gravity)
    
    ball_moment_of_inertia =  (2./5.) * ball_mass * pow(ball_radius,2) 
    print """Moment of inertia:
  solid sphere, radius r: 
  I = m(2/5)*r^2 (along any diameter) = %s"""% (ball_moment_of_inertia)

    print 
    print "Plate stuff"
    table_volume = table_x * table_y * table_z
    table_mass = table_volume * table_density 
      
    print "Volume of table x * y * z = %s m**3" % (table_volume)
    print "Mass of table  = %s kg" % (table_mass)
    a2 = pow(table_x, 2)
    b2 = pow(table_y, 2)
    i1 = 0.5 * table_mass * (a2 + b2)
    i2 = table_mass * b2 / 12.
    i3 = table_mass * a2 / 12. 
    print """Inertia tensor thin rectangular sheet, sides (a, b):
  I1 = m(a^2+b^2)/2 (normal to sheet, at center) = %s
  I2 = mb^2 /12 (parallel to a, at center) = %s
  I3 = ma^2/12  (parallel to b, at center) =%s
"""  % (i1, i2, i3)
    
    print
    print "Ball is at rest, over actuator"
    print "   "
    sum_force_up = ball_gravity + actuator_force
    sum_force_down = -actuator_force + ball_gravity
    print "   force up = actuator - gravity = %f N, acceleration %f m/s^2" % (sum_force_up,  sum_force_up / ball_mass)
    print "   force down = -actuator - gravity = %f N, acceleration %f m/s^2" % (sum_force_down, sum_force_down / ball_mass)
    print "   force rest = gravity = %f N, (acceleration is %f m/s^2)" % (ball_gravity, gravity)
    
    actuator_torque = actuator_lever * actuator_force
    # ball_torque = 
    print
    print "Table rotation"
    print "  actuator torque L = force * lever = %f" % actuator_torque
    ang_acc = actuator_torque / i2
    lin_acc = ang_acc * actuator_lever
    print "  angular accel L/I = %f (rad/s^2), linear accel = angular acc * lever = %f (m/s^2)" %  (ang_acc, lin_acc)
    
    #random.gauss(3,1)
    #3.5199474402918183
    
    #print "%X" % 1599
    #63F

    #int('0xf',16)
    #15

#>>> from struct import *
#>>> pack('HH', 1, 2, 3)
#'\x00\x01\x00\x02\x00\x00\x00\x03'
#>>> unpack('hhl', '\x00\x01\x00\x02\x00\x00\x00\x03')
#(1, 2, 3)
#>>> calcsize('hhl')
#8
# '<BHHHHHHHB'

    
if __name__ == '__main__':
    if params.loadParams():
        fisiks()