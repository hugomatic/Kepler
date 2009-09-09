#!/usr/bin/python

import hugomatic.toolkit    # the GUI stuff
import hugomatic.code       # the GCODE routines

      
def takeAbreak():
    """This function is called when the debug line is printed. Add a breakpoint here
    but don't call print, because the program will go into an infinite loop."""
    a = 42
    
# Create the Parameters object. It is used to create the GUI and set values in global variables
params = hugomatic.toolkit.Parameters('Load cell pillow connector', '1.125 stock, origin  is left side, center', 
                                      picture_file="kepler.gif", # picture on the left
                                      debug_callback=takeAbreak)

# When publishing numbers. the default value determines the type (int or float...)                    
feed = 2.0 
params.addArgument(feed,  'Feed rate in inches per minute', group='setup' )
cut = 0.05
params.addArgument(cut, 'Cut per pass in inches', group='setup')
z_safe = 0.1
params.addArgument(z_safe , 'Safe Z above surface in inches', group='setup')                

tool_dia = 0.25
params.addArgument(tool_dia, 'Tool diameter in inches', group='setup')    
x_length = 2.008
params.addArgument(x_length, 'Bar length along x', group='dim')
y_length = 1.125

operation_groove = True
params.addArgument(operation_groove, 'Cut groove', group='groove')                
z_groove = -0.063
#params.addArgument(z_groove, 'Groove z', group='groove') 
y_groove = 0.475
params.addArgument(y_groove, 'Groove y', group='groove') 

operation_left_end = True
params.addArgument(operation_left_end, 'Left end', group='ends')                
operation_right_end = True
params.addArgument(operation_right_end, 'Right end', group='ends')                

z_holes = -0.39
params.addArgument(z_holes, 'Hole depth along z', group='holes')


operation_back_cap_hole = True
params.addArgument(operation_back_cap_hole, 'Back: screw cap helical', group='back') 
z_cap = -0.23
params.addArgument(z_cap, 'helical depth along z', group='back') 


center_drill_only = False
params.addArgument(center_drill_only, 'Center drill only (no hole)', group='loadcell')

operation_load_cell_screw_hole = True
params.addArgument(operation_load_cell_screw_hole, 'Load cell screw hole', group='loadcell')

operation_dowel_pins = True
params.addArgument(operation_dowel_pins, 'Back: dowel pins hole', group='back')

operation_screw_mounts = True
params.addArgument(operation_screw_mounts, 'Back: Screw mounts hole', group='back')


# TODO: 
def y_slot(y0, y1, zsafe, z_depth, cuts):
    print "g0 Z%.4f (move tool out of the way)" % z_safe 
    
    for z in cuts:
        print "g0 y%.4f " % y0
        print "g1 z%.4f " % z
        print "g1 y%.4f" % y1 

    print "g0 Z%.4f (move tool out of the way)" % z_safe   

def hole(x, y, zsafe, rapid_plane, peck, z, feed ):
    print "g0 Z%.4f (move tool out of the way)" % z_safe
    print "g0 x%.4f y%.4f" % (x,y)
    print "G4 p0.1 (0.1 sec Pause)"
    #print "M0 (Pause)"
       
    print "G83 x%.4f y%.4f z%.4f r%.4f q%.4f" %(x,y, z, rapid_plane, peck)
    print "g0 Z%.4f (move tool out of the way)" % z_safe
    print "G4 p0.1 (0.1 sec Pause)"

# Show the GUI, wait for the user to press the OK button
if params.loadParams():  # the result is False if the window is closed without pressing OK
    
    print
    print 

    # generate GCODE here!
    hugomatic.code.header("Inches", feed)   
    
    

    if operation_groove:
        print "(Groove)"
        print "g0 Z%.4f (move tool out of the way)" % z_safe 
        x0 = 0. - tool_dia * 0.5
        x1 = x_length + tool_dia * 0.5
        y0 = -0.5 * y_groove
        y1 = 0.5 * y_groove
        tool = tool_dia
        zsurf = 0.
        cuts = hugomatic.code.z_cut_compiler(z_groove, cut); # an array of depths
        hugomatic.code.pocketRectangle(x0, y0, x1, y1, z_safe, zsurf,  tool, cuts)
    
    if operation_left_end:
        print "g0 Z%.4f (move tool out of the way)" % z_safe         
        print "(left end)"        
        x = - 0.5 * tool_dia
        print "g0 x%.4f" % x
        z_depth = -0.39
        y0 = -0.5 * y_length
        y1 = 0.5 * y_length 
        cuts = hugomatic.code.z_cut_compiler(z_depth, cut)
        y_slot(y0, y1, z_safe, z_depth, cuts)
        
    if operation_right_end:
        print "g0 Z%.4f (move tool out of the way)" % z_safe 
        print "(right end)"             
        x =  0.5 * tool_dia + x_length 
        print "g0 x%.4f" % x
        z_depth = -0.39
        y0 = -0.5 * y_length
        y1 = 0.5 * y_length 
        cuts = hugomatic.code.z_cut_compiler(z_depth, cut)
        y_slot(y0, y1, z_safe, z_depth, cuts)                           

    if operation_load_cell_screw_hole:
        print "(load_cell_screw_hole)"
        print "g0 Z%.4f (move tool out of the way)" % z_safe
        x = x_length - 1.0
        y = 0.
        z = z_holes
        if center_drill_only:
            z = -0.05
        hole(x, y, z_safe, 0.025, 0.266 , z, feed)


    if operation_back_cap_hole:
        print "(back screw cap hole)"
        
        x = x_length - 1.0
        y = 0.
        print "g0 x%.4f y%.4f Z%.4f (move tool out of the way)" % (x, y, z_safe)
        dia = 0.4
        print "g0 z%.4f" % 0.05
        z = z_cap
        hugomatic.code.HelicalHole(x, y, dia,  z, z_safe, cut, tool_dia)
    
    if operation_dowel_pins:
        print "(dowel_pins)"
        print "g0 Z%.4f (move tool out of the way)" % z_safe
        x = x_length - 0.5
        ytop = 0.4
        ybottom = -0.4  
        z = -0.300
        if center_drill_only:
            z = -0.05
        # zsafe, rapid_plane, peck, z, feed 
        hole(x, ytop, z_safe, 0.025, 0.125, z, feed)
        hole(x, ybottom, z_safe, 0.025, 0.125, z, feed)

    if operation_screw_mounts:
        print "(screw_mounts)"
        print "g0 Z%.4f (move tool out of the way)" % z_safe
        x = x_length - 1.0
        ytop = 0.4
        ybottom = -0.4
        z = z_holes
        if center_drill_only:
            z = -0.05  
        hole(x, ytop, z_safe, 0.025, 0.169, z, feed)
        hole(x, ybottom, z_safe, 0.025, 0.169, z, feed)

    print "g0 Z%.4f (move tool out of the way)" % z_safe 
    hugomatic.code.footer()
