from fuzz.fuzzy import *

def _create_signal(name, scale):
    print "create signal: %s, scale %.4f" % (name, scale)
    
    max = 1.0 * scale
    half =  0.5 * scale
    quart = 0.25 * scale
    # bigger than max to make
    # sure we get values for max and -max
    limit = max + quart 
    
    p = fuzzySignal(name)
    p.addRegion('LN',trapmf([-limit,-max,-half,-quart]))
    p.addRegion('SN',trimf([-half,-quart,0]))
    p.addRegion('Z',trimf([-quart,0,quart]))
    p.addRegion('SP',trimf([0,quart,half]))
    p.addRegion('LP',trapmf([quart,half,max,limit]))
    return p

def get_angle_controller(angle_error_min, 
                         angle_error_max, 
                         rot_speed_error_min, 
                         rot_speed_error_max,
                         accel_min, 
                         accel_max):
    
    escale = max(abs(angle_error_min), abs(angle_error_max))
    dscale = max(abs(rot_speed_error_min), abs(rot_speed_error_max))
    oscale = max(abs(accel_min), abs(accel_max))
    
    p = _create_signal('err', escale) # position error
    s = _create_signal('speederr', dscale)
    a = _create_signal('out', oscale)
    
    fuzz = fuzzy('Dupa')
    fuzz.add_input_signal(p)
    fuzz.add_input_signal(s)
    fuzz.set_output_signal(a)
    
    fuzz.addRule(rule({'err':'LN','speederr':'SN'},'LP'))
    fuzz.addRule(rule({'err':'LN','speederr':'Z'} ,'SP'))
    fuzz.addRule(rule({'err':'LN','speederr':'SP'},'SP'))

    fuzz.addRule(rule({'err':'LP','speederr':'SN'},'SN'))
    fuzz.addRule(rule({'err':'LP','speederr':'Z'} ,'SN'))
    fuzz.addRule(rule({'err':'LP','speederr':'SP'},'SN'))
    
    fuzz.addRule(rule({'err':'SN','speederr':'SN'},'Z'))
    fuzz.addRule(rule({'err':'SN','speederr':'Z'},'SP'))
    fuzz.addRule(rule({'err':'SN','speederr':'SP'},'Z'))
    
    fuzz.addRule(rule({'err':'Z','speederr':'SN'},'SP'))
    fuzz.addRule(rule({'err':'Z','speederr':'Z'}, 'Z'))
    fuzz.addRule(rule({'err':'Z','speederr':'SP'},'SN'))
    
    fuzz.addRule(rule({'err':'SP','speederr':'SN'},'Z'))
    fuzz.addRule(rule({'err':'SP','speederr':'Z'},'SN'))
    fuzz.addRule(rule({'err':'SP','speederr':'SP'},'Z'))
    return fuzz

if __name__ == "__main__":
    
    angle_error_min = -0.0035540
    angle_error_max = 0.0035540 
    rot_speed_error_min =  -0.005
    rot_speed_error_max = 0.005
    accel_min = -1.
    accel_max = 1.
    
    fuzz = get_angle_controller(angle_error_min, angle_error_max, rot_speed_error_min, rot_speed_error_max, accel_min, accel_max)
    
    error_input = 0.00355
    speed_error_input = 0.
    
    
    
    print
    print
    print "err %f, speed err %f" % (error_input, speed_error_input)
    eval = fuzz.evaluate({'err':error_input,'speederr':speed_error_input})
    
    for name, signal in fuzz._sigs_in.iteritems():
        print name, signal.evaluation.values()
    
    rule_activations = []
    for rule in fuzz._rules:
        rule_activations.append(rule.evaluation)
    print "rules %s" % rule_activations
    print "Defuzz = %f" % eval
