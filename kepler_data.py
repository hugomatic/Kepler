import logging

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

def get_data_logger():
    return logger

class KeplerLogger(object):
    
    def __init__(self, memory):
        self.data = []
        self.memory = memory
    
    def dump(self, logger):
        for msg in self.data:
            logger.debug(msg)
        
    def snapshot(self, logger, step, sim_time, data, exclude_list = ()):
        s = "SIM_STEP %s SIM_TIME %s" % (step, sim_time) 
        if self.memory:
            self.data.append(s)
        else:
            logger.debug(s )
        for k,value in data.iteritems():
            if k not in exclude_list:
                s = "%s = %s " %  (k, value[0])
            if self.memory:
                self.data.append(s)
            else:
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
                if not val.startswith('['):
                    val = float(val)
                if not self.history.has_key(key):
                    self.history[key] = []    
                self.history[key].append(val)

class DataLogger(object):
    
    def __init__(self): 
        self.exlude_list =["exlude_list", "history", 'types']
        #self.history_step_count = history_steps
        self.history = {}
        self.types = {}
        for k in self.__dict__.keys():
             if k not in self.exlude_list:
                 self.history[k]=[]
                 self.types[k] = type(self.__dict__[k])

    def snapshot(self, logger=None):           
        keys = self.history.keys()
        for k in keys:
            value = self.__dict__[k]
            self.history[k].append(value)
            if logger:
                s = k + '=' + str(value)
                logger.debug(s)
        if logger:
            logger.debug('SNAPSHOT')
         
    
    def load_from_file(self, file_name):
        def str_to_type(s, typ):
            if typ == type(42):
                return int(s)
            if typ == type(3.14159):
                return float(s)
            return s
        lines = []
        logfile = open(file_name,'r')
        try:
            lines = logfile.readlines()
        finally:
            logfile.close()
            
        for line in lines:
            line = line.strip() # remove \n
            key_eq_val = line.split(':')[-1]
            if key_eq_val == 'SNAPSHOT':
                # write the current data values to history
                self.snapshot()
            else:
                # set one of the data values
                key,val = key_eq_val.split('=')
                self.__dict__[key] = str_to_type(val, self.types[key])
        
        
        # read next line
        # fill data
        # when done, take snapshot
    


class ControlData(DataLogger):
    
    def __init__(self):  
          
        self.actuator1_adc = 0.
        self.actuator2_adc = 0.
        self.actuator1 = 0. # SI
        self.actuator2 = 0. # SI
        
        # desired or 'controlled' positions
        # angles
        self.table_angle1_control = 0.
        self.table_angle2_control = 0.
        # actuator length
        self.actuator1_control = 0. # SI
        self.actuator2_control = 0. # SI
        # actuator pwm
        self.actuator1_servo_control = 0.
        self.actuator2_servo_control = 0.
        
        # measured position
        self.table_angle1 = 0.
        self.table_angle2 = 0.
        self.table_rot_speed1 = 0.
        self.table_rot_speed2 = 0.
        self.table_rot_accel1 = 0.
        self.table_rot_accel2 = 0.
        self.load_cell_x_adc = 0.
        self.load_cell_y_ = 0.
        
        self.ball_pos_x = 0.
        self.ball_pos_y = 0.
        self.ball_speed_x = 0.
        self.ball_speed_y = 0.
        self.ball_accel_x = 0.
        self.ball_accel_y = 0.
        
        DataLogger.__init__(self)
        
                

     

if __name__ == '__main__':

    import unittest

    class TestLog(unittest.TestCase):
        def setUp(self):
            pass
            
        def test_snapshot(self):
            data = ControlData()
            data.actuator1 = 2.
            data.snapshot(logger)
            data.actuator1 += 2.
            data.snapshot(logger)
            data.actuator1 += 2.
            data.snapshot(logger)
            data.actuator1 += 2.
            data.snapshot(logger) 
            old_value = data.history['actuator1'][-2]
            self.assertTrue(old_value < data.actuator1)
            # self.assertRaises(E data.historic('actuator1', 2)
            data2 = ControlData()
            siz = len(data2.history['actuator1'])
            self.assertTrue(siz == 0)
            data2.load_from_file('kepler.log')
            siz = len(data2.history['actuator1'])
            self.assertTrue(siz > 0)
            print data2.history['actuator1']
    

    unittest.main()

    print 'hello'
     
      