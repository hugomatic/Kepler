
class Pid(object):
    """ 
    wikipedia pseudo code
    previous_error = 0
    integral = 0 
    start:
      error = setpoint - actual_position
      integral = integral + (error*dt)
      derivative = (error - previous_error)/dt
      output = (Kp*error) + (Ki*integral) + (Kd*derivative)
      previous_error = error
      wait(dt)
      goto start 
    """
    def __init__(self, kp, ki, kd):
       self.kp = kp
       self.ki = ki
       self.kd = kd
       self.previous_error = 0.
       self.integral = 0.
       self.op = 0
       self.oi = 0
       self.od = 0
       self.error = 0
       self.derivative = 0
        
    def eval(self, actual_position, set_point, delta_time):       
       error = set_point - actual_position
       e = error * delta_time
       self.integral += e
       derivative = (error - self.previous_error)/delta_time
       self.op = (self.kp * error)
       self.oi = (self.ki * self.integral)
       self.od = (self.kd * derivative)
       output = self.op + self.oi + self.od
       self.previous_error = error
       self.error = error
       self.derivative = derivative
       return output
   
   
   
    
class PidWeird(object):

    def __init__(self, kc, ki, kd) :
        self.kc = kc
        self.ki = ki
        self.kd = kd
         
        self._last_reading = 0.
        self._int_error = 0.
        self._prop_error = 0.
        self._der_error = 0.
        
    def _adjust_integral_error(self, pv, prop_error, delta_time):
        self._int_error += prop_error / delta_time
    
    def _adjust_derivative_error(self, pv, delta_time):    
        self._der_error = (self._last_reading - pv) / delta_time
        self._last_reading = pv 
            
    def eval(self, pv, enc, set_point, delta_time):

        self._prop_error = set_point - pv
        self._adjust_integral_error(pv, self._prop_error, delta_time)
        self._adjust_derivative_error(pv, delta_time)
        
        co = self.kc * (self._prop_error + (self.ki * self._int_error) - (self.kd * self._der_error))
        
        
            
        return co

    
if __name__ == "__main__":
    pid3 = Pid(0.5, 0.4, -100.)
    
    for i in range(1000):
        r = pid3.loop(1.,2.,800)
        print r
