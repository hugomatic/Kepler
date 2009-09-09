# Kalman filter example demo in Python

# A Python implementation of the example given in pages 11-15 of "An
# Introduction to the Kalman Filter" by Greg Welch and Gary Bishop,
# University of North Carolina at Chapel Hill, Department of Computer
# Science, TR 95-041,
# http://www.cs.unc.edu/~welch/kalman/kalmanIntro.html








import math 


class Filter(object):    
    def __init__(self, estimated_variance, process_variance): 
        self.R = estimated_variance                             
        self.Q = process_variance
        self.previous_result = 0.
        self.previous_error_estimate = 0.
    
    def filter(self, observation):
        priori_error_estimate = self.previous_error_estimate + self.Q
        blending_factor = priori_error_estimate/(priori_error_estimate+self.R )
        error_estimate = (1-blending_factor)* priori_error_estimate    
        result  = self.previous_result + blending_factor *( observation - self.previous_result)
        self.previous_result = result
        self.previous_error_estimate = error_estimate
        return result   

    def reset(self):
        self.previous_result = 0.
        self.previous_error_estimate = 0.
        

if __name__ == "__main__":
    import pylab
    import numpy
    
    R = 0.01 #0.1**2 # estimate of measurement variance, change to see effect
    Q = 1e-5 # process variance
    calman = Filter(R, Q)

    n_iter = 1600
    x = 0. # -0.37727  # truth value (typo in example at top of p. 13 calls this z)
    z = numpy.random.normal(x, 0.1, size=(n_iter)) # observations (normal about x, sigma=0.1)


    for i in range(1,n_iter):
        rad = math.radians(i)
        z[i] += math.sin(rad)


    xhat = []
    for k in range(1,n_iter):
        o = z[k]
        r = calman.filter(o)
        xhat.append(0.)
        if k % 50 == 0:
            print "reset"
            xhat.append(r)
            calman.reset()




    pylab.figure()
    pylab.plot(z,'k+',label='noisy measurements')
    pylab.plot(xhat,'b-',label='a posteri estimate')
    #print xhat
    
    pylab.axhline(x,color='g',label='truth value')
    pylab.legend()
    pylab.xlabel('Iteration')
    pylab.ylabel('Voltage')
    
    pylab.show()




