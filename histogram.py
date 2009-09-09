#!/usr/bin/env python
# -*- noplot -*-
import time
from pylab import *

def get_memory(t):
    "Simulate a function that returns system memory"
    return 100*(0.5+0.5*sin(0.5*pi*t))

def get_cpu(t):
    "Simulate a function that returns cpu usage"
    return 100*(0.5+0.5*sin(0.2*pi*(t-0.25)))


# turn interactive mode on for dynamic updates.  If you aren't in
# interactive mode, you'll need to use a GUI event handler/timer.
ion()

t1 = arange(0.0, 5.0, 0.1)
t2 = arange(0.0, 5.0, 0.02)

s1 = subplot(211)
l = plot(t1, get_memory(t1), 'bo', t2, get_cpu(t2), 'k--', markerfacecolor='green')
grid(True)
title('A tale of 2 subplots')
ylabel('Damped oscillation')

s2 = subplot(212)
plot(t2, cos(2*pi*t2), 'r.')
grid(True)
xlabel('time (s)')
ylabel('Undamped')

import time
while 1:
   time.sleep(0.1)
   for i in range(len(t1)):
     t1[i] += 0.01
   s1.plot(t1, get_memory(t1), 'bo', t2, get_cpu(t2), 'k--', markerfacecolor='green')
   draw()



