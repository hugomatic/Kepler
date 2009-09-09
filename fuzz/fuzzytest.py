from fuzzy import *

aa = fuzzySignal('x1')

aa.addRegion('S',trimf([-2,-1,0]))
aa.addRegion('C',trimf([-1,0,1]))
aa.addRegion('B',trimf([0,1,2]))


bb = fuzzySignal('x2')

bb.addRegion('S',trimf([-2,-1,0]))
bb.addRegion('C',trimf([-1,0,1]))
bb.addRegion('B',trimf([0,1,2]))

cc = fuzzySignal('y1')

cc.addRegion('S',trimf([-2,-1,0]))
cc.addRegion('C',trimf([-1,0,1]))
cc.addRegion('B',trimf([0,1,2]))

ff = fuzzy('Dupa')

ff.addInSignal(aa)
ff.addInSignal(bb)
ff.addOutSignal(cc)

ff.addRule(rule({'x1':'S','x2':'S'},'B'))
ff.addRule(rule({'x1':'S','x2':'C'},'B'))
ff.addRule(rule({'x1':'S','x2':'B'},'C'))

ff.addRule(rule({'x1':'C','x2':'S'},'B'))
ff.addRule(rule({'x1':'C','x2':'C'},'C'))
ff.addRule(rule({'x1':'C','x2':'B'},'S'))

ff.addRule(rule({'x1':'B','x2':'S'},'C'))
ff.addRule(rule({'x1':'B','x2':'C'},'S'))
ff.addRule(rule({'x1':'B','x2':'B'},'S'))

ff.evaluate({'x1':0,'x2':0.2})

