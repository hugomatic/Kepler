from fuzzyFunctions import *
from fuzzySignal import *

class fuzzy:
	'A fuzzy system'
	
	
	
	def __init__(self,name):
		self._name = name
		self.evaluation = None
		self._sigs_in = {}
		self._rules = []
		self._sig_out = None
		
	def add_input_signal(self,signal):	
		self._sigs_in[signal._name]=signal
		
	def set_output_signal(self,signal):	
		self._sig_out=signal		
		
	def addRule(self,rule):
		self._rules.append(rule)
		
	def evaluate_rules(self, sigs_in):
		memberships = {}
			
		for sig in sigs_in.keys():
			memberships[sig] = self._sigs_in[sig].evaluate(sigs_in[sig])

		vret = {}	
		for key in  self._sig_out._regions.keys():
			vret[key] = 0.0
			
		for rule in self._rules:
			aa =  rule.evaluate(memberships)
			#if aa[1] > 0.0:
			#	print "rule %s > %s = %.4f" % (rule._ireg, rule._oreg, aa[1]) 
			vret[aa[0]] += aa[1]
		self.evaluation = vret
		return vret
	
	def defuzz(self, vret):
		#print "Signal: %s" % self._sig_out._name
		result = self._sig_out.defuzz(vret)
		#print "unfuzzyfied result %f" %result
		return result

	def evaluate(self, sigs_in):
		txt = 'Fuzzy %s : ' % self._name
		#membership = {}
		#for sig in sigs_in.keys():
			#txt += "[%s] %.4f " % (sig, sigs_in[sig] )
			#membership[sig] = self._sigs_in[sig].evaluate(sigs_in[sig])
		#print membership
			
		rule_res = self.evaluate_rules(sigs_in)
		result = self.defuzz(rule_res)
		#txt += " = %.4f" % result
		#print txt
		return result
			
class rule:
	'A fuzzy rule'
	
	def __init__(self,ireg,oreg):
		self._ireg = ireg
		self._oreg = oreg
		self.evaluation = 0.		
	
	def evaluate(self,mvals):
		val = 1.0
		
		for rkey in self._ireg.keys():
			mval = mvals[rkey]
			rval = self._ireg[rkey]
			val = val* mval[rval]
		self.evaluation = val
		return [self._oreg,val]
		