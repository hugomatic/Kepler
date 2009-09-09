from fuzzyFunctions import *

class fuzzySignal(object):
    """A fuzzy input/ output signal
    """
	
    def __init__(self, name):
        self.evaluation = None
        self._name = name
        self._regions = {}
		
    def addRegion(self,name,mf):
		'Adds a new fuzzy region with given name and membership function'
		self._regions[name]=mf
		print " Region %s %s" % (name, mf._p)
		
    def evaluate(self,x):
		regs = self._regions.keys()
		ret = {}
		for key in regs:
			mf = self._regions[key]
			ret[key] = mf.evaluate(x)
		self.evaluation = ret	
		return ret 
		
    def defuzz(self, results):
		data = []
		for region, value in results.iteritems():
			mf = self._regions[region]
			if value > 0.0:
				x, area =  mf.defuzz(value)
				data.append( (x,area) )
		
		# center of mass of composite shape
		accumumlator = 0
		total_area = 0
		for centroid_x, area in data:
			accumumlator += centroid_x * area
			total_area += area
		if total_area == 0:
			return 0
		
		centroid = accumumlator / total_area
		# http://en.wikipedia.org/wiki/Center_of_mass
		# http://en.wikipedia.org/wiki/List_of_centroids
		return centroid