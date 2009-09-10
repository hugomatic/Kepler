#!/usr/bin/python

import time
import math

import direct.directbase.DirectStart

#from pandac.PandaModules import lookAt
from pandac.PandaModules import GeomVertexReader
from pandac.PandaModules import Plane, Point3
from pandac.PandaModules import GeomVertexFormat, GeomVertexData
from pandac.PandaModules import Geom, GeomTriangles, GeomVertexWriter
from pandac.PandaModules import Texture, GeomNode

from pandac.PandaModules import CollisionTraverser,CollisionNode
from pandac.PandaModules import CollisionHandlerQueue,CollisionRay, CollisionPlane
from pandac.PandaModules import Material,LRotationf,NodePath
from pandac.PandaModules import AmbientLight,DirectionalLight
from pandac.PandaModules import LightAttrib,TextNode
from pandac.PandaModules import Vec3,Vec4,BitMask32
from direct.gui.OnscreenText import OnscreenText
from direct.showbase.DirectObject import DirectObject
from direct.interval.MetaInterval import Sequence,Parallel
from direct.interval.LerpInterval import LerpFunc
from direct.interval.FunctionInterval import Func,Wait
from direct.task.Task import Task

from direct.gui.DirectGui import *

import sys
from kepler_data import *



#Some constants for the program
ACCEL = 70         #Acceleration in ft/sec/sec
MAX_SPEED = 55     #Max speed in ft/sec
MAX_SPEED_SQ = MAX_SPEED ** 2  #Squared to make it easier to use lengthSquared
                               #Instead of length
UP = Vec3(0,0,1)   #We need this vector a lot, so its better to just have one
                   #instead of creating a new one every time we need it
font = loader.loadFont("cmss12")

MOTION_ANGLE_DELTA = 0.05

VISIBLE_WALLS = False
DISABLE_MOUSE = False

table_length = 5
xmin = -table_length
xmax = table_length
ymin = -table_length
ymax = table_length

def processVertexData(vdata):
  vertex = GeomVertexReader(vdata, 'vertex')
  texcoord = GeomVertexReader(vdata, 'texcoord')
  while not vertex.isAtEnd():
    v = vertex.getData3f()
    t = texcoord.getData2f()
    print "v = %s, t = %s" % (repr(v), repr(t))

def processPrimitive(prim, vdata):
  vertex = GeomVertexReader(vdata, 'vertex')
  prim = prim.decompose()
  for p in range(prim.getNumPrimitives()):
    s = prim.getPrimitiveStart(p)
    e = prim.getPrimitiveEnd(p)
    for i in range(s, e):
      vi = prim.getVertex(i)
      vertex.setRow(vi)
      v = vertex.getData3f()
      print "prim %s has vertex %s: %s" % (p, vi, repr(v))
    print

def processCollisionSolid(solid):
    print solid

def processCollisionNode(cnode):
    print "Collision Node:", cnode.getName()
    num = cnode.getNumSolids()
    print "solids:", num
    for i in range(num):
        solid = cnode.getSolid(i)
        print "solid ",str(i+1),"/",str(num)
        processCollisionSolid(solid)
        
def processGeom(geom):
  vdata = geom.getVertexData()
  print vdata
  processVertexData(vdata)
  for i in range(geom.getNumPrimitives()):
    prim = geom.getPrimitive(i)
    print prim
    processPrimitive(prim, vdata)

def processGeomNode(geomNode):
  for i in range(geomNode.getNumGeoms()):
    geom = geomNode.getGeom(i)
    state = geomNode.getGeomState(i)
    print geom
    print state
    processGeom(geom) 

def print_tree(node, i=0):
    innernode = node.node()
    print i*" ", node, " type: ", type(innernode)
    if isinstance(innernode, CollisionNode):
        processCollisionNode(innernode)
    if isinstance(innernode, GeomNode):
        processGeomNode(innernode)   
    for child in node.getChildren():
        print_tree( child, i+1)
 
def create_table_geom():
    format = GeomVertexFormat.getV3n3c4t2()
    # GeomVertexData. 
    vdata = GeomVertexData('table_vertex', format, Geom.UHStatic)
    
    vertex = GeomVertexWriter(vdata, 'vertex')
    normal = GeomVertexWriter(vdata, 'normal')
    color = GeomVertexWriter(vdata, 'color')
    texcoord = GeomVertexWriter(vdata, 'texcoord')

    vertex.addData3f(xmax, ymin, 0)
    normal.addData3f(0, 0, 1)
    color.addData4f(0, 0, 1, 1)
    texcoord.addData2f(1, 0)
    
    vertex.addData3f(xmax, ymax, 0)
    normal.addData3f(0, 0, 1)
    color.addData4f(0, 0, 1, 1)
    texcoord.addData2f(1, 1)
    
    vertex.addData3f(xmin, ymax, 0)
    normal.addData3f(0, 0, 1)
    color.addData4f(0, 0, 1, 1)
    texcoord.addData2f(0, 1)
    
    vertex.addData3f(xmin, ymin, 0)
    normal.addData3f(0, 0, 1)
    color.addData4f(0, 0, 1, 1)
    texcoord.addData2f(0, 0)
    
    prim = GeomTriangles(Geom.UHStatic)
    prim.addVertex(0)
    prim.addVertex(1)
    prim.addVertex(2)
    prim.closePrimitive()
    
    prim.addVertex(0)
    prim.addVertex(2)
    prim.addVertex(3)
    prim.closePrimitive()

    geom = Geom(vdata)
    geom.addPrimitive(prim)
    return geom

def process_wall_collide_geom(geom):
    print 'process wall geom'
    print geom

def add_collision_nodes(model):
    wall_collision_node_path = model.getChildren()[0].getChildren()[0]
    cnode = wall_collision_node_path.node() # get collision node

    num = cnode.getNumSolids()
    for i in range(num):
        cnode.removeSolid(0)
    
    # south_wall_cnode = CollisionNode("south_w")
    plane = Plane(Vec3(0, 1, 0), Point3(0,ymin,0) ) 
    south_wall = CollisionPlane( plane )     
    cnode.addSolid(south_wall)
    
    # north
    plane = Plane(Vec3(0, -1, 0), Point3(0,ymax,0))
    cnode.addSolid(CollisionPlane( plane ))   
    # east
    plane = Plane(Vec3(-1, 0, 0), Point3(xmax,0,0))
    cnode.addSolid(CollisionPlane( plane ))    
    # west
    plane = Plane(Vec3(1, 0, 0), Point3(xmin,0,0))
    cnode.addSolid(CollisionPlane( plane ))
    
def process_model(model):
    #print 'before:'
    #print_tree(model)
    maze = model.getChildren()[0].getChildren()[8]
    deleted_walls = []
    
    for child in model.getChildren()[0].getChildren():
        name = child.getName()
        if (name.startswith('hole_collide') ):
            child.removeNode()
        if (name.startswith('wall_collide') ):
            process_wall_collide_geom(child.node())
    
    geom_maze = maze.node()
    geom_maze.removeGeom(1)
    geom_maze.removeGeom(0)
    
    geom_maze.addGeom(create_table_geom())  
    add_collision_nodes(model)
    
    
class WorldBase(DirectObject):
    """
    Basic 3D world with logging and 2D display
    """
    def __init__(self, controller, mem_logger, display_categories):
        
        self.display_categories = display_categories
        
        self.labels = OnscreenText( "", style = 1, fg = ( 1, 1, 1, 1 ), pos = ( -1.0, 0.9 ), scale = .05 )
        self.txt = OnscreenText( "", style = 1, fg = ( 1, 1, 1, 1 ), pos = ( -0.5, 0.9 ), scale = .05 )

        taskMgr.add( self.loop, "loop" )

        self.show_details = True
        self.log = KeplerLogger(mem_logger)        
        self.logging = False
        self.accept("p", self.key_p)
        self.accept( "escape", self.quit )
        self.accept("arrow_up", self.key_up)
        self.accept("arrow_down", self.key_down)
        self.accept("arrow_left", self.key_left)
        self.accept("arrow_right", self.key_right)
        self.accept("l", self.key_l)
        self.accept("s", self.key_s)
        self.accept("a", self.key_a)
        self.accept("f", self.key_f)
        self.accept("g", self.key_g)
        self.accept("d", self.key_d)
        self.accept("o", self.log_start_stop)
        self.accept("q", self.key_q)
        self.accept("w", self.key_w)
        self.accept("z", self.key_z)
        self.accept("x", self.key_x)
        self.accept("c", self.key_c)
        self.accept("v", self.key_v)
        self.accept("b", self.key_b)
        self.accept("n", self.key_n)
            
        self.controller = controller
        base.setFrameRateMeter(True)

        
        #self.txt = OnscreenText( "Nothing", style = 1, fg = ( 1, 1, 1, 1 ), shadow = ( .9, .9, .9, .5 ), pos = ( -1.0, 0.9 ), scale = .07 )
        # task to be called every frame
        
        self.step = 0
        self.last_time = time.time()
        
        self.text_refresh_band = 10
        self.text_refresh_count = self.text_refresh_band
        self._set_title("Hugomatic")

    def _set_title(self, title):
      from pandac.PandaModules import ConfigVariableString
      mygameserver = ConfigVariableString("window-title","Panda")
      mygameserver.setValue(title)

    def loop(self, task):
        #self.last_time = time.time()
        delta_time = task.time - self.last_time
        # avoid division by 0
        if delta_time < 0.0001:
            delta_time = 0.0001
        self.last_time = task.time
        self.step += 1
        txt = ""
        self._refresh_text(txt)
        self.controller.loop(self.step, task.time, delta_time)
        
        if self.logging:
            data = self.controller.get_display_data()
            self.log.snapshot(get_data_logger(), self.step, task.time, data, ('self'))        
        return Task.cont

    def _refresh_text(self, txt):
        if not self.show_details:
            return 
        self.text_refresh_count -=1
        if self.text_refresh_count <0:
            self.text_refresh_count = self.text_refresh_band
            data = self.controller.get_display_data()
            st = str(self.last_time)
            data['Time'] = (st, "sim", 0)
            data['step'] = (self.step, "sim", 0)
            
            self.txt.clearText()
            self.txt.setAlign(TextNode.ALeft)

            keys = data.keys()
            keys.sort()
            txt += "logging: %s\n" % self.logging
            
            for name in keys:
                if name == 'self': continue
                display_data = data[name]                
                value, category,axis = display_data   
                show_it = self.display_categories[category]
                if axis == 1 and self.display_categories['axis1']:
                   show_it = False
                if axis == 2 and self.display_categories['axis2']:
                   show_it = False
                if show_it:                   
                   txt += "%s: %s\n" % (name, value)
            self.txt.appendText(txt)
 
    def call_back(self):
        print "hurray"
    
    def add_check(self, text, value, x, y, callback):
       b = DirectCheckButton(text = text ,scale=.05, pos=(x,0,y), command=callback)
       b["indicatorValue"] = value
       return b
       
    def add_slider(self):
        #slider = DirectSlider(range=(0,100), value=50, pageSize=1, command=self.call_back)
        
        left = 0.1
        right = 0.2
        bottom = 0.1
        top = 0.11
        frame = (left, right, bottom, top)
        
        b = DirectButton(text = ("OK", "click!", "rolling over", "disabled"), scale=.05, pos=(-.3,.6,0), command=self.call_back)
        b = DirectButton(text = ("ALLO", "click!", "rolling over", "disabled"), scale=.05, pos=(.3,-0.6,-0.5), command=self.call_back)

        #b['frameSize'] = frame
        b.resetFrameSize()

    def quit(self):
        self.controller.stop()
        if len(self.log.data) >0:
            self.log.dump(get_data_logger())
        sys.exit()
        
    def log_start_stop(self):
        if self.logging:
            self.controller.stop()
            self.log.dump(get_data_logger())
            self.log.data = []
        
        self.logging = not self.logging
        print "Logging %s" % self.logging 

    def key_p(self):
      self.show_details = not self.show_details
      print "show_details=", self.show_details
        
    def key_right(self):  
        self.controller.key_right()
    def key_left(self): 
        self.controller.key_left()
    def key_l(self): 
        self.controller.key_l()
    def key_s(self): 
        self.controller.key_s()
    def key_a(self): 
        self.controller.key_a()
    def key_f(self): 
        self.controller.key_f()
    def key_g(self):    
        self.controller.key_g()
    def key_d(self): 
        self.controller.key_d()
    def key_up(self):
        self.controller.key_up()
    def key_down(self):    
        self.controller.key_down()
    def key_q(self):    
        self.controller.key_q()
    def key_w(self):    
        self.controller.key_w()   
    def key_z(self):    
        self.controller.key_z()    
    def key_x(self):    
        self.controller.key_x()    
    def key_c(self):    
        self.controller.key_c()    
    def key_v(self):    
        self.controller.key_v()    
    def key_b(self):    
        self.controller.key_b()   
    def key_n(self):    
        self.controller.key_n()    
   
class BallPlateWorld(WorldBase):
    
  def __init__(self, controller, log_2_memory, display_categories):    
    WorldBase.__init__(self, controller, log_2_memory, display_categories)
    self.last_time = time.time()
    self.step = 0
    self.__init_kepler_scene()
  
  def set_3d_scene(self, x,y,z, alpha, beta, dt):
    new_position = Point3(x,y,z)
    self.maze.setP(-self.alpha)
    self.maze.setR(-self.beta)
    self.ballRoot.setPos(new_position)

    #This block of code rotates the ball. It uses a quaternion
    #to rotate the ball around an arbitrary axis. That axis perpendicular to
    #the balls rotation, and the amount has to do with the size of the ball
    #This is multiplied on the previous rotation to incrimentally turn it.
    prevRot = LRotationf(self.ball.getQuat())
    axis = UP.cross(self.ballV)
    newRot = LRotationf(axis, 45.5 * dt * self.ballV.length())
    self.ball.setQuat(prevRot * newRot)

 
  def __init_kepler_scene(self):
    self.hud_count_down = 0
    self.alpha = 0.
    self.beta = 0.  
    self._set_title("Hugomatic 3D sim")
    self.alpha_rot_speed = 0.
    self.beta_rot_speed = 0.
    
    #This code puts the standard title and instruction text on screen
    self.title = OnscreenText(text="Kepler simulation tool 1",
                              style=1, fg=(1,1,1,1),
                              pos=(0.7,-0.95), scale = .07, font = font)
    
    self.instructions = OnscreenText(text="alpha: 0.000\nbeta: 0.000",
                                     pos = (-1.3, .95), fg=(1,1,1,1), font = font,
                                     align = TextNode.ALeft, scale = .05)
    
    if DISABLE_MOUSE:
        base.disableMouse()                    #Disable mouse-based camera control
    camera.setPosHpr(-10, -10, 25, 0, -90, 0)  #Place the camera

    #Load the maze and place it in the scene
    self.maze = loader.loadModel("models/maze")
    process_model(self.maze)
    self.maze.reparentTo(render)

    #Most times, you want collisions to be tested against invisible geometry
    #rather than every polygon. This is because testing against every polygon
    #in the scene is usually too slow. You can have simplified or approximate
    #geometry for the solids and still get good results.
    #
    #Sometimes you'll want to create and position your own collision solids in
    #code, but it's often easier to have them built automatically. This can be
    #done by adding special tags into an egg file. Check maze.egg and ball.egg
    #and look for lines starting with <Collide>. The part is brackets tells
    #Panda exactly what to do. Polyset means to use the polygons in that group
    #as solids, while Sphere tells panda to make a collision sphere around them
    #Keep means to keep the polygons in the group as visable geometry (good
    #for the ball, not for the triggers), and descend means to make sure that
    #the settings are applied to any subgroups.
    #
    #Once we have the collision tags in the models, we can get to them using
    #NodePath's find command

    #Find the collision node named wall_collide
    self.walls = self.maze.find("**/wall_collide")

    #Collision objects are sorted using BitMasks. BitMasks are ordinary numbers
    #with extra methods for working with them as binary bits. Every collision
    #solid has both a from mask and an into mask. Before Panda tests two
    #objects, it checks to make sure that the from and into collision masks
    #have at least one bit in common. That way things that shouldn't interact
    #won't. Normal model nodes have collision masks as well. By default they
    #are set to bit 20. If you want to collide against actual visible polygons,
    #set a from collide mask to include bit 20
    #
    #For this example, we will make everything we want the ball to collide with
    #include bit 0
    self.walls.node().setIntoCollideMask(BitMask32.bit(0))
    #CollisionNodes are usually invisible but can be shown. Uncomment the next
    #line to see the collision walls
    if VISIBLE_WALLS:
        self.walls.show()

    #Ground_collide is a single polygon on the same plane as the ground in the
    #maze. We will use a ray to collide with it so that we will know exactly
    #what height to put the ball at every frame. Since this is not something
    #that we want the ball itself to collide with, it has a different
    #bitmask.
    self.mazeGround = self.maze.find("**/ground_collide")
    self.mazeGround.node().setIntoCollideMask(BitMask32.bit(1))
    
    #Load the ball and attach it to the scene
    #It is on a root dummy node so that we can rotate the ball itself without
    #rotating the ray that will be attached to it
    self.ballRoot = render.attachNewNode("ballRoot")
    self.ball = loader.loadModel("models/ball")
    self.ball.reparentTo(self.ballRoot)

    #Find the collison sphere for the ball which was created in the egg file
    #Notice that it has a from collision mask of bit 0, and an into collison
    #mask of no bits. This means that the ball can only cause collisions, not
    #be collided into
    self.ballSphere = self.ball.find("**/ball")
    self.ballSphere.node().setFromCollideMask(BitMask32.bit(0))
    self.ballSphere.node().setIntoCollideMask(BitMask32.allOff())

    #No we create a ray to start above the ball and cast down. This is to
    #Determine the height the ball should be at and the angle the floor is
    #tilting. We could have used the sphere around the ball itself, but it
    #would not be as reliable
    self.ballGroundRay = CollisionRay()     #Create the ray
    self.ballGroundRay.setOrigin(0,0,10)    #Set its origin
    self.ballGroundRay.setDirection(0,0,-1) #And its direction
    #Collision solids go in CollisionNode
    self.ballGroundCol = CollisionNode('groundRay') #Create and name the node
    self.ballGroundCol.addSolid(self.ballGroundRay) #Add the ray
    self.ballGroundCol.setFromCollideMask(BitMask32.bit(1)) #Set its bitmasks
    self.ballGroundCol.setIntoCollideMask(BitMask32.allOff())
    #Attach the node to the ballRoot so that the ray is relative to the ball
    #(it will always be 10 feet over the ball and point down)
    self.ballGroundColNp = self.ballRoot.attachNewNode(self.ballGroundCol)
    #Uncomment this line to see the ray
    self.ballGroundColNp.show()

    #Finally, we create a CollisionTraverser. CollisionTraversers are what
    #do the job of calculating collisions
    self.cTrav = CollisionTraverser()
    #Collision traverservs tell collision handlers about collisions, and then
    #the handler decides what to do with the information. We are using a
    #CollisionHandlerQueue, which simply creates a list of all of the
    #collisions in a given pass. There are more sophisticated handlers like
    #one that sends events and another that tries to keep collided objects
    #apart, but the results are often better with a simple queue
    self.cHandler = CollisionHandlerQueue()
    #Now we add the collision nodes that can create a collision to the
    #traverser. The traverser will compare these to all others nodes in the
    #scene. There is a limit of 32 CollisionNodes per traverser
    #We add the collider, and the handler to use as a pair
    self.cTrav.addCollider(self.ballSphere, self.cHandler)
    self.cTrav.addCollider(self.ballGroundColNp, self.cHandler)

    #Collision traversers have a built in tool to help visualize collisions.
    #Uncomment the next line to see it.
    if VISIBLE_WALLS:
        self.cTrav.showCollisions(render)
    
    #This section deals with lighting for the ball. Only the ball was lit
    #because the maze has static lighting pregenerated by the modeler
    lAttrib = LightAttrib.makeAllOff()
    ambientLight = AmbientLight( "ambientLight" )
    ambientLight.setColor( Vec4(.55, .55, .55, 1) )
    lAttrib = lAttrib.addLight( ambientLight )
    directionalLight = DirectionalLight( "directionalLight" )
    directionalLight.setDirection( Vec3( 0, 0, -1 ) )
    directionalLight.setColor( Vec4( 0.375, 0.375, 0.375, 1 ) )
    directionalLight.setSpecularColor(Vec4(1,1,1,1))
    lAttrib = lAttrib.addLight( directionalLight )
    self.ballRoot.node().setAttrib( lAttrib )
    
    #This section deals with adding a specular highlight to the ball to make
    #it look shiny
    m = Material()
    m.setSpecular(Vec4(1,1,1,1))
    m.setShininess(96)
    self.ball.setMaterial(m, 1)

    #Finally, we call start for more initialization
    # self.start()
  
      
    #def start(self):
    #The maze model also has a locator in it for where to start the ball
    #To access it we use the find command
    startPos = (0,0,0)#= self.maze.find("**/start").getPos()
    self.ballRoot.setPos(startPos)   #Set the ball in the starting position
    self.ballV = Vec3(0,0,0)         #Initial velocity is 0
    self.accelV = Vec3(0,0,0)        #Initial acceleration is 0
    
    #For a traverser to actually do collisions, you need to call
    #traverser.traverse() on a part of the scene. Fortunatly, base has a
    #task that does this for the entire scene once a frame. This sets up our
    #traverser as the one to be called automatically
    base.cTrav = self.cTrav

  #This function handles the collision between the ray and the ground
  #Information about the interaction is passed in colEntry
  def groundCollideHandler(self, colEntry):
    #Set the ball to the appropriate Z value for it to be exactly on the ground
    newZ = colEntry.getSurfacePoint(render).getZ()
    self.ballRoot.setZ(newZ + .4)

    #Find the acceleration direction. First the surface normal is crossed with
    #the up vector to get a vector perpendicular to the slope
    norm = colEntry.getSurfaceNormal(render)
    accelSide = norm.cross(UP)
    #Then that vector is crossed with the surface normal to get a vector that
    #points down the slope. By getting the acceleration in 3D like this rather
    #than in 2D, we reduce the amount of error per-frame, reducing jitter
    self.accelV = norm.cross(accelSide)

  #This function handles the collision between the ball and a wall
  def wallCollideHandler(self, colEntry):
    #First we calculate some numbers we need to do a reflection
    norm = colEntry.getSurfaceNormal(render) * -1 #The normal of the wall
    curSpeed = self.ballV.length()                #The current speed
    inVec = self.ballV / curSpeed                 #The direction of travel
    velAngle = norm.dot(inVec)                    #Angle of incidance
    hitDir = colEntry.getSurfacePoint(render) - self.ballRoot.getPos()
    hitDir.normalize()                            
    hitAngle = norm.dot(hitDir)   #The angle between the ball and the normal

    #Ignore the collision if the ball is either moving away from the wall
    #already (so that we don't accidentally send it back into the wall)
    #and ignore it if the collision isn't dead-on (to avoid getting caught on
    #corners)
    if velAngle > 0 and hitAngle > .995:
      #Standard reflection equation
      reflectVec = (norm * norm.dot(inVec * -1) * 2) + inVec
        
      #This makes the velocity half of what it was if the hit was dead-on
      #and nearly exactly what it was if this is a glancing blow
      self.ballV = reflectVec * (curSpeed * (((1-velAngle)*.5)+.5))
      #Since we have a collision, the ball is already a little bit buried in
      #the wall. This calculates a vector needed to move it so that it is
      #exactly touching the wall
      disp = (colEntry.getSurfacePoint(render) -
              colEntry.getInteriorPoint(render))
      newPos = self.ballRoot.getPos() + disp
      self.ballRoot.setPos(newPos)

  def update_hud(self):
    self.hud_count_down -= 1
    if self.hud_count_down <= 0:
        self.hud_count_down = 5
        p1, p2 = self.get_ball_position()
        text =  "\nalpha: %.5f\nbeta: %.5f\npos [%.05f,%.05f]\n" % (self.alpha, self.beta, p1,p2)
        self.instructions.setText(text) 
    
  def control_task(self,task):
        delta_time = task.time - self.last_time
        self.last_time = task.time
        self.step += 1
        if self.controller:
            self.controller.loop(self.step, task.time, delta_time)     

        if self.logging:
            data = self.controller.get_display_data()
            self.log.snapshot(get_data_logger(), self.step, task.time, data, ('self'))        
 
  def get_table_inclination(self):
      angle1 = math.radians(self.alpha)
      angle2 = math.radians(self.beta)
      return (angle1, angle2)
  
  def set_table_rotation_speed(self, alpha_rot_speed, beta_rot_speed):
      self.alpha_rot_speed = alpha_rot_speed
      self.beta_rot_speed = beta_rot_speed
  
  def get_ball_position(self):
      p = self.ballRoot.getPos()
      p1 = p[0]
      p2 = p[1]
      #print "ball position [%s, %s]" % (p1, p2)
      return (p1, p2)
    
    
class BallPlateSimulatorWorld(BallPlateWorld):
  
  def __init__(self, controller, log_2_memory, display_categories):
    BallPlateWorld.__init__(self, controller, log_2_memory, display_categories)
    #Create the movement task, but first make sure it is not already running
    #taskMgr.remove("rollTask")
    self.mainLoop = taskMgr.add(self.rollTask, "rollTask")
    self.mainLoop.last = 0
    
    self.add_slider()

   
  #This is the task that deals with making everything interactive
  def rollTask(self, task):
    #Standard technique for finding the amount of time since the last frame
    dt = task.time - task.last
    task.last = task.time
    self.update_hud()
    self.control_task(task)
    #If dt is large, then there has been a #hiccup that could cause the ball
    #to leave the field if this functions runs, so ignore the frame
    if dt > .2: return Task.cont   

    #The collision handler collects the collisions. We dispatch which function
    #to handle the collision based on the name of what was collided into
    for i in range(self.cHandler.getNumEntries()):
      entry = self.cHandler.getEntry(i)
      name = entry.getIntoNode().getName()
      if   name == "wall_collide":   
          self.wallCollideHandler(entry)
      elif name == "ground_collide": 
          #print "ground collide"
          self.groundCollideHandler(entry)
      elif name == "loseTrigger": 
          "lose"   
          self.loseGame(entry)

    if DISABLE_MOUSE:
        #Read the mouse position and tilt the maze accordingly
        if base.mouseWatcherNode.hasMouse():
          mpos = base.mouseWatcherNode.getMouse() #get the mouse position
          self.alpha = mpos.getY() * -10
          self.beta = mpos.getX() *  10
    
    # set table inclination
    delta_alpha = dt * self.alpha_rot_speed
    delta_beta  = dt * self.beta_rot_speed
    
    self.alpha += delta_alpha
    self.beta += delta_beta
    
    #Finally, we move the ball
    #Update the velocity based on acceleration
    self.ballV += self.accelV * dt * ACCEL
    #Clamp the velocity to the maximum speed
    if self.ballV.lengthSquared() > MAX_SPEED_SQ:
      self.ballV.normalize()
      self.ballV *= MAX_SPEED
    #Update the position based on the velocity
    
    new_position = self.ballRoot.getPos() + (self.ballV * dt)
    x,y,z = new_position 
    self.set_3d_scene(x,y,z, self.alpha, self.beta, dt)
                      
    return Task.cont       #Continue the task indefinitely

  #If the ball hits a hole trigger, then it should fall in the hole.
  #This is faked rather than dealing with the actual physics of it.
  def loseGame(self, entry):
      print "loseGame?"
      return

if __name__ == '__main__':
    #Finally, create an instance of our class and start 3d rendering
    w = World(None)
    run()

