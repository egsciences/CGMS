from __future__ import print_function
###################################################################
### Con stabilityThreshold=0.5 sera posible ejecutar el codigo de forma mas rapida y pasar a la etapa del triaxial
readParamsFromTable(rParticle=0.09, rRelFuzz=0,rCoff=2,bot_limit=1,width=1,tot_limit=1,toy_limit=1,ejex_limit=5.5,num_spheres=500,thick = 0.01,key='_define_a_name_',stabilityThreshold=0.01)##Ojo,que el primer while del unbalanced force nunca llego al valor 0.01 del stabilityThreshold
from yade.params.table import *
from numpy import arange
from yade import pack
import pylab
from yade import pack
import gts, os.path, locale
####### Necesario para la ejecucion de la geometria gts
locale.setlocale(
        locale.LC_ALL, 'en_US.UTF-8'
)

############################################
### DEFINIR VARIABLES Y MATERIALES ###
############################################

## corners of the initial packing
mn,mx=Vector3(0,0,0),Vector3(5.5,1,1) #Ubicacion paredes del triaxial
compFricDegree = 30
finalFricDegree = 30
rate=0.2
damp=0.7
young=5e6 
#Material paredes
O.materials.append(FrictMat(young=5e6,poisson=0.5,frictionAngle=0,density=0,label='walls'))
## create walls around the packing
walls=aabbWalls([mn,mx],thickness=thick,material='walls')
wallIds=O.bodies.append(walls)
#Define Materials
Rockfill=O.materials.append(FrictMat(young=100e6,poisson=0.3,frictionAngle=radians(30),density=2650,label='spheres'))

surf = gts.read(open('talud2.coarse.gts'))
# Muestra la geometria gts en el modelo
gtstalud=O.bodies.append(pack.gtsSurface2Facets(surf, color=(1, 0, 1)))


# fill this solid with triaxial packing; it will compute minimum-volume oriented bounding box
# to minimize the number of throw-away spheres.
# It does away with about 3k spheres for rParticle 3e-2
sp1 = SpherePack()
sp1 = pack.randomDensePack(pack.inGtsSurface(surf),radius=rParticle,material=Rockfill, rRelFuzz=rRelFuzz,spheresInCell=num_spheres, memoizeDb='/tmp/gts-triax.sqlite', returnSpherePack=True)
rockfill = sp1.toSimulation()

############################################
### PRIMER ENGINE DE PRUEBA ###
############################################

O.engines = [
        ForceResetter(),
        # sphere, facet, wall
        InsertionSortCollider([Bo1_Sphere_Aabb(), Bo1_Facet_Aabb(), Bo1_Wall_Aabb()]),
        InteractionLoop(
                # the loading plate is a wall, we need to handle sphere+sphere, sphere+facet, sphere+wall
                [Ig2_Sphere_Sphere_ScGeom(), Ig2_Facet_Sphere_ScGeom(), Ig2_Wall_Sphere_ScGeom()],
                [Ip2_FrictMat_FrictMat_FrictPhys()],
                [Law2_ScGeom_FrictPhys_CundallStrack()]
        ),
        NewtonIntegrator(gravity=(0, -9.81,0), damping=0.99),
]

##Aqui podria ir la condicion de estabilidad de las particulas dentro del gts, como se muestra a continuacion##

#while 1:#Se busca estabilizar las particulas dentro del gts.
  #O.run(1000, True)
  ##the global unbalanced force on dynamic bodies, thus excluding boundaries, which are not at equilibrium
  #unb=unbalancedForce()
  #print('unbalanced force:',unb)
  #if unb<stabilityThreshold:
    #break

#Fija las pastirculas exteriores de la figura en cada eje
#AXIS Y (vertical)
#AXIS X (up-down stream)
#AXIS Z (width)
bot = [O.bodies[s] for s in rockfill if O.bodies[s].state.pos[1]<rParticle*rCoff*2] #eje y
tot = [O.bodies[s] for s in rockfill if O.bodies[s].state.pos[2]<=rParticle*rCoff*2]#eje z
toy = [O.bodies[s] for s in rockfill if O.bodies[s].state.pos[2]>=width-rParticle*rCoff*2]#eje z
ejex = [O.bodies[s] for s in rockfill if O.bodies[s].state.pos[0]<=rParticle*rCoff*2] #eje x
ejexx = [O.bodies[s] for s in rockfill if O.bodies[s].state.pos[0]>=ejex_limit-rParticle*rCoff*2]#eje x

for s in bot:
        if s.state.pos[1]<=bot_limit:
                bot_limit = s.state.pos[1]#Define the minimal position Y (vertical) from the dense particles
                bot_id = s.id
for b in bot: 
        b.state.blockedDOFs = 'xyzXYZ'
        b.state.vel = (0,0,0)

for s in tot:
        if s.state.pos[2]<=tot_limit:
                tot_limit = s.state.pos[2]
                tot_id = s.id
for b in tot: #reemplazar wall por top o tot layers
        b.state.blockedDOFs = 'xyzXYZ'
        b.state.vel = (0,0,0)

for s in toy:
        if s.state.pos[2]<=toy_limit:
                toy_limit = s.state.pos[2]
                toy_id = s.id
for b in toy: #reemplazar wall por top o toy layers
        b.state.blockedDOFs = 'xyzXYZ'
        b.state.vel = (0,0,0)

for s in ejex:
        if s.state.pos[0]<=ejex_limit:
                ejex_limit = s.state.pos[0]
                ejex_id = s.id
for b in ejex: #reemplazar wall por top o ejex layers
        b.state.blockedDOFs = 'xyz'
        b.state.vel = (0,0,0)

for s in ejexx:
        if s.state.pos[0]<=ejex_limit:
                ejexx_limit = s.state.pos[0]
                ejexx_id = s.id
for b in ejexx: #reemplazar wall por top o ejexx layers
        b.state.blockedDOFs = 'xyz'
        b.state.vel = (0,0,0)

#################################################################
### CONDICION DE ESTABILIDAD DE LAS PARTICULAS DENTRO DEL GTS ###
#################################################################

while 1:#Se busca estabilizar las particulas dentro del gts.
  O.run(1000, True)
  #the global unbalanced force on dynamic bodies, thus excluding boundaries, which are not at equilibrium
  unb=unbalancedForce()
  print('unbalanced force:',unb)
  if unb<stabilityThreshold:
    break

O.dt=.5*PWaveTimeStep() # initial timestep, to not explode right away
O.usesTimeStepper=True

############################################
### DEFINIR ENGINE DE LA PRUEBA TRIAXIAL ###
############################################

triax=ThreeDTriaxialEngine(
	maxMultiplier=1.+2e4/young, # spheres growing factor (fast growth)
	finalMaxMultiplier=1.+2e3/young, # spheres growing factor (slow growth)
	thickness = thick,
	stressControl_1 = True,
	stressControl_2 = True,
	stressControl_3 = True,
	## Independant stress values for anisotropic loadings
	goal1=-10000,
	goal2=-10000,
	goal3=-10000,
	internalCompaction=True, # If true the confining pressure is generated by growing particles
	Key=key, # passed to the engine so that the output file will have the correct name
)

NewtonIntegrator(gravity=(0,-9.81,0),damping=damp,label='newton')

O.engines=[
 ForceResetter(),
 InsertionSortCollider([Bo1_Sphere_Aabb(),Bo1_Box_Aabb(), Bo1_Facet_Aabb()]),
 InteractionLoop(
  [Ig2_Sphere_Sphere_ScGeom(),Ig2_Box_Sphere_ScGeom(),Ig2_Facet_Sphere_ScGeom()],
  [Ip2_FrictMat_FrictMat_FrictPhys()],
  [Law2_ScGeom_FrictPhys_CundallStrack()]
 ),
 ## We will use the global stiffness of each body to determine an optimal timestep (see https://yade-dem.org/w/images/1/1b/Chareyre&Villard2005_licensed.pdf)
 GlobalStiffnessTimeStepper(active=1,timeStepUpdateInterval=100,timestepSafetyCoefficient=0.8),
 triax,
 TriaxialStateRecorder(iterPeriod=100,file='WallStresses'+key),
 newton
]


#Display spheres with 2 colors for seeing rotations better
Gl1_Sphere.stripes=0
yade.qt.Controller(), yade.qt.View()

#######################################
### APLICAR PRESION DE CONFINAMIENTO ###
#######################################

while 1:
  O.run(1000, True)
  #the global unbalanced force on dynamic bodies, thus excluding boundaries, which are not at equilibrium
  unb=unbalancedForce()
  #average stress
  #note: triax.stress(k) returns a stress vector, so we need to keep only the normal component
  meanS=(triax.stress(triax.wall_right_id)[0]+triax.stress(triax.wall_top_id)[1]+triax.stress(triax.wall_front_id)[2])/3
  print('unbalanced force:',unb,' mean stress: ',meanS)
  if unb<stabilityThreshold*0.1 and abs(meanS+10000)/10000<0.001:
    break

O.save('compressedState'+key+'.xml')
print("###      Isotropic state saved      ###")

##############################
### ESFUERZO DESVIADOR ###
##############################

#We move to deviatoric loading, let us turn internal compaction off to keep particles sizes constant)
triax.internalCompaction=True#False 

### Change contact friction (remember that decreasing it would generate instantaneous instabilities)
setContactFriction(radians(finalFricDegree))

#... and make stress control independant on each axis
triax.stressControl_1=triax.stressControl_2=triax.stressControl_3=True
# We have to turn all these flags true, else boundaries will be fixed
triax.wall_bottom_activated=True
triax.wall_top_activated=True#False (False para que la pared superior baje, mientras las otras se mantienen fijas)
triax.wall_left_activated=True
triax.wall_right_activated=True
triax.wall_back_activated=True
triax.wall_front_activated=True


#If we want a triaxial loading at imposed strain rate, let's assign srain rate instead of stress
triax.stressControl_2=0 #we are tired of typing "True" and "False", we use implicit conversion from integer to boolean
triax.strainRate2=0.01
triax.strainRate1=triax.strainRate3=1000.0

O.run()

##############################
### ELIMINAR EL GTS###
##############################

#for b in gtstalud:
#        O.bodies.erase(b)
################################

######## ESTE CODIGO PUEDE SER MODIFICADO, POR SI POR EJEMPLO, SE QUIEREN INCREMENTOS DE ESFUERZOS IMPUESTOS ETC. VIENDO LOS EJEMPLOS DEL SIGUIENTE LINK: https://gricad-gitlab.univ-grenoble-alpes.fr/cailletr/yade/-/tree/d67436b26377d3e394760d3e3ba01e8cd7539cf7/examples/triax-tutorial####

#####################################################
###    EJEMPLO DE COMO REGISTRAR Y TRAZAR DATOS   ###
#####################################################

#from yade import plot

### a function saving variables
#def history():
  	#plot.addData(e11=triax.strain[0], e22=triax.strain[1], e33=triax.strain[2],
		    #s11=triax.stress(triax.wall_right_id)[0],
		    #s22=triax.stress(triax.wall_top_id)[1],
		    #s33=triax.stress(triax.wall_front_id)[2],
		    #i=O.iter)

#if 1:
  ## include a periodic engine calling that function in the simulation loop
  #O.engines=O.engines[0:5]+[PyRunner(iterPeriod=20,command='history()',label='recorder')]+O.engines[5:7]
  ##O.engines.insert(4,PyRunner(iterPeriod=20,command='history()',label='recorder'))
#else:
  ## With the line above, we are recording some variables twice. We could in fact replace the previous
  ## TriaxialRecorder
  ## by our periodic engine. Uncomment the following line:
  #O.engines[4]=PyRunner(iterPeriod=20,command='history()',label='recorder')

#O.run(100,True)

### declare what is to plot. "None" is for separating y and y2 axis
#plot.plots={'i':('e11','e22','e33',None,'s11','s22','s33')}

##display on the screen (doesn't work on VMware image it seems)
##plot.plot()

## In that case we can still save the data to a text file at the the end of the simulation, with: 
#plot.saveDataTxt('results'+key)
##or even generate a script for gnuplot. Open another terminal and type  "gnuplot plotScriptKEY.gnuplot:
#plot.saveGnuplot('plotScript'+key)


