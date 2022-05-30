from __future__ import print_function
from itertools import count
from turtle import pos
###################################################################
### Con stabilityThreshold=0.5 sera posible ejecutar el codigo de forma mas rapida y pasar a la etapa del triaxial
readParamsFromTable(rParticle=0.03, rRelFuzz=0,rCoff=2,top_limit=1,bot_limit=1,width=1,tot_limit=1,toy_limit=1,ejex_limit=5.5,num_spheres=500,thick = 0.01,key='_define_a_name_',stabilityThreshold=0.1)##Ojo,que el primer while del unbalanced force nunca llego al valor 0.01 del stabilityThreshold
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
#Material paredes
O.materials.append(FrictMat(young=5e6,poisson=0.5,frictionAngle=0,density=0,label='walls'))
## create walls around the packing
##walls=aabbWalls([mn,mx],thickness=thick,material='walls')
##wallIds=O.bodies.append(walls)
#Define Materials
Rockfill=O.materials.append(FrictMat(young=20e6,poisson=0.15,frictionAngle=radians(30),density=2500,label='spheres'))

surf = gts.read(open('talud2.coarse.gts'))
# Muestra la geometria gts en el modelo
#gtstalud=O.bodies.append(pack.gtsSurface2Facets(surf, color=(1, 0, 1)))

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
        GlobalStiffnessTimeStepper(active=1,timeStepUpdateInterval=100,timestepSafetyCoefficient=0.8),
        NewtonIntegrator(gravity=(0, -9.81,0), damping=0.3),
]


##Aqui podria ir la condicion de estabilidad de las particulas dentro del gts, como se muestra a continuacion##
H=max(O.bodies[s].state.pos[1] for s in rockfill)-min(O.bodies[s].state.pos[1] for s in rockfill)+rParticle
#Ha= H - 3*rParticle*rCoff
#Fija las pastirculas exteriores de la figura en cada eje
#AXIS Y (vertical)
#AXIS X (up-down stream)
#AXIS Z (width)

top = [O.bodies[s] for s in rockfill if O.bodies[s].state.pos[1]>= H - rParticle*rCoff*2] #eje y 
bot = [O.bodies[s] for s in rockfill if O.bodies[s].state.pos[1]<rParticle*rCoff*2] #eje y
tot = [O.bodies[s] for s in rockfill if O.bodies[s].state.pos[2]<=rParticle*rCoff*2]#eje z
toy = [O.bodies[s] for s in rockfill if O.bodies[s].state.pos[2]>=width-rParticle*rCoff*2]#eje z
ejex = [O.bodies[s] for s in rockfill if O.bodies[s].state.pos[0]<=rParticle*rCoff*2] #eje x
ejexx = [O.bodies[s] for s in rockfill if O.bodies[s].state.pos[0]>=ejex_limit-rParticle*rCoff*2]#eje x

#Calculo de fuerza 
SC= 12000 # [Pa]
dx=max(O.bodies[s].state.pos[0] for s in rockfill)-min(O.bodies[s].state.pos[0] for s in rockfill) + rParticle
dz=max(O.bodies[s].state.pos[2] for s in rockfill)-min(O.bodies[s].state.pos[2] for s in rockfill) + rParticle

A=dx*dz #Area superficie top
FT=A*SC #Fuerza Total
FPP=FT/len(top) #Fuerza por particula

for s in top:
        if s.state.pos[1]<=top_limit:
                top_limit = s.state.pos[1]
                top_id = s.id
        #O.forces.setPermF(s.id,(0,-FPP,0))
        s.state.vel = (0,0,0)
        s.shape.color=(0,0,1)
        
for s in bot:
        if s.state.pos[1]<=bot_limit:
                bot_limit = s.state.pos[1]#Define the minimal position Y (vertical) from the dense particles
                bot_id = s.id
        s.state.blockedDOFs = 'xyzXYZ'
        s.state.vel = (0,0,0)
        s.shape.color=(255/255.,0/255.,0/255.)
        
for s in tot:
        if s.state.pos[2]<=tot_limit:
                tot_limit = s.state.pos[2]
                tot_id = s.id
        s.state.blockedDOFs = 'xyzXYZ'
        s.state.vel = (0,0,0)
        s.shape.color=(255/255.,0/255.,0/255.)
        
for s in toy:
        if s.state.pos[2]<=toy_limit:
                toy_limit = s.state.pos[2]
                toy_id = s.id
        s.state.blockedDOFs = 'xyzXYZ'
        s.state.vel = (0,0,0)
        s.shape.color=(255/255.,0/255.,0/255.)
        
for s in ejex:
        if s.state.pos[0]<=ejex_limit:
                ejex_limit = s.state.pos[0]
                ejex_id = s.id
        s.state.blockedDOFs = 'xyz'
        s.state.vel = (0,0,0)
        s.shape.color=(255/255.,0/255.,0/255.)
        
for s in ejexx:
        if s.state.pos[0]<=ejex_limit:
                ejexx_limit = s.state.pos[0]
                ejexx_id = s.id
        s.state.blockedDOFs = 'xyz'
        s.state.vel = (0,0,0)
        s.shape.color=(255/255.,0/255.,0/255.)

count=0
for s in rockfill:
        if O.bodies[s].state.pos[1]<0:
                count+=1
        #print(count)

        
#################################################################
### CONDICION DE ESTABILIDAD DE LAS PARTICULAS DENTRO DEL GTS ###
#################################################################

while 1:#Se busca estabilizar las particulas dentro del gts.
  O.run(500, True)
  #the global unbalanced force on dynamic bodies, thus excluding boundaries, which are not at equilibrium
  unb=unbalancedForce()
  print('unbalanced force:',unb)
  if unb<stabilityThreshold:
    break

O.dt=.1*PWaveTimeStep() # initial timestep, to not explode right away
O.usesTimeStepper=True

#Display spheres with 2 colors for seeing rotations better
Gl1_Sphere.stripes=0
yade.qt.Controller(), yade.qt.View()

##############################
### ELIMINAR EL GTS###
##############################

#for b in gtstalud:
#        O.bodies.erase(b)
################################

######## ESTE CODIGO PUEDE SER MODIFICADO, POR SI POR EJEMPLO, SE QUIEREN INCREMENTOS DE ESFUERZOS IMPUESTOS ETC. VIENDO LOS EJEMPLOS DEL SIGUIENTE LINK: https://gricad-gitlab.univ-grenoble-alpes.fr/cailletr/yade/-/tree/d67436b26377d3e394760d3e3ba01e8cd7539cf7/examples/triax-tutorial####


### declare what is to plot. "None" is for separating y and y2 axis
#plot.plots={'i':('e11','e22','e33',None,'s11','s22','s33')}

##display on the screen (doesn't work on VMware image it seems)
##plot.plot()

## In that case we can still save the data to a text file at the the end of the simulation, with: 
#plot.saveDataTxt('results'+key)
##or even generate a script for gnuplot. Open another terminal and type  "gnuplot plotScriptKEY.gnuplot:
#plot.saveGnuplot('plotScript'+key)
