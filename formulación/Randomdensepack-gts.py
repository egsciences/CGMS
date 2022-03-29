# -*- encoding=utf-8 -*-
""" CAUTION:
Running this script can take very long!
"""
import gts, os.path, locale
locale.setlocale(
        locale.LC_ALL, 'en_US.UTF-8'
)  #gts is locale-dependend.  If, for example, german locale is used, gts.read()-function does not import floats normally
'''
if you get "Error: unsupported locale setting"
-> type as root: "dpkg-reconfigure locales"
-> choose "en_US.UTF-8" (press space to choose)
'''
###################################################################
readParamsFromTable(radius=0.09/3, rRelFuzz=0,rCoff=2,bot_limit=1,height_0=1,tot_limit=1,toy_limit=1,ejex_limit=5.5)
from yade.params.table import *

from numpy import arange
from yade import pack
import pylab

Rockfill=O.materials.append(FrictMat(young=100e6,poisson=0.3,density=2650,frictionAngle=radians(30),label='frictMat'))

# define the section shape as polygon in 2d; repeat first point at the end to close the polygo
surf = gts.read(open('talud2.coarse.gts'))
# fill this solid with triaxial packing; it will compute minimum-volume oriented bounding box
# to minimize the number of throw-away spheres.
# It does away with about 3k spheres for radius 3e-2
sp1 = SpherePack()
sp1 = pack.randomDensePack(pack.inGtsSurface(surf), radius=radius, rRelFuzz=rRelFuzz,spheresInCell=1000, memoizeDb='/tmp/gts-triax.sqlite', returnSpherePack=True)

rockfill = sp1.toSimulation()
from yade import qt
qt.View()
######################################################################################################3
#################################################################################

bot = [O.bodies[s] for s in rockfill if O.bodies[s].state.pos[1]<radius*rCoff*1.8]
tot = [O.bodies[s] for s in rockfill if O.bodies[s].state.pos[2]<=radius*rCoff*1.6]
toy = [O.bodies[s] for s in rockfill if O.bodies[s].state.pos[2]>=height_0*(0.9)]
ejex = [O.bodies[s] for s in rockfill if O.bodies[s].state.pos[0]<=radius*rCoff*1.8]
ejexx = [O.bodies[s] for s in rockfill if O.bodies[s].state.pos[0]>=ejex_limit*0.98]


for s in bot:
        if s.state.pos[1]<=bot_limit:
                bot_limit = s.state.pos[1]
                bot_id = s.id
for b in bot: #reemplazar wall por top o bot layers
        b.state.blockedDOFs = 'xyz'
        b.state.vel = (0,0,0)

for s in tot:
        if s.state.pos[2]<=tot_limit:
                tot_limit = s.state.pos[2]
                tot_id = s.id
for b in tot: #reemplazar wall por top o tot layers
        b.state.blockedDOFs = 'xyz'
        b.state.vel = (0,0,0)

for s in toy:
        if s.state.pos[2]<=toy_limit:
                toy_limit = s.state.pos[2]
                toy_id = s.id
for b in toy: #reemplazar wall por top o toy layers
        b.state.blockedDOFs = 'xyz'
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
        NewtonIntegrator(gravity=(0, 9.81,0), damping=0.3),
]
O.dt = .5 * PWaveTimeStep()