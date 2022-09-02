# basic simulation showing sphere falling ball gravity,
# bouncing against another sphere representing the support

# DATA COMPONENTS

# add 2 particles to the simulation
# they the default material (utils.defaultMat)
O.bodies.append(
        [
                # fixed: particle's position in space will not change (support)
                sphere(center=(0, 0, 0), radius=.5, fixed=True),
                # this particles is free, subject to dynamics
                sphere((0, 0, 2), .5)
        ]
)

movel = O.bodies[1]

# FUNCTIONAL COMPONENTS

# simulation loop -- see presentation for the explanation
O.engines = [
        ForceResetter(),
        InsertionSortCollider([Bo1_Sphere_Aabb()]),
        InteractionLoop(
                [Ig2_Sphere_Sphere_ScGeom()],  # collision geometry
                [Ip2_FrictMat_FrictMat_FrictPhys()],  # collision "physics"
                [Law2_ScGeom_FrictPhys_CundallStrack()]  # contact law -- apply forces
        ),
        # Apply gravity force to particles. damping: numerical dissipation of energy.
        NewtonIntegrator(gravity=(0, 0, -9.81), damping=0.1)
]

from yade import plot

# Periodic storing of data is done with PyRunner and the plot.addData function. Also let’s enable energy tracking:

O.trackEnergy=True
def addPlotData():
        # this function adds current values to the history of data, under the names specified
        plot.addData(t=O.time,
        Ek=utils.kineticEnergy()
        ,N=utils.avgNumInteractions(),
        unForce=utils.unbalancedForce(),
        totalEnergy=abs(O.energy.total()),
        h = movel.state.pos[2])
# Now this function can be added to O.engines:

O.engines+=[PyRunner(command='addPlotData()',iterPeriod=20)]

# set timestep to a fraction of the critical timestep
# the fraction is very small, so that the simulation is not too fast
# and the motion can be observed
O.dt = .5e-3 * PWaveTimeStep()

plot.plots={'t':('h'),'t ':('Ek')}                # kinetic energy will have legend on the right as indicated by None separator.
# Abre o plot na tela
plot.plot()

# save the simulation, so that it can be reloaded later, for experimentation
O.saveTmp()