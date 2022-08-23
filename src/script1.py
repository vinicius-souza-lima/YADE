from yade import pack

sp = pack.SpherePack()
sp.makeCloud((1,1,1),(2,2,2), rMean =.2)

for c,r in sp:
    print(c,r)

sp.toSimulation()

O.bodies.append(utils.wall(-1,axis=2))

O.engines=[                   # newlines and indentations are not important until the brace is closed
        ForceResetter(),
        InsertionSortCollider([Bo1_Sphere_Aabb(),Bo1_Wall_Aabb()]),
        InteractionLoop(           # dtto for the parenthesis here
                 [Ig2_Sphere_Sphere_ScGeom(),Ig2_Wall_Sphere_ScGeom()],
                 [Ip2_FrictMat_FrictMat_FrictPhys()],
                 [Law2_ScGeom_FrictPhys_CundallStrack()]
        ),
        GravityEngine(gravity=(0,0,-9.81)),                    # 9.81 is the gravity acceleration, and we say that
        NewtonIntegrator(damping=.2,label='newtonCustomLabel') # define a label under which we can access this engine easily
]

O.dt=utils.PWaveTimeStep()

O.saveTmp()
