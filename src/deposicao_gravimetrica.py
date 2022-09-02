# Deposição gravimétrica em uma caixa

# importando os módulos do yade que vamos utilizar
from yade import pack, plot

# cria caixa retangular
O.bodies.append(geom.facetBox((.5, .5, .5), (.5, .5, .5), wallMask=31))

# cria empacotamento de esferas vazio
sp = pack.SpherePack()
# Gera esferas de forma aleatória com distribuição de raios uniforme
sp.makeCloud((0, 0, 0), (1, 1, 1), rMean=.05, rRelFuzz=.5)
# Adiciona o empacotamento à simulação
sp.toSimulation()

O.engines = [
    ForceResetter(),
    InsertionSortCollider([Bo1_Sphere_Aabb(), Bo1_Facet_Aabb()]),
    InteractionLoop(
        # handle sphere+sphere and facet+sphere collisions
        [Ig2_Sphere_Sphere_ScGeom(), Ig2_Facet_Sphere_ScGeom()],
        [Ip2_FrictMat_FrictMat_FrictPhys()],
        [Law2_ScGeom_FrictPhys_CundallStrack()]
    ),
    NewtonIntegrator(gravity=(0, 0, -9.81), damping=0.4),
    # call the checkUnbalanced function (defined below) every 2 seconds
    PyRunner(command='checkUnbalanced()', realPeriod=2),
    # call the addPlotData function every 200 steps
    PyRunner(command='addPlotData()', iterPeriod=100)
]
O.dt = .5 * PWaveTimeStep()

# enable energy tracking; any simulation parts supporting it
# can create and update arbitrary energy types, which can be
# accessed as O.energy['energyName'] subsequently
O.trackEnergy = True


# if the unbalanced forces goes below .05, the packing
# is considered stabilized, therefore we stop collected
# data history and stop
def checkUnbalanced():
    if unbalancedForce() < .05:
        O.pause()
        plot.saveDataTxt('bbb.txt.bz2')
        # plot.saveGnuplot('bbb') is also possible


# collect history of data which will be plotted
def addPlotData():
    # each item is given a names, by which it can be the unsed in plot.plots
    # the **O.energy converts dictionary-like O.energy to plot.addData arguments
    plot.addData(i=O.iter, forca_nao_balanceada=unbalancedForce(), **O.energy)


# define how to plot data: 'i' (step number) on the x-axis, unbalanced force
# on the left y-axis, all energies on the right y-axis
# (O.energy.keys is function which will be called to get all defined energies)
# None é usado para separar os eixos verticais esquerdo e direito
plot.plots = {'i': ('forca_nao_balanceada'), 'i ': (O.energy.keys)}

# Mostra o gráfico na tela, e atualiza enquanto a simulação é executada
plot.plot()