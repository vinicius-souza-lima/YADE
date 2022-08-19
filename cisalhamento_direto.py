# encoding: utf-8

# script para teste de cisalhamento simples periódico, com limites periódicos
# primeiro comprime para atingir algum estrese isotrópico (checkStress),
# então carrega o cisalhamento (checkDistorsion)
#
# o empacotamento inicial é regular (hexagonal), com bandas vazias ao longo dos limites,
# ou uma nuvem de esferas aleatórias e periódicas
#
# ângulo de fricção material é inicialemnte colocado para zero, de forma que o empacotamento resultante é denso.
# (Rearranjar esferas é mais fácil se não há atrito)
#

# Configura o limite periódico
from __future__ import print_function
O.periodic = True
O.cell.hSize = Matrix3(2, 0, 0, 
					   0, 2, 0,
					   0, 0, 2)

from yade import pack, plot

# "if 0" para regular, "if 1"para nuvem de esferas

if 0:
	# cria nuvem de esferas e insere-as na simulação
	# Damos os cantos, raio médio, variação do raio
	sp = pack.SpherePack()
	sp.makeCloud((0, 0, 0), (2, 2, 2), rMean=.1, rRelFuzz=.6, periodic=True)
	# insere o empacotamento na simulação
	sp.toSimulation(color=(0, 0, 1))  # azul puro
else:
	# Nesse caso, adiciona empacotamento denso
	O.bodies.append(pack.regularHexa(pack.inAlignedBox((0, 0, 0), (2, 2, 2)), radius=.05, gap=0, color=(0, 0, 1)))

# cria empacotamento "denso" ao colocar atrito igual a zero inicialmente
O.materials[0].frictionAngle = 0

# Loop da simulação (vai rodar à cada passo)
O.engines = [
        ForceResetter(),
        InsertionSortCollider([Bo1_Sphere_Aabb()]),
        InteractionLoop(
                # loop de interação
                [Ig2_Sphere_Sphere_ScGeom()],
                [Ip2_FrictMat_FrictMat_FrictPhys()],
                [Law2_ScGeom_FrictPhys_CundallStrack()]
        ),
        NewtonIntegrator(damping=.4),
        # Roda a função checkStress (definida abaixo) todo segundo
        # O rótulo é arbitrário,  e é usado depois para se referir à engine
        PyRunner(command='checkStress()', realPeriod=1, label='checker'),
        # Grava informação para plotagem à cada 100 passos; Função addData é definida abaixo
        PyRunner(command='addData()', iterPeriod=100)
]

# Intervalo de tempo de integração igual à metade do intervalo de tempo crítico
O.dt = .5 * PWaveTimeStep()

# Define a deformação isotrópica normal(taxa de deformação constante)
# da célula periódica
O.cell.velGrad = Matrix3( -.1,   0 ,   0 ,
						  	0, -.1 ,   0 ,
						    0,   0 , -.1)

# quando parar a compressão isotrópica (usada dentro do checkStress)
limitMeanStress = -5.95e5


# Chamada a cada segundo pela engine PyRunner
def checkStress():
	# Tensor de tensões como a soma de constribuições normais e tangentes
	# Matrix3.Zero é o valor inicial para soma(...)
	stress = getStress().trace() / 3.
	tens_stress = sum(normalShearStressTensors(), Matrix3.Zero)
	print('mean stress', stress)
	# Se a tensão média é menor (maior em valor absoluto)  que limitMeanStress, começa o cisalhamento
	if stress < limitMeanStress:
		# Aplica deformação à taxa constante da célula periódica
		O.cell.velGrad = Matrix3(0, 0, 0.09,
								 0, 0, 0, 
								 0, 0, 0)
		# Muda a função chamada pela engine checadora
		# (checkStress não será mais chamada)
		checker.command = 'checkDistorsion()'
		# bloqueia rotações de partículas para aumentar tanPhi, se desejado
		# Desativado por padrão(para ativar mude para 'if 1')
		if 0:
			for b in O.bodies:
				# bloqueia rotações em X,Y,Z, translações são livres
				b.state.blockedDOFs = 'XYZ'
				# Para rotações se existirem, como blockedDOFs bloqueia aceleração apenas
				b.state.angVel = (0, 0, 0)
		# Coloca ângulo de fricção de volta à valor não-nulo
		# tangensOfFrictionAngle é computado pelo Ip2_* functor do material
		# Para futuros contatos mudar material (há apenas um material para todas as partículas)
		O.materials[0].frictionAngle = .3  # radianos
		# Para contatos existentes, coloca fricção de contato diretamente
		for i in O.interactions:
			i.phys.tangensOfFrictionAngle = tan(.3)


# Chama a engine checadora periodicamente, durante a fase de cisalhamento
def checkDistorsion():
	# Se o valor da distorsão é >.5, termina a execução; De outra forma, não faz nada.
	if abs(O.cell.trsf[0, 2]) > .5:
		# Salva informação de addData(...) antes de exportar para um arquivo
		# use O.tags['id'] para diferenciar execuções individuais de cada simulação
		plot.saveDataTxt(O.tags['id'] + '.txt')
		# Sai do programa
		#importa sys
		#sys.exit(0) # Sem erro (0)
		O.pause()


# Chamado periodicamente para armazenar o histórico de informações
def addData():
	# Obtém o tensor de tensões (como uma matriz 3x3)
	stress = sum(normalShearStressTensors(), Matrix3.Zero)
	# Dá nomes aos valores que estamos interessados e os salva.
	plot.addData(exz=O.cell.trsf[0, 2], szz=stress[2, 2], sxz=abs(stress[0, 2]), tanPhi=(stress[0, 2] / stress[2, 2]) if stress[2, 2] != 0 else 0, i=O.iter)
	# Colore partículas baseada no quanto rotacionou
	for b in O.bodies:
		# rot() dá  o vetor de rotação entre  a referência e a posição atual.
		b.shape.color = scalarOnColorScale(b.state.rot().norm(), 0, pi / 2.)


# define o que plotar (3 plots no total)
## exz(i), [eixo y da esquerda, separado por None:] szz(i), sxz(i)
## szz(exz), sxz(exz)
## tanPhi(i)
# Note o espaço em 'i ' para que não seja reescrita a entrada i
plot.plots = {'exz': ('sxz','szz'), 'i ': ('tanPhi',)}

# Melhor demonstração da rotação das partículas
Gl1_Sphere.stripes = True

# Abre o plot na tela
plot.plot()

O.saveTmp()
