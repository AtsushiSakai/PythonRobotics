<img src="https://github.com/AtsushiSakai/PythonRobotics/raw/master/icon.png?raw=true" align="right" width="300" alt="header pic"/>

# PythonRobotics
![GitHub_Action_Linux_CI](https://github.com/AtsushiSakai/PythonRobotics/workflows/Linux_CI/badge.svg)
![GitHub_Action_MacOS_CI](https://github.com/AtsushiSakai/PythonRobotics/workflows/MacOS_CI/badge.svg)
[![Build status](https://ci.appveyor.com/api/projects/status/sb279kxuv1be391g?svg=true)](https://ci.appveyor.com/project/AtsushiSakai/pythonrobotics)
[![codecov](https://codecov.io/gh/AtsushiSakai/PythonRobotics/branch/master/graph/badge.svg)](https://codecov.io/gh/AtsushiSakai/PythonRobotics)
[![Language grade: Python](https://img.shields.io/lgtm/grade/python/g/AtsushiSakai/PythonRobotics.svg?logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/AtsushiSakai/PythonRobotics/context:python)
[![tokei](https://tokei.rs/b1/github/AtsushiSakai/PythonRobotics)](https://github.com/AtsushiSakai/PythonRobotics)

Códigos em python para algoritmos de robótica.


# Índice de Conteúdo
   * [Sobre o projeto](#sobre-o-projeto)
   * [Requisitos](#requisitos)
   * [Documentação](#documentação)
   * [Como usar](#como-usar)
   * [Algoritmos de Localização](#algoritmos-de-localização)
      * [Filtro de kalman estendido](#filtro-de-kalman-estendido)
      * [Filtro de partícula](#filtro-de-partículas)
      * [Filtro por Histograma](#filtro-por-histograma)
   * [Algoritmos de Mapeamento](#algoritmos-de-mapeamento)
      * [Mapa de grade Gaussiana](#mapa-de-grade-gaussiana)
      * [Mapa de grade Ray Casting](#mapa-de-grade-ray-casting)
      * [Lidar para mapa de grade](#lidar-para-mapa-de-grade)
      * [Clusterização de dados k-means](#clusterização-de-dados-k-means)
      * [Encaixe com retângulos](#encaixe-com-retângulos)
   * [SLAM](#slam)
      * [Correspondência de ponto mais próximo iterativo (ICP)](#correspondência-de-ponto-mais-próximo-iterativo-(icp))
      * [FastSLAM 1.0](#fastslam-10)
   * [Planejamento de caminho](#planejamento-de-caminho)
      * [Abordagem janela dinâmica](#abordagem-janela-dinâmica)
      * [Busca baseada em grade](#busca-baseada-em-grade)
         * [Algoritmo de Dijkstra](#algoritmo-de-dijkstra)
         * [Algoritmo A*](#algoritmo-a)
         * [Algoritmo D*](#algoritmo-d)
         * [Algoritmo D* lite](#algoritmo-d-lite)
         * [Algoritmo potencial de campo](#algoritmo-potencial-de-campo)
         * [Planejamento de caminho baseado em grade](#planejamento-de-caminhode-cobertura-baseado-em-grade)
      * [Planejamento por rede de estado](#planejamento-por-rede-de-estado)
         * [Amostragem polar tendenciosa](#amostragem-polar-tendenciosa)
         * [Amostragem de caminho](#amostragem-de-caminho)
      * [Planejamento de caminho probabilístico](#planejamento-de-caminho-probabilístico)
      * [Árvore aleatória de exploração rápida](#árvore-aleatória-de-exploração-rápida)
         * [RRT*](#rrt)
         * [RRT* com trajetória reeds-shepp](#rrt*-com-trajetória-reeds-shepp)
         * [LQR-RRT*](#lqr-rrt)
      * [Planejamento de polinômio de quinta ordem](#planejamento-de-polinômio-de-quinta-ordem)
      * [Planejamento de caminho Reeds Shepp](#planejamento-de-caminho-reeds-shepp)
      * [Planejamento de caminho baseado em LQR](#planejamento-de-trajetória-baseado-em-lqr)
      * [Trajetória ótima em um Frenet Frame](#trajetória-ótima-em-um-frenet-frame)
   * [Rastreamento de Caminho](#reastramento-de-caminho)
      * [Mover para uma posição de controle](#mover-para-uma-posição-de-controle)
      * [Controle Stanley](#controle-stanley)
      * [Controle de feedback da roda traseira](#controle-de-feedback-da-roda-traseira)
      * [Controle de velocidade e direção usando LQR(linear-quadratic regulator)](#controle-de-velocidade-e-direção-usando-lqr(linear-quadratic-regulator))
      * [Modelo preditivo de velocidade e controle de posição](#modelo-preditivo-de-velocidade-e-controle-de-posição)
      * [Modelo preditivo não linear de controle com C-GMRES](#nonlinear-model-predictive-control-with-c-gmres)
   * [Movimento de braço robótico](#movimento-de-braço-robótico)
      * [Número N de juntas até um ponto de controle](#número-n-de-juntas-até-um-ponto-de-controle)
      * [Movimento de braço robótico com obstáculo](#movimento-de-braço-robótico-com-obstáculo)
   * [Navegação Aérea](#navegação-aérea)
      * [Drone se movendo em plano tridimensional(3D)](#drone-se-movendo-em-plano-tridimensional)
      * [Pouso de um foguete](#pouso-de-um-foguete)
   * [ Robôs Bípedes](#robôs-bípedes)
      * [Plajemento de movimento bípede com pêndulo invertido](#planejamento-de-movimento-bípede-com-pêndulo-invertido)
   * [Licença](#licença)
   * [Caso de Uso](#caso-de-uso)
   * [Contribuição](#contribuição)
   * [Citando o projeto](#citando-o-projeto)
   * [Apoiar](#apoiar)
   * [Patrocinadores](#patrocinadores)
      * [JetBrains](#JetBrains)
      * [1Password](#1password)
   * [Autores](#autores)

# Sobre o Projeto

Coleção de algoritmos de robótica feitos em python.

Recursos:

1. Fácil de ler para entender a idéia básica de cada algoritmo.

2. Os algoritmos amplamente usados e práticos foram selecionados.

3. Depêndencia mínima.

Leia esse documento para mais detalhes:

- [\[1808\.10703\] PythonRobotics: a Python code collection of robotics algorithms](https://arxiv.org/abs/1808.10703) ([BibTeX](https://github.com/AtsushiSakai/PythonRoboticsPaper/blob/master/python_robotics.bib))


# Requisitos

Para executar cada exemplo de código:

- [Python 3.10.x](https://www.python.org/)
 
- [NumPy](https://numpy.org/)
 
- [SciPy](https://scipy.org/)
 
- [Matplotlib](https://matplotlib.org/)
 
- [cvxpy](https://www.cvxpy.org/) 

Para o desenvolvimento:
  
- [pytest](https://pytest.org/) (para testes de unidade)
  
- [pytest-xdist](https://pypi.org/project/pytest-xdist/) (para testes paralelos de unidade)
  
- [mypy](http://mypy-lang.org/) (para checagem de tipo)
  
- [sphinx](https://www.sphinx-doc.org/) (geração de documentação)
  
- [pycodestyle](https://pypi.org/project/pycodestyle/) (chegagem de tipo de estilização)

# Documentação

Esse arquivo README mostra apenas alguns exemplos desse projeto. 

Se você estiver interresado em outros exemplos ou ainda na matemática por trás de cada algoritmo, 

Você pode conferir a documentação completa online através do link: [Welcome to PythonRobotics’s documentation\! — PythonRobotics documentation](https://atsushisakai.github.io/PythonRobotics/index.html)

Todas as animações e gifs estão armazenadas em: [AtsushiSakai/PythonRoboticsGifs: Animation gifs of PythonRobotics](https://github.com/AtsushiSakai/PythonRoboticsGifs)

# Como usar

1. Clone esse repositório.

   ```terminal
   git clone https://github.com/AtsushiSakai/PythonRobotics.git
   ```


2. Instale as bibliotecas necessárias.

- usando conda :

  ```terminal
  conda env create -f requirements/environment.yml
  ```
 
- usando pip :

  ```terminal
  pip install -r requirements/requirements.txt
  ```


3. Execute o script em python em cada diretório.

4. Adicione esse repositório aos favoritos se gostar :smiley:. 

# Algoritmos de Localização

## Localização por filtro estendido de kalman

<img src="https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/Localization/extended_kalman_filter/animation.gif" width="640" alt="EKF pic">

Documentação: [Notebook](https://github.com/AtsushiSakai/PythonRobotics/blob/master/Localization/extended_kalman_filter/extended_kalman_filter_localization.ipynb)

## Localização por filtro de partícula

![2](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/Localization/particle_filter/animation.gif)

Essa é uma fusão de sensores com o filtro de partícula(FP).

A linha azul é a trajetória verdadeira, a linha preta é a trajetória calculada morta,

a linha vermelha é a trajetória estimada com FP.

Assume-se que o robô pode medir uma distância entre pontos de referência (RFID).

Essas medidas são utilizadas para localização de FP.

Referência:

- [PROBABILISTIC ROBOTICS](http://www.probabilistic-robotics.org/)


## Localização por filtro de histograma

![3](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/Localization/histogram_filter/animation.gif)

Este é um exemplo de localização em 2D utilizando filtro de histograma.

A cruz vermelha é a posição verdadeira,os pontos pretos são as posições RFID.

A grade azul mostra uma probabilidade de posição do filtro de histograma.  

Nessa simulação, x,y são desconhecidos e yaw é conhecido.

O filtro integra a entrada de velocidade e observações de alcance de RFID para a localização.

A posição inicial não é necessária.

Referência:

- [PROBABILISTIC ROBOTICS](http://www.probabilistic-robotics.org/)

# Algoritmos de Mapeamento

## Mapeamento de grade gaussiana

Esse é um exemplo de mapeamento de grade gaussiana 2D

![2](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/Mapping/gaussian_grid_map/animation.gif)

## Mapa de grade ray casting

Exemplo em 2D do mapeamento de grade por ray casting.

![2](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/Mapping/raycasting_grid_map/animation.gif)

## Lidar para mapa de grade

Esse exemplo mostra como converter um intervalo de medição 2D em um mapa de grade.

![2](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/Mapping/lidar_to_grid_map/animation.gif)

## Clusterização de objetos k-means.

Exemplo de um cluster de objetos 2D com algoritmo k-means.

![2](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/Mapping/kmeans_clustering/animation.gif)

## Encaixe com Retângulos

Exemplo de um retângulo adequado para detecção de veículos.

![2](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/Mapping/rectangle_fitting/animation.gif)


# SLAM

Exemplos de localização e mapeamento simultâneos(SLAM)

## Iterative Closest Point (ICP) Matching

Esse exemplo de corresponência de ICP 2D com decomposição de valor singular.

O algoritmo pode calcular uma matriz de rotação e um vetor de translação entre pontos.

![3](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/SLAM/iterative_closest_point/animation.gif)

Referência:

- [Introduction to Mobile Robotics: Iterative Closest Point Algorithm](https://cs.gmu.edu/~kosecka/cs685/cs685-icp.pdf)


## FastSLAM 1.0

Exemplo de SLAM baseado em recursos usando o FastSLAM 1.0.

A linha azul(ground truth) é a linha que foi fornecida diretamente por observação e medição, a linha preta é o cálculo exato, a linha vermelha é a trajetória estimada com o algoritmo FastSLAM.

Os pontos vermelhos são partículas de FastSLAM.

Os pontos pretos são pontos de referência, as cruzes azuis são posições de referência estimadas pelo FastSLAM.


![3](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/SLAM/FastSLAM1/animation.gif)


Referências:

- [PROBABILISTIC ROBOTICS](http://www.probabilistic-robotics.org/)

- [SLAM simulations by Tim Bailey](http://www-personal.acfr.usyd.edu.au/tbailey/software/slam_simulations.htm)


# Planejamento de caminho

## Abordagem janela dinâmica

Código de exemplo para navegação 2D com abordagem de janela dinâmica.

- [The Dynamic Window Approach to Collision Avoidance](https://www.ri.cmu.edu/pub_files/pub1/fox_dieter_1997_1/fox_dieter_1997_1.pdf)

![2](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/DynamicWindowApproach/animation.gif)


## Busca baseada em grade

### Algoritmo de Dijkstra

Busca em grade 2D baseada no planejamento de caminho mais curto com o algoritmo de dijkstra.

![PythonRobotics/figure_1.png at master · AtsushiSakai/PythonRobotics](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/Dijkstra/animation.gif)

Na animação, os pontos cianos são nós pesquisados.

### Algoritmo A*

Esse é um exemplo de busca em grade 2D baseada no planejamento de caminho mais curto com o algoritmo A*(A-estrela).

![PythonRobotics/figure_1.png at master · AtsushiSakai/PythonRobotics](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/AStar/animation.gif)

Na animação, os pontos cianos são nós pesquisados.

Sua heurística é a distância Euclides 2D.

### Algoritmo D*

Esse é um exemplo de busca em grade 2D baseada no planejamento de caminho mais curto com o algoritmo D*(D-estrela).

![figure at master · nirnayroy/intelligentrobotics](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/DStar/animation.gif)

A animação mostra um robô encontrando seeu caminho e desviando dos obstáculos usando o algoritmo de busca D*.

Referência:

- [D* Algorithm Wikipedia](https://en.wikipedia.org/wiki/D*)

### Algoritmo D* Lite

Esse algoritmo encontra o caminho mais curto entre dois pontos enquanto também descobre obstáculos.Foi implementado para um grade 2D.

![D* Lite](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/DStarLite/animation.gif)

A animação mostra um robô encontrando seu caminho e redirecionando para evitar obstáculos à medida que são descobertos usando o algoritmo D* Lite.

Referências:

- [D* Lite](http://idm-lab.org/bib/abstracts/papers/aaai02b.pd)
- [Improved Fast Replanning for Robot Navigation in Unknown Terrain](http://www.cs.cmu.edu/~maxim/files/dlite_icra02.pdf)

### Algoritmo potencial de campo

Planejamento de caminho com o algoritmo potencial de campo.

![PotentialField](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/PotentialFieldPlanning/animation.gif)

Na animação, o mapa de calor azul mostra o valor do potencial em cada grade.

Referência:

- [Robotic Motion Planning:Potential Functions](https://www.cs.cmu.edu/~motionplanning/lecture/Chap4-Potential-Field_howie.pdf)

### Planejamento de caminho de cobertura baseado em grade

Simulação de planejamento de caminho de cobertura baseada em grade.

![PotentialField](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/GridBasedSweepCPP/animation.gif)

## Planejamento por rede de estado

Esse script é um código de planejamento de caminho por rede de estado.

Esse código usa o gerador de trajetória preditiva do modelo para resolver o problema da fronteira.

Referências: 

- [Optimal rough terrain trajectory generation for wheeled mobile robots](http://journals.sagepub.com/doi/pdf/10.1177/0278364906075328)

- [State Space Sampling of Feasible Motions for High-Performance Mobile Robot Navigation in Complex Environments](http://www.frc.ri.cmu.edu/~alonzo/pubs/papers/JFR_08_SS_Sampling.pdf)


### Amostragem polar tendenciosa

![PythonRobotics/figure_1.png at master · AtsushiSakai/PythonRobotics](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/StateLatticePlanner/BiasedPolarSampling.gif)


### Amostragem de caminho

![PythonRobotics/figure_1.png at master · AtsushiSakai/PythonRobotics](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/StateLatticePlanner/LaneSampling.gif)

## Planejamento de caminho probabílistico (PRM)

![PRM](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/ProbabilisticRoadMap/animation.gif)

Esse algoritmo (PRM) usa o método dijkstra para pesquisa em grafos.

Na animação os pontos azuis são pontos de amostra,

As cruzes em ciano representam os pontos pesquisados pelo método de dijkstra,

A linha vermelha é o caminho final do PRM.

Referência:

- [Probabilistic roadmap \- Wikipedia](https://en.wikipedia.org/wiki/Probabilistic_roadmap)

　　

## Árvores aleatórias de exploração rápida (RRT)

### RRT\*

![PythonRobotics/figure_1.png at master · AtsushiSakai/PythonRobotics](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/RRTstar/animation.gif)

Código de planejamento de caminho usando o algoritmo RRT\*

Os círculos pretos são obstáculos, a linha verde é uma árvore pesquisada, as cruzes vermelhas são as posições inicial e final.

Referências:

- [Incremental Sampling-based Algorithms for Optimal Motion Planning](https://arxiv.org/abs/1005.0416)

- [Sampling-based Algorithms for Optimal Motion Planning](http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.419.5503&rep=rep1&type=pdf)

### RRT\* com trajetória reeds-shep

![Robotics/animation.gif at master · AtsushiSakai/PythonRobotics](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/RRTStarReedsShepp/animation.gif))

Planejamento de caminho para um carro robô usando RRT* e reeds-shepp.

### LQR-RRT\*

Simulação de planejamento de caminho usando LQR-RRT\*.

Um modelo de movimento integrador duplo é usado para o planejador local LQR.

![LQR_RRT](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/LQRRRTStar/animation.gif)

Referências:

- [LQR\-RRT\*: Optimal Sampling\-Based Motion Planning with Automatically Derived Extension Heuristics](http://lis.csail.mit.edu/pubs/perez-icra12.pdf)

- [MahanFathi/LQR\-RRTstar: LQR\-RRT\* method is used for random motion planning of a simple pendulum in its phase plot](https://github.com/MahanFathi/LQR-RRTstar)


## Planejamento de polinômio de quinta ordem

Planejamento de movimento com polinômio de ordem 5.

![2](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/QuinticPolynomialsPlanner/animation.gif)

Ele pode calcular um caminho 2D, velocidade e perfil de aceleração com base em polinômios de grau 5.

Referência:

- [Local Path Planning And Motion Control For Agv In Positioning](http://ieeexplore.ieee.org/document/637936/)

## Planejamento de caminho reeds-shepp

Código de exemplo com o algoritmo reeds-shepp.

![RSPlanning](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/ReedsSheppPath/animation.gif?raw=true)

Referências:

- [15.3.2 Reeds\-Shepp Curves](http://planning.cs.uiuc.edu/node822.html) 

- [optimal paths for a car that goes both forwards and backwards](https://pdfs.semanticscholar.org/932e/c495b1d0018fd59dee12a0bf74434fac7af4.pdf)

- [ghliu/pyReedsShepp: Implementation of Reeds Shepp curve\.](https://github.com/ghliu/pyReedsShepp)


## Planejamento de caminho baseado em LQR

Um código de exemplo usando o planejamento de caminho baseado em LQR para o modelo de integrador duplo.

![RSPlanning](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/LQRPlanner/animation.gif?raw=true)


## Trajetória ótima em um Frenet Frame 

![3](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/FrenetOptimalTrajectory/animation.gif)

Determinação de uma trajetória ótima em um Frenet Frame.

A linha ciano é o percurso alvo e as cruzes pretas são obstáculos.

A linha vermelham é o caminho previsto.

Referências:

- [Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame](https://www.researchgate.net/profile/Moritz_Werling/publication/224156269_Optimal_Trajectory_Generation_for_Dynamic_Street_Scenarios_in_a_Frenet_Frame/links/54f749df0cf210398e9277af.pdf)

- [Optimal trajectory generation for dynamic street scenarios in a Frenet Frame](https://www.youtube.com/watch?v=Cj6tAQe7UCY)


# Rastreamento de caminho

## Mover para uma posição de controle

Simulação de movimento para um ponto de controle.

![2](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathTracking/move_to_pose/animation.gif)

Referência:

- [P. I. Corke, "Robotics, Vision and Control" \| SpringerLink p102](https://link.springer.com/book/10.1007/978-3-642-20144-8)


## Controle Stanley

Simulação de rastreamento de caminho com controle de direção Stanley e controle de velocidade PID.

![2](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathTracking/stanley_controller/animation.gif)

Referências:

- [Stanley: The robot that won the DARPA grand challenge](http://robots.stanford.edu/papers/thrun.stanley05.pdf)

- [Automatic Steering Methods for Autonomous Automobile Path Tracking](https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf)



## Controle de feedback da roda traseira

Simulação de rastreamento de caminho com controle de direção de feedback da roda traseira e controle de velocidade PID.

![PythonRobotics/figure_1.png at master · AtsushiSakai/PythonRobotics](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathTracking/rear_wheel_feedback/animation.gif)

Referência:

- [A Survey of Motion Planning and Control Techniques for Self-driving Urban Vehicles](https://arxiv.org/abs/1604.07446)


## Controle de velocidade e direção usando LQR(linear-quadratic regulator)

Simulação de rastreamento de caminho com velocidade LQR e controle de direção.

![3](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathTracking/lqr_speed_steer_control/animation.gif)

Referência:

- [Towards fully autonomous driving: Systems and algorithms \- IEEE Conference Publication](http://ieeexplore.ieee.org/document/5940562/)


## Modelo preditivo de velocidade e controle de posição

Simulação de rastreamento de caminho com velocidade preditiva de modelo linear iterativo e controle de direção.

<img src="https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathTracking/model_predictive_speed_and_steer_control/animation.gif" width="640" alt="MPC pic">

Referências:

- [notebook](https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathTracking/model_predictive_speed_and_steer_control/Model_predictive_speed_and_steering_control.ipynb)

- [Real\-time Model Predictive Control \(MPC\), ACADO, Python \| Work\-is\-Playing](http://grauonline.de/wordpress/?page_id=3244)

## Modelo preditivo não linear de controle com C-GMRES

Planejamento de movimento e rastreamento de caminho usando NMPC de C-GMRES

![3](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathTracking/cgmres_nmpc/animation.gif)

Referência:

- [notebook](https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathTracking/cgmres_nmpc/cgmres_nmpc.ipynb)


# Movimento de braço robótico

## Número N de juntas até um ponto de controle

 Simulação de um número N de juntas até um ponto de controle

Essa é uma simulção interativa.

Você pode definir a posição final com o botão esquerdo do mouse na área de plotagem.. 

![3](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/ArmNavigation/n_joint_arm_to_point_control/animation.gif)

Nessa simulação  N = 10, no entanto você pode alterá-lo..

## Movimento de braço robótico com obstáculo

Simualção de movimento de braço robótico com obstáculo

![3](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/ArmNavigation/arm_obstacle_navigation/animation.gif)


# Navegação aérea

## Drone se movendo em plano tridimensional(3D) 

Esta é uma trajetória 3d seguindo a simulação para um quadrotor.

![3](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/AerialNavigation/drone_3d_trajectory_following/animation.gif)

## Pouso de um foguete

Essa trajetória 3D simula um pouso de um foguete.

![3](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/AerialNavigation/rocket_powered_landing/animation.gif)

Referência:

- [notebook](https://github.com/AtsushiSakai/PythonRobotics/blob/master/AerialNavigation/rocket_powered_landing/rocket_powered_landing.ipynb)

#  Robôs Bípedes

## Planejamento de movimento bípede com pêndulo invertido

Este é um planejador de movimento para robôs bípedes usando um modificador de  passos para um pêndulo invertido.

Você pode definir os passos e o planejador os modificará automaticamente.

![3](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/Bipedal/bipedal_planner/animation.gif)

# Licença

MIT

# Caso de Uso

Se de alguma esse projeto te ajudar, seria um prazer saber como, entre em contato criando uma issue.

O vídeo do seu robô que está usando o PythonRobotics, é muito bem vindo!!

Essa é uma lista de comentários e referências dos usuários:[users\_comments](https://github.com/AtsushiSakai/PythonRobotics/blob/master/users_comments.md)

# Contribuição

Qualquer contribuição é bem-vinda!! 

Por favor leia esse documento:[Como contribuir: — PythonRobotics documentação](https://atsushisakai.github.io/PythonRobotics/how_to_contribute.html)

# Citando o Projeto

Caso usar o código deste projeto para seu trabalho acadêmico, recomendamos que você cite [nossos artigos](https://arxiv.org/abs/1808.10703) 

Se usar o código deste projeto na indústria, adoraríamos ouvir você também; sinta-se à vontade para entrar em contato diretamente com os desenvolvedores.

# <a id="support"></a>Apoiando o projeto

Se você ou sua empresa gostaria de apoiar o projeto , por favor considere:

- [Patrocinar @AtsushiSakai no GitHub Patrocinadores](https://github.com/sponsors/AtsushiSakai)

- [Torne-se um apoiador ou patrocinador no Patreon](https://www.patreon.com/myenigma)

- [Doação via PayPal](https://www.paypal.me/myenigmapay/)

Se você gostaria de nos apoiar de uma outra forma, por favor entre em contato criando uma issue.

## <a id="sponsors"></a>Patrocinadores

### <a id="JetBrains"></a>[JetBrains](https://www.jetbrains.com/)

Eles estão fornecendo uma licença gratuita de suas IDEs para o desenvolvimento do projeto.   

### [1Password](https://github.com/1Password/1password-teams-open-source)

Eles estão fornecendo uma licença gratuita da 1Password teams para esse projeto.   


# Autores

- [Contribuidores do repositório AtsushiSakai/PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics/graphs/contributors)

