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
         * [Algoritmo campo potencial](#algoritmo-campo-potencial)
         * [Planejamento de caminho baseado em grade](#planejamento-de-caminho-baseado-em-grade)
      * [Planejamento por rede de estado](#planejamento-por-rede-de-estado)
         * [Amostragem polar tendenciosa](#amostragem-polar-tendenciosa)
         * [Amostragem de caminho](#amostragem-de-caminho)
      * [Planejador de caminho probabilístico](#planejador-de-caminho-probabilístico)
      * [Árvore aleatória de exploração rápida](#árvore-aleatória-de-exploração-rápida)
         * [RRT*](#rrt)
         * [RRT* com trajetória reeds-shepp](#rrt*-com-trajetória-reeds-shepp)
         * [LQR-RRT*](#lqr-rrt)
      * [Planejamento de polinômio de quinta ordem](#planejamento-de-polinômio-de-quinta-ordem)
      * [Planejamento de trajetória Reeds Shepp](#planejamento-de-trajetória-reeds-shepp)
      * [Planejamento de trajetória baseado em LQR](#planejamento-de-trajetória-baseado-em-lqr)
      * [Trajetória ótima em um Frenet Frame](#trajetória-ótima-em-um-frenet-frame)
   * [Rastreamento de Caminho](#reastramento-de-caminho)
      * [mover para uma posição de controle](#mover-para-uma-posição-de-controle)
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
   * [Bípedes](#bípedes)
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

busca em grade 2D baseada no planejamento de caminho mais curto com o algoritmo de dijkstra.

![PythonRobotics/figure_1.png at master · AtsushiSakai/PythonRobotics](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/Dijkstra/animation.gif)

Na animação, os pontos cianos são nós pesquisados.

### Algoritmo A*

This is a 2D grid based the shortest path planning with A star algorithm.

![PythonRobotics/figure_1.png at master · AtsushiSakai/PythonRobotics](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/AStar/animation.gif)

In the animation, cyan points are searched nodes.

Its heuristic is 2D Euclid distance.

### D\* algorithm

This is a 2D grid based the shortest path planning with D star algorithm.

![figure at master · nirnayroy/intelligentrobotics](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/DStar/animation.gif)

The animation shows a robot finding its path avoiding an obstacle using the D* search algorithm.

Referência:

- [D* Algorithm Wikipedia](https://en.wikipedia.org/wiki/D*)

### D\* Lite algorithm

This algorithm finds the shortest path between two points while rerouting when obstacles are discovered. It has been implemented here for a 2D grid.

![D* Lite](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/DStarLite/animation.gif)

The animation shows a robot finding its path and rerouting to avoid obstacles as they are discovered using the D* Lite search algorithm.

Referências:

- [D* Lite](http://idm-lab.org/bib/abstracts/papers/aaai02b.pd)
- [Improved Fast Replanning for Robot Navigation in Unknown Terrain](http://www.cs.cmu.edu/~maxim/files/dlite_icra02.pdf)

### Potential Field algorithm

This is a 2D grid based path planning with Potential Field algorithm.

![PotentialField](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/PotentialFieldPlanning/animation.gif)

In the animation, the blue heat map shows potential value on each grid.

Referência:

- [Robotic Motion Planning:Potential Functions](https://www.cs.cmu.edu/~motionplanning/lecture/Chap4-Potential-Field_howie.pdf)

### Grid based coverage path planning

This is a 2D grid based coverage path planning simulation.

![PotentialField](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/GridBasedSweepCPP/animation.gif)

## State Lattice Planning

This script is a path planning code with state lattice planning.

This code uses the model predictive trajectory generator to solve boundary problem.

Referências: 

- [Optimal rough terrain trajectory generation for wheeled mobile robots](http://journals.sagepub.com/doi/pdf/10.1177/0278364906075328)

- [State Space Sampling of Feasible Motions for High-Performance Mobile Robot Navigation in Complex Environments](http://www.frc.ri.cmu.edu/~alonzo/pubs/papers/JFR_08_SS_Sampling.pdf)


### Biased polar sampling

![PythonRobotics/figure_1.png at master · AtsushiSakai/PythonRobotics](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/StateLatticePlanner/BiasedPolarSampling.gif)


### Lane sampling

![PythonRobotics/figure_1.png at master · AtsushiSakai/PythonRobotics](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/StateLatticePlanner/LaneSampling.gif)

## Probabilistic Road-Map (PRM) planning 

![PRM](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/ProbabilisticRoadMap/animation.gif)

This PRM planner uses Dijkstra method for graph search.

In the animation, blue points are sampled points,

Cyan crosses means searched points with Dijkstra method,

The red line is the final path of PRM.

Referência:

- [Probabilistic roadmap \- Wikipedia](https://en.wikipedia.org/wiki/Probabilistic_roadmap)

　　

## Rapidly-Exploring Random Trees (RRT)

### RRT\*

![PythonRobotics/figure_1.png at master · AtsushiSakai/PythonRobotics](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/RRTstar/animation.gif)

This is a path planning code with RRT\*

Black circles are obstacles, green line is a searched tree, red crosses are start and goal positions.

Referência:

- [Incremental Sampling-based Algorithms for Optimal Motion Planning](https://arxiv.org/abs/1005.0416)

- [Sampling-based Algorithms for Optimal Motion Planning](http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.419.5503&rep=rep1&type=pdf)

### RRT\* with reeds-shepp path

![Robotics/animation.gif at master · AtsushiSakai/PythonRobotics](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/RRTStarReedsShepp/animation.gif))

Path planning for a car robot with RRT\* and reeds shepp path planner.

### LQR-RRT\*

This is a path planning simulation with LQR-RRT\*.

A double integrator motion model is used for LQR local planner.

![LQR_RRT](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/LQRRRTStar/animation.gif)

Referência:

- [LQR\-RRT\*: Optimal Sampling\-Based Motion Planning with Automatically Derived Extension Heuristics](http://lis.csail.mit.edu/pubs/perez-icra12.pdf)

- [MahanFathi/LQR\-RRTstar: LQR\-RRT\* method is used for random motion planning of a simple pendulum in its phase plot](https://github.com/MahanFathi/LQR-RRTstar)


## Quintic polynomials planning

Motion planning with quintic polynomials.

![2](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/QuinticPolynomialsPlanner/animation.gif)

It can calculate a 2D path, velocity, and acceleration profile based on quintic polynomials.

Referência:

- [Local Path Planning And Motion Control For Agv In Positioning](http://ieeexplore.ieee.org/document/637936/)

## Reeds Shepp planning

A sample code with Reeds Shepp path planning.

![RSPlanning](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/ReedsSheppPath/animation.gif?raw=true)

Referência:

- [15.3.2 Reeds\-Shepp Curves](http://planning.cs.uiuc.edu/node822.html) 

- [optimal paths for a car that goes both forwards and backwards](https://pdfs.semanticscholar.org/932e/c495b1d0018fd59dee12a0bf74434fac7af4.pdf)

- [ghliu/pyReedsShepp: Implementation of Reeds Shepp curve\.](https://github.com/ghliu/pyReedsShepp)


## LQR based path planning

A sample code using LQR based path planning for double integrator model.

![RSPlanning](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/LQRPlanner/animation.gif?raw=true)


## Optimal Trajectory in a Frenet Frame 

![3](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/FrenetOptimalTrajectory/animation.gif)

This is optimal trajectory generation in a Frenet Frame.

The cyan line is the target course and black crosses are obstacles.

The red line is the predicted path.

Referência:

- [Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame](https://www.researchgate.net/profile/Moritz_Werling/publication/224156269_Optimal_Trajectory_Generation_for_Dynamic_Street_Scenarios_in_a_Frenet_Frame/links/54f749df0cf210398e9277af.pdf)

- [Optimal trajectory generation for dynamic street scenarios in a Frenet Frame](https://www.youtube.com/watch?v=Cj6tAQe7UCY)


# Path Tracking

## move to a pose control

This is a simulation of moving to a pose control

![2](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathTracking/move_to_pose/animation.gif)

Referência:

- [P. I. Corke, "Robotics, Vision and Control" \| SpringerLink p102](https://link.springer.com/book/10.1007/978-3-642-20144-8)


## Stanley control

Path tracking simulation with Stanley steering control and PID speed control.

![2](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathTracking/stanley_controller/animation.gif)

Referência:

- [Stanley: The robot that won the DARPA grand challenge](http://robots.stanford.edu/papers/thrun.stanley05.pdf)

- [Automatic Steering Methods for Autonomous Automobile Path Tracking](https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf)



## Rear wheel feedback control

Path tracking simulation with rear wheel feedback steering control and PID speed control.

![PythonRobotics/figure_1.png at master · AtsushiSakai/PythonRobotics](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathTracking/rear_wheel_feedback/animation.gif)

Referência:

- [A Survey of Motion Planning and Control Techniques for Self-driving Urban Vehicles](https://arxiv.org/abs/1604.07446)


## Linear–quadratic regulator (LQR) speed and steering control

Path tracking simulation with LQR speed and steering control.

![3](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathTracking/lqr_speed_steer_control/animation.gif)

Referência:

- [Towards fully autonomous driving: Systems and algorithms \- IEEE Conference Publication](http://ieeexplore.ieee.org/document/5940562/)


## Model predictive speed and steering control

Path tracking simulation with iterative linear model predictive speed and steering control.

<img src="https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathTracking/model_predictive_speed_and_steer_control/animation.gif" width="640" alt="MPC pic">

Referência:

- [notebook](https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathTracking/model_predictive_speed_and_steer_control/Model_predictive_speed_and_steering_control.ipynb)

- [Real\-time Model Predictive Control \(MPC\), ACADO, Python \| Work\-is\-Playing](http://grauonline.de/wordpress/?page_id=3244)

## Nonlinear Model predictive control with C-GMRES

A motion planning and path tracking simulation with NMPC of C-GMRES 

![3](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathTracking/cgmres_nmpc/animation.gif)

Referência:

- [notebook](https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathTracking/cgmres_nmpc/cgmres_nmpc.ipynb)


# Arm Navigation

## N joint arm to point control

N joint arm to a point control simulation.

This is an interactive simulation.

You can set the goal position of the end effector with left-click on the plotting area. 

![3](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/ArmNavigation/n_joint_arm_to_point_control/animation.gif)

In this simulation N = 10, however, you can change it.

## Arm navigation with obstacle avoidance 

Arm navigation with obstacle avoidance simulation.

![3](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/ArmNavigation/arm_obstacle_navigation/animation.gif)


# Navegação aérea

## drone 3d trajectory following 

This is a 3d trajectory following simulation for a quadrotor.

![3](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/AerialNavigation/drone_3d_trajectory_following/animation.gif)

## rocket powered landing

This is a 3d trajectory generation simulation for a rocket powered landing.

![3](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/AerialNavigation/rocket_powered_landing/animation.gif)

Ref:

- [notebook](https://github.com/AtsushiSakai/PythonRobotics/blob/master/AerialNavigation/rocket_powered_landing/rocket_powered_landing.ipynb)

# Bipedal

## bipedal planner with inverted pendulum

This is a bipedal planner for modifying footsteps for an inverted pendulum.

You can set the footsteps, and the planner will modify those automatically.

![3](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/Bipedal/bipedal_planner/animation.gif)

# Licença

MIT

# Use-case

If this project helps your robotics project, please let me know with creating an issue.

Your robot's video, which is using PythonRobotics, is very welcome!!

This is a list of user's comment and references:[users\_comments](https://github.com/AtsushiSakai/PythonRobotics/blob/master/users_comments.md)

# Contribuição

Qualquer contribuição é bem-vinda!! 

Por favor leia esse documento:[Como contribuir: — PythonRobotics documentação](https://atsushisakai.github.io/PythonRobotics/how_to_contribute.html)

# Citando o Projeto

If you use this project's code for your academic work, we encourage you to cite [our papers](https://arxiv.org/abs/1808.10703) 

If you use this project's code in industry, we'd love to hear from you as well; feel free to reach out to the developers directly.

# <a id="support"></a>Supporting this project

If you or your company would like to support this project, please consider:

- [Sponsor @AtsushiSakai on GitHub Sponsors](https://github.com/sponsors/AtsushiSakai)

- [Become a backer or sponsor on Patreon](https://www.patreon.com/myenigma)

- [One-time donation via PayPal](https://www.paypal.me/myenigmapay/)

Se você gostaria de nos apoiar de uma outra forma, por favor entre em contato criando uma issue.

## <a id="sponsors"></a>Patrocinadores

### <a id="JetBrains"></a>[JetBrains](https://www.jetbrains.com/)

They are providing a free license of their IDEs for this OSS development.   

### [1Password](https://github.com/1Password/1password-teams-open-source)

They are providing a free license of their 1Password team license for this OSS project.   


# Autores

- [Contribuidores do repositório AtsushiSakai/PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics/graphs/contributors)

