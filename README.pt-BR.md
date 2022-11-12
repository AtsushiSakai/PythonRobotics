<img src="https://github.com/AtsushiSakai/PythonRobotics/raw/master/icon.png?raw=true" align="right" width="300" alt="header pic"/>

# PythonRobotics
![GitHub_Action_Linux_CI](https://github.com/AtsushiSakai/PythonRobotics/workflows/Linux_CI/badge.svg)
![GitHub_Action_MacOS_CI](https://github.com/AtsushiSakai/PythonRobotics/workflows/MacOS_CI/badge.svg)
[![Build status](https://ci.appveyor.com/api/projects/status/sb279kxuv1be391g?svg=true)](https://ci.appveyor.com/project/AtsushiSakai/pythonrobotics)
[![codecov](https://codecov.io/gh/AtsushiSakai/PythonRobotics/branch/master/graph/badge.svg)](https://codecov.io/gh/AtsushiSakai/PythonRobotics)
[![Language grade: Python](https://img.shields.io/lgtm/grade/python/g/AtsushiSakai/PythonRobotics.svg?logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/AtsushiSakai/PythonRobotics/context:python)
[![tokei](https://tokei.rs/b1/github/AtsushiSakai/PythonRobotics)](https://github.com/AtsushiSakai/PythonRobotics)

Python codes for robotics algorithm.


# Table of Contents
   * [Oque é isso??](#Oque-é-isso)
   * [Requirements](#requirements)
   * [Documentation](#documentation)
   * [How to use](#how-to-use)
   * [Localization](#localization)
      * [Extended Kalman Filter localization](#extended-kalman-filter-localization)
      * [Particle filter localization](#particle-filter-localization)
      * [Histogram filter localization](#histogram-filter-localization)
   * [Mapping](#mapping)
      * [Gaussian grid map](#gaussian-grid-map)
      * [Ray casting grid map](#ray-casting-grid-map)
      * [Lidar to grid map](#lidar-to-grid-map)
      * [k-means object clustering](#k-means-object-clustering)
      * [Rectangle fitting](#rectangle-fitting)
   * [SLAM](#slam)
      * [Iterative Closest Point (ICP) Matching](#iterative-closest-point-icp-matching)
      * [FastSLAM 1.0](#fastslam-10)
   * [Path Planning](#path-planning)
      * [Dynamic Window Approach](#dynamic-window-approach)
      * [Grid based search](#grid-based-search)
         * [Dijkstra algorithm](#dijkstra-algorithm)
         * [A* algorithm](#a-algorithm)
         * [D* algorithm](#d-algorithm)
         * [D* Lite algorithm](#d-lite-algorithm)
         * [Potential Field algorithm](#potential-field-algorithm)
         * [Grid based coverage path planning](#grid-based-coverage-path-planning)
      * [State Lattice Planning](#state-lattice-planning)
         * [Biased polar sampling](#biased-polar-sampling)
         * [Lane sampling](#lane-sampling)
      * [Probabilistic Road-Map (PRM) planning](#probabilistic-road-map-prm-planning)
      * [Rapidly-Exploring Random Trees (RRT)](#rapidly-exploring-random-trees-rrt)
         * [RRT*](#rrt)
         * [RRT* with reeds-shepp path](#rrt-with-reeds-shepp-path)
         * [LQR-RRT*](#lqr-rrt)
      * [Quintic polynomials planning](#quintic-polynomials-planning)
      * [Reeds Shepp planning](#reeds-shepp-planning)
      * [LQR based path planning](#lqr-based-path-planning)
      * [Optimal Trajectory in a Frenet Frame](#optimal-trajectory-in-a-frenet-frame)
   * [Path Tracking](#path-tracking)
      * [move to a pose control](#move-to-a-pose-control)
      * [Stanley control](#stanley-control)
      * [Rear wheel feedback control](#rear-wheel-feedback-control)
      * [Linear–quadratic regulator (LQR) speed and steering control](#linearquadratic-regulator-lqr-speed-and-steering-control)
      * [Model predictive speed and steering control](#model-predictive-speed-and-steering-control)
      * [Nonlinear Model predictive control with C-GMRES](#nonlinear-model-predictive-control-with-c-gmres)
   * [Arm Navigation](#arm-navigation)
      * [N joint arm to point control](#n-joint-arm-to-point-control)
      * [Arm navigation with obstacle avoidance](#arm-navigation-with-obstacle-avoidance)
   * [Aerial Navigation](#aerial-navigation)
      * [drone 3d trajectory following](#drone-3d-trajectory-following)
      * [rocket powered landing](#rocket-powered-landing)
   * [Bipedal](#bipedal)
      * [bipedal planner with inverted pendulum](#bipedal-planner-with-inverted-pendulum)
   * [License](#license)
   * [Use-case](#use-case)
   * [Contribution](#contribution)
   * [Citing](#citing)
   * [Support](#support)
   * [Sponsors](#sponsors)
      * [JetBrains](#JetBrains)
      * [1Password](#1password)
   * [Authors](#authors)

# Oque é isso?

Isso é uma coleção de códigos em Python de algorítimos de robótica.

Recursos:

1. Leitura de fácil compreensão da ideia básica de cada algorítimo.

2. Algoritmos amplamente utilizados e práticos são selecionados.

3. Dependência mínima.

Veja este documento para mais detalhes:

- [\[1808\.10703\] PythonRobotics: a Python code collection of robotics algorithms](https://arxiv.org/abs/1808.10703) ([BibTeX](https://github.com/AtsushiSakai/PythonRoboticsPaper/blob/master/python_robotics.bib))


# Requerimentos

Para executar cada código de amostra:

- [Python 3.10.x](https://www.python.org/)
 
- [NumPy](https://numpy.org/)
 
- [SciPy](https://scipy.org/)
 
- [Matplotlib](https://matplotlib.org/)
 
- [cvxpy](https://www.cvxpy.org/) 

Para desenvolvimento:
  
- [pytest](https://pytest.org/) (para testes de unidade)
  
- [pytest-xdist](https://pypi.org/project/pytest-xdist/) (para testes unitários paralelos)
  
- [mypy](http://mypy-lang.org/) (para verificação de tipo)
  
- [sphinx](https://www.sphinx-doc.org/) (para geração de documentação)
  
- [pycodestyle](https://pypi.org/project/pycodestyle/) (para verificação de estilo de código)

# Documentação

Este README mostra apenas alguns exemplos deste projeto. 

Se você estiver interessado em outros exemplos ou fundamentos matemáticos de cada algoritmo,

Você pode verificar a documentação completa online: [Welcome to PythonRobotics’s documentation\! — PythonRobotics documentation](https://atsushisakai.github.io/PythonRobotics/index.html)

Todos os gifs de animação são armazenados aqui: [AtsushiSakai/PythonRoboticsGifs: Animation gifs of PythonRobotics](https://github.com/AtsushiSakai/PythonRoboticsGifs)

# Como Utilizar

1. Clone esse repositório.

   ```terminal
   git clone https://github.com/AtsushiSakai/PythonRobotics.git
   ```


2. Instale as bibliotecas necessárias.

- using conda :

  ```terminal
  conda env create -f requirements/environment.yml
  ```
 
- using pip :

  ```terminal
  pip install -r requirements/requirements.txt
  ```


3. Execute o script python em cada diretório.

4. Adicione estrela a este repositório se você gostar:smiley:. 

# Localização

## Localização estendida do filtro Kalman

<img src="https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/Localization/extended_kalman_filter/animation.gif" width="640" alt="EKF pic">

Documentation: [Notebook](https://github.com/AtsushiSakai/PythonRobotics/blob/master/Localization/extended_kalman_filter/extended_kalman_filter_localization.ipynb)

## Localização do filtro de partículas

![2](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/Localization/particle_filter/animation.gif)

Esta é uma localização de fusão de sensores com Filtro de Partículas(PF).

A linha azul é a trajetória verdadeira, a linha preta é a trajetória de cálculo morto,

e a linha vermelha é uma trajetória estimada com PF.

Assume-se que o robô pode medir uma distância de pontos de referência (RFID).

Essas medidas são usadas para localização do PF.

Ref:

- [PROBABILISTIC ROBOTICS](http://www.probabilistic-robotics.org/)


## Localização do filtro de histograma

![3](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/Localization/histogram_filter/animation.gif)

Este é um exemplo de localização 2D com filtro Histograma.

A cruz vermelha é a posição verdadeira, os pontos pretos são as posições RFID.

A grade azul mostra uma probabilidade de posição do filtro de histograma.

Nesta simulação, x,y são desconhecidos, yaw é conhecido.

O filtro integra a entrada de velocidade e observações de alcance de RFID para localização.

A posição inicial não é necessária.

Ref:

- [PROBABILISTIC ROBOTICS](http://www.probabilistic-robotics.org/)

# Mapeamento

## Mapa de grade gaussiana

Este é um exemplo de mapeamento de grade gaussiana 2D.

![2](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/Mapping/gaussian_grid_map/animation.gif)

## Mapa de grade de transmissão de raios

Este é um exemplo de mapeamento de grade de projeção de raios 2D.

![2](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/Mapping/raycasting_grid_map/animation.gif)

## Lidar para mapa de grade

Este exemplo mostra como converter uma medição de intervalo 2D em um mapa de grade.

![2](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/Mapping/lidar_to_grid_map/animation.gif)

## agrupamento de objetos k-means

Este é um cluster de objetos 2D com algoritmo k-means.

![2](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/Mapping/kmeans_clustering/animation.gif)

## Encaixe retangular

Este é um retângulo 2D adequado para detecção de veículos.

![2](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/Mapping/rectangle_fitting/animation.gif)


# SLAM

Exemplos de localização e mapeamento simultâneos (SLAM)

## Correspondência de ponto mais próximo iterativo (ICP)

Este é um exemplo de correspondência de ICP 2D com decomposição de valor singular.

Ele pode calcular uma matriz de rotação e um vetor de translação entre pontos e pontos.

![3](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/SLAM/iterative_closest_point/animation.gif)

Ref:

- [Introduction to Mobile Robotics: Iterative Closest Point Algorithm](https://cs.gmu.edu/~kosecka/cs685/cs685-icp.pdf)


## FastSLAM 1.0

Este é um exemplo de SLAM baseado em recursos usando FastSLAM 1.0.

A linha azul é a verdade do solo, a linha preta é o cálculo exato, a linha vermelha é a trajetória estimada com FastSLAM.

Os pontos vermelhos são partículas de FastSLAM.

Pontos pretos são pontos de referência, cruzes azuis são posições de referência estimadas pelo FastSLAM.


![3](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/SLAM/FastSLAM1/animation.gif)


Ref:

- [PROBABILISTIC ROBOTICS](http://www.probabilistic-robotics.org/)

- [SLAM simulations by Tim Bailey](http://www-personal.acfr.usyd.edu.au/tbailey/software/slam_simulations.htm)


# Planejamento de caminho

## Abordagem de janela dinâmica

Este é um código de amostra de navegação 2D com abordagem de janela dinâmica.

- [The Dynamic Window Approach to Collision Avoidance](https://www.ri.cmu.edu/pub_files/pub1/fox_dieter_1997_1/fox_dieter_1997_1.pdf)

![2](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/DynamicWindowApproach/animation.gif)


## Pesquisa baseada em grade

### Algoritmo de Dijkstra

Esta é uma grade 2D baseada no planejamento do caminho mais curto com o algoritmo de Dijkstra.

![PythonRobotics/figure_1.png at master · AtsushiSakai/PythonRobotics](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/Dijkstra/animation.gif)

Na animação, os pontos ciano são nós pesquisados.

### algorítimo A\*  

Esta é uma grade 2D baseada no planejamento do caminho mais curto com um algoritmo de estrela.

![PythonRobotics/figure_1.png at master · AtsushiSakai/PythonRobotics](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/AStar/animation.gif)

Na animação, os pontos ciano são nós pesquisados.

Sua heurística é a distância Euclides 2D.

###  Algoritmo D\* 

Esta é uma grade 2D baseada no planejamento do caminho mais curto com o algoritmo D star.

![figure at master · nirnayroy/intelligentrobotics](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/DStar/animation.gif)

A animação mostra um robô encontrando seu caminho evitando um obstáculo usando o algoritmo de busca D*.

Ref:

- [D* Algorithm Wikipedia](https://en.wikipedia.org/wiki/D*)

### Algoritmo leve D\*

Este algoritmo encontra o caminho mais curto entre dois pontos durante o reencaminhamento quando os obstáculos são descobertos. Foi implementado aqui para uma grade 2D.

![D* Lite](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/DStarLite/animation.gif)

A animação mostra um robô encontrando seu caminho e redirecionando para evitar obstáculos à medida que são descobertos usando o algoritmo de busca D* Lite.

Refs:

- [D* Lite](http://idm-lab.org/bib/abstracts/papers/aaai02b.pd)
- [Improved Fast Replanning for Robot Navigation in Unknown Terrain](http://www.cs.cmu.edu/~maxim/files/dlite_icra02.pdf)

### Algoritmo de campo potencial

Este é um planejamento de caminho baseado em grade 2D com algoritmo de campo potencial.

![PotentialField](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/PotentialFieldPlanning/animation.gif)

Na animação, o mapa de calor azul mostra o valor potencial em cada grade.

Ref:

- [Robotic Motion Planning:Potential Functions](https://www.cs.cmu.edu/~motionplanning/lecture/Chap4-Potential-Field_howie.pdf)

### Planejamento de caminho de cobertura baseado em grade

Esta é uma simulação de planejamento de caminho de cobertura baseada em grade 2D.

![PotentialField](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/GridBasedSweepCPP/animation.gif)

## Planejamento de rede estadual

Este script é um código de planejamento de caminho com planejamento de rede de estado.

Este código usa o gerador de trajetória preditiva do modelo para resolver o problema de fronteira.

Ref: 

- [Optimal rough terrain trajectory generation for wheeled mobile robots](http://journals.sagepub.com/doi/pdf/10.1177/0278364906075328)

- [State Space Sampling of Feasible Motions for High-Performance Mobile Robot Navigation in Complex Environments](http://www.frc.ri.cmu.edu/~alonzo/pubs/papers/JFR_08_SS_Sampling.pdf)


### Amostragem polar tendenciosa

![PythonRobotics/figure_1.png at master · AtsushiSakai/PythonRobotics](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/StateLatticePlanner/BiasedPolarSampling.gif)


### Amostragem de pista

![PythonRobotics/figure_1.png at master · AtsushiSakai/PythonRobotics](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/StateLatticePlanner/LaneSampling.gif)

## Planejamento de roteiro probabilístico (PRM)

![PRM](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/ProbabilisticRoadMap/animation.gif)

Este planejador de PRM usa o método Dijkstra para pesquisa de gráficos.

Na animação, os pontos azuis são pontos amostrados,

Cyan crosses significa pontos pesquisados ​​com o método Dijkstra,

A linha vermelha é o caminho final do PRM.

Ref:

- [Probabilistic roadmap \- Wikipedia](https://en.wikipedia.org/wiki/Probabilistic_roadmap)

　　

## Árvores Aleatórias de Exploração Rápida (RRT)

### RRT\*

![PythonRobotics/figure_1.png at master · AtsushiSakai/PythonRobotics](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/RRTstar/animation.gif)

Este é um código de planejamento de caminho com RRT\*

Os círculos pretos são obstáculos, a linha verde é uma árvore pesquisada, as cruzes vermelhas são as posições inicial e final.

Ref:

- [Incremental Sampling-based Algorithms for Optimal Motion Planning](https://arxiv.org/abs/1005.0416)

- [Sampling-based Algorithms for Optimal Motion Planning](http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.419.5503&rep=rep1&type=pdf)

### RRT\* com caminho reeds-shepp

![Robotics/animation.gif at master · AtsushiSakai/PythonRobotics](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/RRTStarReedsShepp/animation.gif))

Planejamento de caminho para um robô de carro com RRT\* e planejador de caminho de palhetas shepp.

### LQR-RRT\*

Esta é uma simulação de planejamento de caminho com LQR-RRT\*.

Um modelo de movimento integrador duplo é usado para o planejador local LQR.

![LQR_RRT](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/LQRRRTStar/animation.gif)

Ref:

- [LQR\-RRT\*: Optimal Sampling\-Based Motion Planning with Automatically Derived Extension Heuristics](http://lis.csail.mit.edu/pubs/perez-icra12.pdf)

- [MahanFathi/LQR\-RRTstar: LQR\-RRT\* method is used for random motion planning of a simple pendulum in its phase plot](https://github.com/MahanFathi/LQR-RRTstar)


## Planejamento de polinômios quínticos

Planejamento de movimento com polinômios quínticos.

![2](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/QuinticPolynomialsPlanner/animation.gif)

Ele pode calcular um caminho 2D, velocidade e perfil de aceleração com base em polinômios quínticos.
Ref:

- [Local Path Planning And Motion Control For Agv In Positioning](http://ieeexplore.ieee.org/document/637936/)

## Planejamento de Reeds Shepp

Um código de exemplo com o planejamento de caminho de Reeds Shepp.

![RSPlanning](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/ReedsSheppPath/animation.gif?raw=true)

Ref:

- [15.3.2 Reeds\-Shepp Curves](http://planning.cs.uiuc.edu/node822.html) 

- [optimal paths for a car that goes both forwards and backwards](https://pdfs.semanticscholar.org/932e/c495b1d0018fd59dee12a0bf74434fac7af4.pdf)

- [ghliu/pyReedsShepp: Implementation of Reeds Shepp curve\.](https://github.com/ghliu/pyReedsShepp)


## Planejamento de caminho baseado em LQR

Um código de amostra usando o planejamento de caminho baseado em LQR para o modelo de integrador duplo.

![RSPlanning](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/LQRPlanner/animation.gif?raw=true)


## Trajetória ótima em um quadro Frenet 

![3](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathPlanning/FrenetOptimalTrajectory/animation.gif)

Esta é a geração de trajetória ideal em um quadro Frenet.

A linha ciano é o curso alvo e as cruzes pretas são obstáculos.

A linha vermelha é o caminho previsto.

Ref:

- [Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame](https://www.researchgate.net/profile/Moritz_Werling/publication/224156269_Optimal_Trajectory_Generation_for_Dynamic_Street_Scenarios_in_a_Frenet_Frame/links/54f749df0cf210398e9277af.pdf)

- [Optimal trajectory generation for dynamic street scenarios in a Frenet Frame](https://www.youtube.com/watch?v=Cj6tAQe7UCY)


# Rastreamento de caminho

## mover para um controle de pose

Esta é uma simulação de mover para um controle de pose

![2](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathTracking/move_to_pose/animation.gif)

Ref:

- [P. I. Corke, "Robotics, Vision and Control" \| SpringerLink p102](https://link.springer.com/book/10.1007/978-3-642-20144-8)


## Controle Stanley

Simulação de rastreamento de caminho com controle de direção Stanley e controle de velocidade PID.

![2](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathTracking/stanley_controller/animation.gif)

Ref:

- [Stanley: The robot that won the DARPA grand challenge](http://robots.stanford.edu/papers/thrun.stanley05.pdf)

- [Automatic Steering Methods for Autonomous Automobile Path Tracking](https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf)



## Controle de feedback da roda traseira

Simulação de rastreamento de caminho com controle de direção de feedback da roda traseira e controle de velocidade PID.

![PythonRobotics/figure_1.png at master · AtsushiSakai/PythonRobotics](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathTracking/rear_wheel_feedback/animation.gif)

Ref:

- [A Survey of Motion Planning and Control Techniques for Self-driving Urban Vehicles](https://arxiv.org/abs/1604.07446)


## Controle de velocidade e direção do regulador linear-quadrático (LQR)

Simulação de rastreamento de caminho com velocidade LQR e controle de direção.

![3](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathTracking/lqr_speed_steer_control/animation.gif)

Ref:

- [Towards fully autonomous driving: Systems and algorithms \- IEEE Conference Publication](http://ieeexplore.ieee.org/document/5940562/)


## Velocidade preditiva do modelo e controle de direção

Simulação de rastreamento de caminho com velocidade preditiva de modelo linear iterativo e controle de direção.

<img src="https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathTracking/model_predictive_speed_and_steer_control/animation.gif" width="640" alt="MPC pic">

Ref:

- [notebook](https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathTracking/model_predictive_speed_and_steer_control/Model_predictive_speed_and_steering_control.ipynb)

- [Real\-time Model Predictive Control \(MPC\), ACADO, Python \| Work\-is\-Playing](http://grauonline.de/wordpress/?page_id=3244)

## Controle preditivo de modelo não linear com C-GMRES

Um planejamento de movimento e simulação de rastreamento de caminho com NMPC de C-GMRES

![3](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/PathTracking/cgmres_nmpc/animation.gif)

Ref:

- [notebook](https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathTracking/cgmres_nmpc/cgmres_nmpc.ipynb)


# Navegação do braço

## N braço articulado para controle de ponto

N braço articulado para uma simulação de controle de ponto.

Esta é uma simulação interativa.

Você pode definir a posição final do efetuador final com o botão esquerdo do mouse na área de plotagem.

![3](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/ArmNavigation/n_joint_arm_to_point_control/animation.gif)

Nesta simulação N = 10, no entanto, você pode alterá-lo.

## Navegação de braço com prevenção de obstáculos 

Navegação de braço com simulação de prevenção de obstáculos.

![3](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/ArmNavigation/arm_obstacle_navigation/animation.gif)


# Navegação Aérea

## drone 3d trajetória seguindo

Esta é uma trajetória 3d seguindo a simulação para um quadricóptero.

![3](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/AerialNavigation/drone_3d_trajectory_following/animation.gif)

## pouso movido a foguete

Esta é uma simulação de geração de trajetória 3D para um pouso movido a foguete.

![3](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/AerialNavigation/rocket_powered_landing/animation.gif)

Ref:

- [notebook](https://github.com/AtsushiSakai/PythonRobotics/blob/master/AerialNavigation/rocket_powered_landing/rocket_powered_landing.ipynb)

# Bípede

## planner bípede com pêndulo invertido

Este é um planejador bípede para modificar passos para um pêndulo invertido.

Você pode definir os passos e o planejador os modificará automaticamente.

![3](https://github.com/AtsushiSakai/PythonRoboticsGifs/raw/master/Bipedal/bipedal_planner/animation.gif)

# Licença  

MIT

# Use-case

Se este projeto ajudar seu projeto de robótica, informe-nos sobre a criação de um problema.

O vídeo do seu robô, que está usando PythonRobotics, é muito bem vindo!!

Esta é uma lista de comentários e referências do usuário:[users\_comments](https://github.com/AtsushiSakai/PythonRobotics/blob/master/users_comments.md)

# Contribuição

Qualquer contribuição é bem vinda!! 

Por favor, verifique este documento:[How To Contribute — PythonRobotics documentation](https://atsushisakai.github.io/PythonRobotics/how_to_contribute.html)

# Citando

Se você usar o código deste projeto para seu trabalho acadêmico, nós o encorajamos a citar [our papers](https://arxiv.org/abs/1808.10703) 

Se você usar o código deste projeto na indústria, adoraríamos ouvir você também; sinta-se à vontade para entrar em contato diretamente com os desenvolvedores.

# <a id="support"></a>Apoiando este projeto

Se você ou sua empresa gostaria de apoiar este projeto, considere:

- [Sponsor @AtsushiSakai on GitHub Sponsors](https://github.com/sponsors/AtsushiSakai)

- [Become a backer or sponsor on Patreon](https://www.patreon.com/myenigma)

- [One-time donation via PayPal](https://www.paypal.me/myenigmapay/)

Se você gostaria de nos apoiar de alguma outra forma, entre em contato com a criação de um problema.

## <a id="sponsors"></a>Patrocinadores

### <a id="JetBrains"></a>[JetBrains](https://www.jetbrains.com/)

Eles estão fornecendo uma licença gratuita de seus IDEs para este desenvolvimento de OSS.  

### [1Password](https://github.com/1Password/1password-teams-open-source)

Eles estão fornecendo uma licença gratuita de sua licença de equipe 1Password para este projeto OSS. 


# Autores

- [Contributors to AtsushiSakai/PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics/graphs/contributors)

