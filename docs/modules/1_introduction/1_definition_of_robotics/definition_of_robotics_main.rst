Definition of Robotics
----------------------

This section explains the definition, history, key components, and applications of robotics.

What is Robotics?
^^^^^^^^^^^^^^^^^^

Robot is a machine that can perform tasks automatically or semi-autonomously.
Robotics is the study of robots.

The word “robot” comes from the Czech word “robota,” which means “forced labor” or “drudgery.”
It was first used in the 1920 science fiction play `R.U.R.`_ (Rossum’s Universal Robots)
by the Czech writer `Karel Čapek`_.
In the play, robots were artificial workers created to serve humans, but they eventually rebelled.

Over time, “robot” came to refer to machines or automated systems that can perform tasks,
often with some level of intelligence or autonomy.

Currently, 2 millions robots are working in the world, and the number is increasing every year.
In South Korea, where the adoption of robots is particularly rapid,
50 robots are operating per 1,000 people.

.. _`R.U.R.`: https://thereader.mitpress.mit.edu/origin-word-robot-rur/
.. _`Karel Čapek`: https://en.wikipedia.org/wiki/Karel_%C4%8Capek

The History of Robots
^^^^^^^^^^^^^^^^^^^^^^^^^^^

This timeline highlights key milestones in the history of robotics:

Ancient and Early Concepts (Before 1500s)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The idea of **automated machines** has existed for thousands of years.
Ancient civilizations imagined mechanical beings:

- **Ancient Greece (4th Century BC)** – Greek engineer `Hero of Alexandria`_ designed early **automata** (self-operating machines) powered by water or air.
- **Chinese and Arabic Automata (9th–13th Century)** – Inventors like `Ismail Al-Jazari`_ created intricate mechanical devices, including water clocks and automated moving peacocks driven by hydropower.

.. _`Hero of Alexandria`: https://en.wikipedia.org/wiki/Hero_of_Alexandria
.. _`Ismail Al-Jazari`: https://en.wikipedia.org/wiki/Ismail_al-Jazari

The Birth of Modern Robotics (1500s–1800s)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

- `Leonardo da Vinci’s Robot`_  (1495) – Designed a humanoid knight with mechanical movement.
- `Jacques de Vaucanson’s Digesting Duck`_ (1738) – Created robotic figures like a mechanical duck that could "eat" and "digest."
- `Industrial Revolution`_ (18th–19th Century) – Machines began replacing human labor in factories, setting the foundation for automation.

.. _`Leonardo da Vinci’s Robot`: https://en.wikipedia.org/wiki/Leonardo%27s_robot
.. _`Jacques de Vaucanson’s Digesting Duck`: https://en.wikipedia.org/wiki/Jacques_de_Vaucanson
.. _`Industrial Revolution`: https://en.wikipedia.org/wiki/Industrial_Revolution

The Rise of Industrial Robots (1900s–1950s)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

- **The Term “Robot” (1921)** – Czech writer `Karel Čapek`_ introduced the word *“robot”* in his play `R.U.R.`_ (Rossum’s Universal Robots).
- **Early Cybernetics (1940s–1950s)** – Scientists like `Norbert Wiener`_ developed theories of self-regulating machines, influencing modern robotics (Cybernetics).

.. _`Norbert Wiener`: https://en.wikipedia.org/wiki/Norbert_Wiener

The Birth of Modern Robotics (1950s–1980s)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

- **First Industrial Robot (1961)** – `Unimate`_, created by `George Devol`_ and `Joseph Engelberger`_, was the first programmable robot used in a factory.
- **Rise of AI & Autonomous Robots (1970s–1980s)** – Researchers developed mobile robots like `Shakey`_ (Stanford, 1966) and AI-based control systems.

.. _`Unimate`: https://en.wikipedia.org/wiki/Unimate
.. _`George Devol`: https://en.wikipedia.org/wiki/George_Devol
.. _`Joseph Engelberger`: https://en.wikipedia.org/wiki/Joseph_Engelberger
.. _`Shakey`: https://en.wikipedia.org/wiki/Shakey_the_robot

Advanced Robotics and AI Integration (1990s–Present)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

- **Industrial Robots** – Advanced robots like `Baxter`_ and `Amazon Robotics`_ revolutionized manufacturing and logistics.
- **Autonomous Vehicles** – Self-driving cars for robo taxi like `Waymo`_ and autonomous haulage system in mining like `AHS`_ became more advanced and bisiness-ready.
- **Exploration Robots** – Used for planetary exploration (e.g., NASA’s `Mars rovers`_).
- **Medical Robotics** – Robots like `da Vinci Surgical System`_ revolutionized healthcare.
- **Personal Robots** – Devices like `Roomba`_ (vacuum robot) and `Sophia`_ (AI humanoid) became popular.
- **Service Robots** - Assistive robots like serving robots in restaurants and hotels like `Bellabot`_.
- **Collaborative Robots (Drones)** – Collaborative robots like UAV (Unmanned Aerial Vehicle) in drone shows and delivery services.

.. _`Baxter`: https://en.wikipedia.org/wiki/Baxter_(robot)
.. _`Amazon Robotics`: https://en.wikipedia.org/wiki/Amazon_Robotics
.. _`Mars rovers`: https://en.wikipedia.org/wiki/Mars_rover
.. _`Waymo`: https://waymo.com/
.. _`AHS`: https://www.futurebridge.com/industry/perspectives-industrial-manufacturing/autonomous-haulage-systems-the-future-of-mining-operations/
.. _`da Vinci Surgical System`: https://en.wikipedia.org/wiki/Da_Vinci_Surgical_System
.. _`Roomba`: https://en.wikipedia.org/wiki/Roomba
.. _`Sophia`: https://en.wikipedia.org/wiki/Sophia_(robot)
.. _`Bellabot`: https://www.pudurobotics.com/en

Key Components of Robotics
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Robotics consists of several essential components:

#. Sensors – Gather information from the environment (e.g., Cameras, LiDAR, GNSS, Gyro, Accelerometer, Wheel encoders).
#. Actuators – Enable movement and interaction with the world (e.g., Motors, Hydraulic systems).
#. Computers – Process sensor data and make decisions (e.g., Micro-controllers, CPUs, GPUs).
#. Power Supply – Provides energy to run the robot (e.g., Batteries, Solar power).
#. Software & Algorithms – Allow the robot to function and make intelligent decisions (e.g., ROS, Machine learning models, Localization, Mapping, Path planning, Control).

This project, `PythonRobotics`, focuses on the software and algorithms part of robotics.
If you are interested in `Sensors` hardware, you can check :ref:`Internal Sensors for Robots` or :ref:`External Sensors for Robots`.
