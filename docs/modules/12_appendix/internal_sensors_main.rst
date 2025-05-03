.. _`Internal Sensors for Robots`:

Internal Sensors for Robots
============================

This project, `PythonRobotics`, focuses on algorithms, so hardware is not included. However, having basic knowledge of hardware in robotics is also important for understanding algorithms. Therefore, we will provide an overview.

Introduction
-------------

In robotic systems, internal sensors play a crucial role in monitoring the robot’s internal state, such as orientation, acceleration, angular velocity, altitude, and temperature. These sensors provide essential feedback that supports control, localization, and safety mechanisms. While external sensors perceive the environment, internal sensors give the robot self-awareness of its own motion and condition. Understanding the principles and characteristics of these sensors is vital to fully utilize their information within algorithms and decision-making systems. This section outlines the main internal sensors used in robotics.

Global Navigation Satellite System (GNSS)
-----------------------------------------

GNSS is a satellite-based navigation system that provides global positioning and time information by analyzing signals from multiple satellites. It is commonly used in outdoor environments for absolute positioning. Although GNSS offers global coverage without infrastructure dependency, its performance is limited indoors or in obstructed areas, and it suffers from low update rates and susceptibility to signal noise. It is widely used in outdoor navigation for drones, vehicles, and delivery robots.

Gyroscope
----------

A gyroscope measures angular velocity around the robot’s axes, enabling orientation and attitude estimation through detection of the Coriolis effect. Gyroscopes are compact, cost-effective, and provide high update rates, but they are prone to drift and require calibration or sensor fusion for long-term accuracy. These sensors are essential in drones, balancing robots, and IMU-based systems for motion tracking.

Accelerometer
---------------

An accelerometer measures linear acceleration along one or more axes, typically by detecting mass displacement due to motion. It is small, affordable, and useful for detecting movement, tilt, or vibration. However, accelerometers are limited by noisy output and cannot independently determine position without integration and fusion. They are commonly applied in wearable robotics, step counters, and vibration sensing.

Magnetometer
--------------

A magnetometer measures the direction and intensity of magnetic fields, enabling heading estimation based on Earth’s magnetic field. It is lightweight and useful for orientation, especially in GPS-denied environments, though it is sensitive to interference from electronics and requires calibration. Magnetometers are often used in conjunction with IMUs for navigation and directional awareness.

Inertial Measurement Unit (IMU)
--------------------------------

An IMU integrates a gyroscope, accelerometer, and sometimes a magnetometer to estimate a robot's motion and orientation through sensor fusion techniques such as Kalman filters. IMUs are compact and provide high-frequency data, which is essential for localization and navigation in GPS-denied areas. Nonetheless, they accumulate drift over time and require complex filtering to maintain accuracy. IMUs are found in drones, mobile robots, and motion tracking systems.

Pressure Sensor
----------------

Pressure sensors detect atmospheric or fluid pressure by measuring the force exerted on a diaphragm. They are effective for estimating altitude and monitoring environmental conditions, especially in drones and underwater robots. Although cost-effective and efficient, their accuracy may degrade due to temperature variation and limitations in low-altitude resolution.

Temperature Sensor
--------------------

Temperature sensors monitor environmental or internal component temperatures, using changes in resistance or voltage depending on sensor type (e.g., RTD or thermocouple). They are simple and reliable for safety and thermal regulation, though they may respond slowly and be affected by nearby electronics. Common applications include battery and motor monitoring, safety systems, and ambient sensing.

References
----------

- *Introduction to Autonomous Mobile Robots*, Roland Siegwart, Illah Nourbakhsh, Davide Scaramuzza
- *Probabilistic Robotics*, Sebastian Thrun, Wolfram Burgard, Dieter Fox
- Wikipedia articles:

  - `Inertial Measurement Unit (IMU) <https://en.wikipedia.org/wiki/Inertial_measurement_unit>`_
  - `Accelerometer <https://en.wikipedia.org/wiki/Accelerometer>`_
  - `Gyroscope <https://en.wikipedia.org/wiki/Gyroscope>`_
  - `Magnetometer <https://en.wikipedia.org/wiki/Magnetometer>`_
  - `Pressure sensor <https://en.wikipedia.org/wiki/Pressure_sensor>`_
  - `Temperature sensor <https://en.wikipedia.org/wiki/Thermometer>`_
- `Adafruit Sensor Guides <https://learn.adafruit.com/>`_