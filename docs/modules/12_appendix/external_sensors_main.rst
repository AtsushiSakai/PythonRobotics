.. _`External Sensors for Robots`:

External Sensors for Robots
============================

This project, `PythonRobotics`, focuses on algorithms, so hardware is not included.
However, having basic knowledge of hardware in robotics is also important for understanding algorithms.
Therefore, we will provide an overview.

Introduction
------------

In recent years, the application of robotic technology has advanced, particularly in areas such as autonomous vehicles and disaster response robots. A crucial element in these technologies is external recognition—the robot's ability to understand its surrounding environment, identify safe zones, and detect moving objects using onboard sensors. Achieving effective external recognition involves various techniques, but equally important is the selection of appropriate sensors. Robots, like the sensors they employ, come in many forms, but external recognition sensors can be broadly categorized into three types. Developing an advanced external recognition system requires a thorough understanding of each sensor's principles and characteristics to determine their optimal application. This article summarizes the principles and features of these sensors for personal study purposes.

Laser Sensors
-------------

Laser sensors measure distances by utilizing light, commonly referred to as Light Detection and Ranging (LIDAR). They operate by emitting light towards an object and calculating the distance based on the time it takes for the reflected light to return, using the speed of light as a constant.

Radar Sensors
-------------

Radar measures distances using radio waves, commonly referred to as Radio Detection and Ranging (RADAR). It operates by transmitting radio signals towards an object and calculating the distance based on the time it takes for the reflected waves to return, using the speed of radio waves as a constant. 


Monocular Cameras
-----------------

Monocular cameras utilize a single camera to recognize the external environment. Compared to other sensors, they can detect color and brightness information, making them primarily useful for object recognition. However, they face challenges in independently measuring distances to surrounding objects and may struggle in low-light or dark conditions.

Requirements for Cameras and Image Processing in Robotics
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

While camera sensors are widely used in applications like surveillance, deploying them in robotics necessitates meeting specific requirements:

1. High dynamic range to adapt to various lighting conditions
2. Wide measurement range
3. Capability for three-dimensional measurement through techniques like motion stereo
4. Real-time processing with high frame rates
5. Cost-effectiveness

Stereo Cameras
--------------

Stereo cameras employ multiple cameras to measure distances to surrounding objects. By knowing the positions and orientations of each camera and analyzing the disparity in the images (parallax), the distance to a specific point (the object represented by a particular pixel) can be calculated.

Characteristics of Stereo Cameras
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Advantages of stereo cameras include the ability to obtain high-precision and high-density distance information at close range, depending on factors like camera resolution and the distance between cameras (baseline). This makes them suitable for indoor robots that require precise shape recognition of nearby objects. Additionally, stereo cameras are relatively cost-effective compared to other sensors, leading to their use in consumer products like Subaru's EyeSight system. However, stereo cameras are less effective for long-distance measurements due to a decrease in accuracy proportional to the square of the distance. They are also susceptible to environmental factors such as lighting conditions.

Ultrasonic Sensors
------------------

Ultrasonic sensors are commonly used in indoor robots and some automotive autonomous driving systems. Their features include affordability compared to laser or radar sensors, the ability to detect very close objects, and the capability to sense materials like glass, which may be challenging for lasers or cameras. However, they have limitations such as shorter maximum measurement distances and lower resolution and accuracy.

References
----------

- Wikipedia articles:

  - `Lidar Sensors <https://en.wikipedia.org/wiki/Lidar>`_
  - `Radar Sensors <https://en.wikipedia.org/wiki/Radar>`_
  - `Stereo Cameras <https://en.wikipedia.org/wiki/Stereo_camera>`_
  - `Ultrasonic Sensors <https://en.wikipedia.org/wiki/Ultrasonic_transducer>`_