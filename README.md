<img src="https://github.com/AtsushiSakai/PythonRobotics/raw/master/icon.png?raw=true" align="right" width="300" alt="header pic"/>

# 🧠 PythonRobotics

[![GitHub Linux CI](https://github.com/AtsushiSakai/PythonRobotics/workflows/Linux_CI/badge.svg)](https://github.com/AtsushiSakai/PythonRobotics)
[![GitHub MacOS CI](https://github.com/AtsushiSakai/PythonRobotics/workflows/MacOS_CI/badge.svg)](https://github.com/AtsushiSakai/PythonRobotics)
[![GitHub Windows CI](https://github.com/AtsushiSakai/PythonRobotics/workflows/Windows_CI/badge.svg)](https://github.com/AtsushiSakai/PythonRobotics)
[![Build status](https://ci.appveyor.com/api/projects/status/sb279kxuv1be391g?svg=true)](https://ci.appveyor.com/project/AtsushiSakai/pythonrobotics)

A collection of simple and easy-to-understand Python examples for common robotics algorithms. Great for **beginners**, students, and professionals who want to learn how robots think and move.

---

## 📘 What is PythonRobotics?

PythonRobotics is a project that helps you learn how robots **navigate**, **map**, and **track paths** by providing clean and easy-to-read code examples in Python.

It’s also a great companion to our free [online textbook](https://atsushisakai.github.io/PythonRobotics/index.html), which explains the math behind each algorithm.

### ✅ Key features:

* Simple code that’s easy to read and modify
* Real robotics algorithms used in research and industry
* No heavy dependencies – works on almost any computer

📖 [Getting started guide](https://atsushisakai.github.io/PythonRobotics/modules/0_getting_started/1_what_is_python_robotics.html)
🎥 [YouTube overview](https://www.youtube.com/watch?v=uMeRnNoJAfU)
📄 [Research paper](https://arxiv.org/abs/1808.10703)

---

## ⚙️ Requirements

To run the examples, you’ll need:

* Python 3.13 (or newer)
* Libraries:

  * `numpy`
  * `scipy`
  * `matplotlib`
  * `cvxpy`

For development:

* `pytest`, `pytest-xdist`, `mypy`, `sphinx`, `pycodestyle`

---

## 🚀 How to Get Started

1. **Clone the repository**

```bash
git clone https://github.com/AtsushiSakai/PythonRobotics.git
cd PythonRobotics
```

2. **Install the dependencies**

With **Conda**:

```bash
conda env create -f requirements/environment.yml
conda activate pythonrobotics
```

With **pip**:

```bash
pip install -r requirements/requirements.txt
```

3. **Run any Python script** from the folders to see an algorithm in action:

```bash
python PathPlanning/AStar/a_star.py
```

4. ⭐ If you find it useful, give the repo a star!

---

## 🧭 Topics Covered

Each folder contains ready-to-run simulations. Here are some of the categories:

### 🤖 Localization (robot figuring out where it is)

* Extended Kalman Filter (EKF)
* Particle Filter
* Histogram Filter

### 🗺️ Mapping (building a map from sensor data)

* Grid mapping with Gaussian or Ray Casting
* Lidar to map
* Object clustering and shape fitting

### 🔁 SLAM (Simultaneous Localization and Mapping)

* FastSLAM
* ICP (Iterative Closest Point)

### 📍 Path Planning (finding a route)

* Dijkstra, A\*, D\*, PRM, RRT\*
* State lattice planning
* Frenet trajectories

### 🧭 Path Tracking (following a path)

* Stanley controller
* LQR and PID control
* Model Predictive Control (MPC)

### 🦾 Arm Navigation

* Multi-joint arm control
* Obstacle avoidance

### 🚁 Aerial Navigation

* Drone path tracking
* Rocket landing simulation

### 🦿 Bipedal Movement

* Footstep planning with inverted pendulum model

---

## 📚 Full Documentation

The full **textbook** with explanations and formulas is available here:
📘 [PythonRobotics Documentation](https://atsushisakai.github.io/PythonRobotics/index.html)

---

## 💡 Real-world Use

If you're using this code in your robot or project, we’d love to hear from you!
➡️ Share your story or video in an [issue](https://github.com/AtsushiSakai/PythonRobotics/issues).
📝 Check out the [user stories](https://github.com/AtsushiSakai/PythonRobotics/blob/master/users_comments.md)

---

## 🤝 Contributing

Contributions are welcome! Whether it’s fixing a bug, improving documentation, or adding new examples.
📄 [How to Contribute Guide](https://atsushisakai.github.io/PythonRobotics/modules/0_getting_started/3_how_to_contribute.html)

---

## 📌 Citation

If you use this project in academic research, please cite the [PythonRobotics paper](https://arxiv.org/abs/1808.10703).

```bibtex
@article{sakai2018pythonrobotics,
  title={PythonRobotics: a Python code collection of robotics algorithms},
  author={Atsushi Sakai et al.},
  journal={arXiv preprint arXiv:1808.10703},
  year={2018}
}
```

---

## ❤️ Support This Project

You can help support this open-source work by:

* [GitHub Sponsor](https://github.com/sponsors/AtsushiSakai)
* [Patreon](https://www.patreon.com/myenigma)
* [PayPal](https://www.paypal.com/paypalme/myenigmapay/)

---

## 🎉 Sponsors

* [JetBrains](https://www.jetbrains.com/)
* [1Password](https://github.com/1Password/for-open-source)

---

## 👥 Authors

Built and maintained by [Atsushi Sakai](https://github.com/AtsushiSakai) and [contributors](https://github.com/AtsushiSakai/PythonRobotics/graphs/contributors).
