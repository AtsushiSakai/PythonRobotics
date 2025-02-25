.. _`Path Planning`:

Path Planning
=============

Path planning is the ability of a robot to search feasible and efficient path to the goal. The path has to satisfy some constraints based on the robot’s motion model and obstacle positions, and optimize some objective functions such as time to goal and distance to obstacle. In path planning, dynamic programming based approaches and sampling based approaches are widely used[22]. Fig.5 shows simulation results of potential field path planning and LQRRRT* path planning[27].

.. toctree::
   :maxdepth: 2
   :caption: Contents

   dynamic_window_approach/dynamic_window_approach
   bugplanner/bugplanner
   grid_base_search/grid_base_search
   time_based_grid_search/time_based_grid_search
   model_predictive_trajectory_generator/model_predictive_trajectory_generator
   state_lattice_planner/state_lattice_planner
   prm_planner/prm_planner
   visibility_road_map_planner/visibility_road_map_planner
   vrm_planner/vrm_planner
   rrt/rrt
   cubic_spline/cubic_spline
   bspline_path/bspline_path
   catmull_rom_spline/catmull_rom_spline
   clothoid_path/clothoid_path
   eta3_spline/eta3_spline
   bezier_path/bezier_path
   quintic_polynomials_planner/quintic_polynomials_planner
   dubins_path/dubins_path
   reeds_shepp_path/reeds_shepp_path
   lqr_path/lqr_path
   hybridastar/hybridastar
   frenet_frame_path/frenet_frame_path
   coverage_path/coverage_path
   elastic_bands/elastic_bands