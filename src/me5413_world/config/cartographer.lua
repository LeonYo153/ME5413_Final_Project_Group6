------- This is lua for slope -------

-- include "map_builder.lua"
-- include "trajectory_builder.lua"

-- options = {
--   map_builder = MAP_BUILDER,
--   trajectory_builder = TRAJECTORY_BUILDER,
--   map_frame = "map",
--   tracking_frame = "imu_link",
--   published_frame = "odom",
--   odom_frame = "odom",
--   provide_odom_frame = false,
--   publish_frame_projected_to_2d = true,
--   use_odometry = true,
--   use_nav_sat = false,
--   use_landmarks = true,
--   num_laser_scans = 1,
--   num_multi_echo_laser_scans = 0,
--   num_subdivisions_per_laser_scan = 1,
--   num_point_clouds = 0,
--   lookup_transform_timeout_sec = 1.,
--   submap_publish_period_sec = 0.3,
--   pose_publish_period_sec = 5e-3,
--   trajectory_publish_period_sec = 30e-3,
--   rangefinder_sampling_ratio = 1.,
--   odometry_sampling_ratio = 0.5,
--   fixed_frame_pose_sampling_ratio = 1.,
--   imu_sampling_ratio = 1.,
--   landmarks_sampling_ratio = 1.,
-- }

-- MAP_BUILDER.use_trajectory_builder_2d = true

-- TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

-- TRAJECTORY_BUILDER_2D.min_range = 0.3
-- TRAJECTORY_BUILDER_2D.max_range = 10. -- Decrease
-- TRAJECTORY_BUILDER_2D.missing_data_ray_length = 2.
-- TRAJECTORY_BUILDER_2D.use_imu_data = true
-- TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
-- TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15
-- TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
-- TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 10.
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.--
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.-- increase the imu weight 40
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 3. -- increase the laser weight

-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(15.)
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 3.

-- POSE_GRAPH.optimization_problem.huber_scale = 1e2 --1e2

-- -----------------TUNE THESE PARAMETERS FOR LOW LATENCY-------------------------------

-- ------------Global SLAM------------
-- POSE_GRAPH.optimize_every_n_nodes = 10-- Decrease 100
-- MAP_BUILDER.num_background_threads = 4 -- Increase up to number of cores
-- POSE_GRAPH.global_sampling_ratio = 0.003 -- Decrease
-- POSE_GRAPH.constraint_builder.sampling_ratio = 0.4 -- Decrease
-- POSE_GRAPH.constraint_builder.min_score = 0.85 -- Increase
-- POSE_GRAPH.global_constraint_search_after_n_seconds = 30 -- Increase
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 10 -- Decrease

-- -- trust the frontend
-- POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e15
-- POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e15

-- ---------Global/Local SLAM---------
-- TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 100 -- Decrease 100
-- TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 10. -- Decrease
-- TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 1.0 -- Increase 1.0
-- TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.min_num_points = 50 -- Decrease 50
-- TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_range = 10. -- Decrease
-- TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_length = 1.8 -- Increase
-- TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05 -- Increase
-- TRAJECTORY_BUILDER_2D.submaps.num_range_data = 100 -- Decrease 100

-- -- landmark
-- TRAJECTORY_BUILDER.collate_landmarks = false

-- -------------------------------------------------------------------------------------

-- return options


------- This is lua for 1st floor -------
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = true,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 1.,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 0.5,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
  -- publish_tracked_pose = true
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

TRAJECTORY_BUILDER_2D.min_range = 0.3
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 2.
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 100

POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(15.)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 3.
POSE_GRAPH.optimization_problem.log_solver_summary = true
POSE_GRAPH.optimization_problem.huber_scale = 1e1
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e3
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e3
-----------------TUNE THESE PARAMETERS FOR LOW LATENCY-------------------------------

------------Global SLAM------------
POSE_GRAPH.optimize_every_n_nodes = 70 -- Decrease
MAP_BUILDER.num_background_threads = 4 -- Increase up to number of cores
POSE_GRAPH.global_sampling_ratio = 0.003 -- Decrease
POSE_GRAPH.constraint_builder.sampling_ratio = 0.4 -- Decrease
POSE_GRAPH.constraint_builder.min_score = 0.55 -- Increase
POSE_GRAPH.global_constraint_search_after_n_seconds = 30 -- Increase
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 20 -- Decrease 5

---------Global/Local SLAM---------
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 100 -- Decrease
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 10. -- Decrease
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 1.0 -- Increase
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.min_num_points = 50 -- Decrease
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_range = 10. -- Decrease
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_length = 1.8 -- Increase
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05 -- Increase
-- TRAJECTORY_BUILDER_2D.submaps.resolution=0.05 -- Increase
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 100 -- Decrease
TRAJECTORY_BUILDER_2D.max_range = 10. -- Decrease

-------------------------------------------------------------------------------------
-- TRAJECTORY_BUILDER.pure_localization_trimmer = {
--   max_submaps_to_keep = 3,
-- }

return options

------- this is lua for 2nd with landmarks -------

-- include "map_builder.lua"

-- include "trajectory_builder.lua"


-- options = {

--   map_builder = MAP_BUILDER,
--   trajectory_builder = TRAJECTORY_BUILDER,
--   map_frame = "map",
--   tracking_frame = "imu_link",
--   published_frame = "odom",
--   odom_frame = "odom",
--   provide_odom_frame = false,
--   publish_frame_projected_to_2d = true,
--   use_odometry = true,
--   use_nav_sat = false,
--   use_landmarks = true,
--   num_laser_scans = 1,
--   num_multi_echo_laser_scans = 0,
--   num_subdivisions_per_laser_scan = 1,
--   num_point_clouds = 0,
--   lookup_transform_timeout_sec = 1.,
--   submap_publish_period_sec = 0.3,
--   pose_publish_period_sec = 5e-3,
--   trajectory_publish_period_sec = 30e-3,
--   rangefinder_sampling_ratio = 1.,
--   odometry_sampling_ratio = 0.5,
--   fixed_frame_pose_sampling_ratio = 1.,
--   imu_sampling_ratio = 1.,
--   landmarks_sampling_ratio = 1.,

-- }


-- MAP_BUILDER.use_trajectory_builder_2d = true

-- TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
-- TRAJECTORY_BUILDER_2D.min_range = 0.3
-- TRAJECTORY_BUILDER_2D.missing_data_ray_length = 2.
-- TRAJECTORY_BUILDER_2D.use_imu_data = true
-- TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
-- TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
-- TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
-- TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 10.
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 8
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 100


-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(15.)
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 5.
-- POSE_GRAPH.constraint_builder.max_constraint_distance = 10.0 --add
-- POSE_GRAPH.optimization_problem.huber_scale = 1e0
-- POSE_GRAPH.optimization_problem.log_solver_summary = true
-- POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e4
-- POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 5e4
-- POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e1
-- POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e1


-- ------------Global SLAM------------

-- POSE_GRAPH.optimize_every_n_nodes = 50 -- Decrease
-- MAP_BUILDER.num_background_threads = 4 -- Increase up to number of cores
-- POSE_GRAPH.global_sampling_ratio = 0.003 -- Decrease
-- POSE_GRAPH.constraint_builder.sampling_ratio = 0.4 -- Decrease
-- POSE_GRAPH.constraint_builder.min_score = 0.85 -- Increase
-- POSE_GRAPH.global_constraint_search_after_n_seconds = 30 -- Increase
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 5 -- Decrease


-- ---------Global/Local SLAM---------

-- TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 100 -- Decrease
-- TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 10. -- Decrease
-- TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 1.0 -- Increase
-- TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.min_num_points = 50 -- Decrease
-- TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_range = 10. -- Decrease
-- TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_length = 1.8 -- Increase
-- TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05 -- Increase
-- TRAJECTORY_BUILDER_2D.submaps.num_range_data = 70 -- Decrease
-- TRAJECTORY_BUILDER_2D.max_range = 10. -- Decrease

-- TRAJECTORY_BUILDER.collate_landmarks = false

-- -------------------------------------------------------------------------------------


-- return options


------- this is lua for 2nd without landmarks -------

-- include "map_builder.lua"

-- include "trajectory_builder.lua"


-- options = {

--   map_builder = MAP_BUILDER,
--   trajectory_builder = TRAJECTORY_BUILDER,
--   map_frame = "map",
--   tracking_frame = "imu_link",
--   published_frame = "odom",
--   odom_frame = "odom",
--   provide_odom_frame = false,
--   publish_frame_projected_to_2d = true,
--   use_odometry = true,
--   use_nav_sat = false,
--   use_landmarks = false,
--   num_laser_scans = 1,
--   num_multi_echo_laser_scans = 0,
--   num_subdivisions_per_laser_scan = 1,
--   num_point_clouds = 0,
--   lookup_transform_timeout_sec = 1.,
--   submap_publish_period_sec = 0.3,
--   pose_publish_period_sec = 5e-3,
--   trajectory_publish_period_sec = 30e-3,
--   rangefinder_sampling_ratio = 1.,
--   odometry_sampling_ratio = 0.5,
--   fixed_frame_pose_sampling_ratio = 1.,
--   imu_sampling_ratio = 1.,
--   landmarks_sampling_ratio = 1.,

-- }


-- MAP_BUILDER.use_trajectory_builder_2d = true

-- TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
-- TRAJECTORY_BUILDER_2D.min_range = 0.3
-- TRAJECTORY_BUILDER_2D.missing_data_ray_length = 2.
-- TRAJECTORY_BUILDER_2D.use_imu_data = true
-- TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
-- TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
-- TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
-- TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 10.
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 8
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 50


-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(15.)
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 5.
-- POSE_GRAPH.constraint_builder.max_constraint_distance = 10.0 --add
-- POSE_GRAPH.optimization_problem.huber_scale = 1e3
-- POSE_GRAPH.optimization_problem.log_solver_summary = true
-- POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e2
-- POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e2
-- POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e1
-- POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e1


-- ------------Global SLAM------------

-- POSE_GRAPH.optimize_every_n_nodes = 50 -- Decrease
-- MAP_BUILDER.num_background_threads = 4 -- Increase up to number of cores
-- POSE_GRAPH.global_sampling_ratio = 0.003 -- Decrease
-- POSE_GRAPH.constraint_builder.sampling_ratio = 0.4 -- Decrease
-- POSE_GRAPH.constraint_builder.min_score = 0.92 -- Increase
-- POSE_GRAPH.global_constraint_search_after_n_seconds = 30 -- Increase
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 5 -- Decrease


-- ---------Global/Local SLAM---------

-- TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 100 -- Decrease
-- TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 10. -- Decrease
-- TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 1.0 -- Increase
-- TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.min_num_points = 50 -- Decrease
-- TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_range = 10. -- Decrease
-- TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_length = 1.8 -- Increase
-- TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05 -- Increase
-- TRAJECTORY_BUILDER_2D.submaps.num_range_data = 70 -- Decrease
-- TRAJECTORY_BUILDER_2D.max_range = 10. -- Decrease

-- TRAJECTORY_BUILDER.collate_landmarks = false

-- -------------------------------------------------------------------------------------


-- return options