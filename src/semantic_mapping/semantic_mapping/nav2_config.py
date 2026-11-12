"""Semantic-navigation-only Nav2 controller configuration."""


def configure_semantic_navigation_params(params):
    """Use conservative controller and BT timeouts in the per-run Nav2 copy."""
    bt_navigator = params['bt_navigator']['ros__parameters']
    bt_navigator['default_server_timeout'] = 1500
    bt_navigator['bt_loop_duration'] = 50
    bt_navigator['wait_for_service_timeout'] = 3000

    controller = params['controller_server']['ros__parameters']
    controller['controller_frequency'] = 10.0
    controller['controller_plugins'] = ['FollowPath']
    progress = controller['progress_checker']
    progress['required_movement_radius'] = 0.08
    progress['movement_time_allowance'] = 45.0
    follow_path = controller['FollowPath']
    follow_path.clear()
    follow_path.update({
        'plugin': 'nav2_mppi_controller::MPPIController',
        'motion_model': 'DiffDrive',
        # Humble MPPI requires model_dt to match the controller period.
        # 56 steps predict 5.6 s / at most 0.672 m at this low driving speed.
        'model_dt': 0.1,
        'time_steps': 56,
        'batch_size': 1000,
        'iteration_count': 1,
        'vx_std': 0.06,
        'vy_std': 0.1,
        'wz_std': 0.15,
        'vx_max': 0.12,
        'vx_min': 0.0,
        'vy_max': 0.0,
        'wz_max': 0.2,
        'temperature': 0.3,
        'gamma': 0.015,
        'prune_distance': 1.5,
        'transform_tolerance': 1.0,
        'reset_period': 1.0,
        'retry_attempt_limit': 1,
        'regenerate_noises': False,
        # Humble publishes the optimized local trajectory as MarkerArray on
        # /trajectories, rather than the DWB /local_plan Path topic.
        'visualize': True,
        # Limit candidate marker generation to 20 trajectories per cycle;
        # RViz displays only the optimized trajectory by default.
        'TrajectoryVisualizer': {'trajectory_step': 50, 'time_step': 4},
        'critics': [
            'ConstraintCritic', 'ObstaclesCritic', 'GoalCritic',
            'GoalAngleCritic', 'PathAlignCritic', 'PathFollowCritic',
            'PathAngleCritic', 'PreferForwardCritic',
        ],
        'ConstraintCritic': {'enabled': True, 'cost_power': 1, 'cost_weight': 4.0},
        'ObstaclesCritic': {
            'enabled': True,
            'cost_power': 1,
            'consider_footprint': True,
            'critical_weight': 20.0,
            'repulsion_weight': 1.5,
            'collision_cost': 10000.0,
            'collision_margin_distance': 0.1,
            'near_goal_distance': 0.3,
            # Match the local costmap inflation layer (Humble-specific inputs).
            'cost_scaling_factor': 3.0,
            'inflation_radius': 0.4,
        },
        'GoalCritic': {
            'enabled': True, 'cost_power': 1, 'cost_weight': 5.0,
            'threshold_to_consider': 0.6,
        },
        'GoalAngleCritic': {
            'enabled': True, 'cost_power': 1, 'cost_weight': 3.0,
            'threshold_to_consider': 0.3,
        },
        'PathAlignCritic': {
            'enabled': True, 'cost_power': 1, 'cost_weight': 14.0,
            'threshold_to_consider': 0.3, 'offset_from_furthest': 6,
            'trajectory_point_step': 4, 'max_path_occupancy_ratio': 0.05,
            'use_path_orientations': False,
        },
        'PathFollowCritic': {
            'enabled': True, 'cost_power': 1, 'cost_weight': 5.0,
            'threshold_to_consider': 0.3, 'offset_from_furthest': 4,
        },
        'PathAngleCritic': {
            'enabled': True, 'cost_power': 1, 'cost_weight': 2.0,
            'threshold_to_consider': 0.3, 'offset_from_furthest': 4,
            'max_angle_to_furthest': 1.0, 'mode': 0,
        },
        'PreferForwardCritic': {
            'enabled': True, 'cost_power': 1, 'cost_weight': 5.0,
            'threshold_to_consider': 0.3,
        },
    })
    smoother = params['velocity_smoother']['ros__parameters']
    smoother['max_velocity'] = [0.12, 0.0, 0.2]
    smoother['min_velocity'] = [-0.12, 0.0, -0.2]
    smoother['max_accel'] = [0.2, 0.0, 0.2]
    smoother['max_decel'] = [-0.2, 0.0, -0.2]
    behavior = params['behavior_server']['ros__parameters']
    behavior['max_rotational_vel'] = 0.2
    behavior['min_rotational_vel'] = 0.05
    behavior['rotational_acc_lim'] = 0.2
    return params
