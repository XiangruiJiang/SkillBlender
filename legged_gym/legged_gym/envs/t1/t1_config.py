# SPDX-License-Identifier: BSD-3-Clause
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2024 Beijing RobotEra TECHNOLOGY CO.,LTD. All rights reserved.


from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO


class T1RoughCfg(LeggedRobotCfg):
    """
    Configuration class for the T1 humanoid robot, specifically tailored for t1.py.
    """

    class env(LeggedRobotCfg.env):
        num_actions = 12
        # Parameters for history buffers in t1.py
        frame_stack = 1
        c_frame_stack = 3
        command_dim = 3
        # num_single_obs calculation from the original config, seems correct for t1.py's obs buffer
        num_single_obs = 3 * num_actions + 6 + command_dim
        num_observations = int(frame_stack * num_single_obs)
        # single_num_privileged_obs calculation from the original config
        single_num_privileged_obs = 4 * num_actions + 25
        num_privileged_obs = int(c_frame_stack * single_num_privileged_obs)

        num_envs = 4096
        episode_length_s = 24
        # Flag to enable/disable reference motion addition in t1.py
        use_ref_actions = False

    class asset(LeggedRobotCfg.asset):
        file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/T1/T1_locomotion.urdf'
        name = "t1"
        foot_name = "foot"
        # CRITICAL FIX: Use "Shank" to identify knee links, as "Knee" does not exist in the URDF links.
        knee_name = "Shank"
        elbow_name = "elbow"
        torso_name = "Trunk"
        wrist_name = "wrist"

        terminate_after_contacts_on = ['Trunk', 'Hip', 'Shank']
        penalize_contacts_on = ["Hip", 'Knee']  # Note: 'Knee' might not match any link, 'Shank' is better.
        self_collisions = 0
        flip_visual_attachments = False
        fix_base_link = False

    class terrain(LeggedRobotCfg.terrain):
        mesh_type = 'trimesh'
        curriculum = True
        measure_heights = False  # Set to False as per the original config
        static_friction = 0.6
        dynamic_friction = 0.6
        terrain_length = 8.
        terrain_width = 8.
        num_rows = 20
        num_cols = 20
        max_init_terrain_level = 10
        terrain_proportions = [0.2, 0.2, 0.4, 0.1, 0.1, 0, 0]
        restitution = 0.

    class init_state(LeggedRobotCfg.init_state):
        pos = [0.0, 0.0, 0.72]
        # All 12 movable joints from the URDF must be defined here.
        default_joint_angles = {
            # Left Leg
            'Left_Hip_Pitch': -0.2,
            'Left_Hip_Roll': 0.0,  # ADDED
            'Left_Hip_Yaw': 0.0,  # ADDED
            'Left_Knee_Pitch': 0.4,
            'Left_Ankle_Pitch': -0.25,
            'Left_Ankle_Roll': 0.0,  # ADDED

            # Right Leg
            'Right_Hip_Pitch': -0.2,
            'Right_Hip_Roll': 0.0,  # ADDED
            'Right_Hip_Yaw': 0.0,  # ADDED
            'Right_Knee_Pitch': 0.4,
            'Right_Ankle_Pitch': -0.25,
            'Right_Ankle_Roll': 0.0,  # ADDED
        }

    class control(LeggedRobotCfg.control):
        control_type = 'P'
        stiffness = {"Hip": 200., "Knee": 200., "Ankle": 50.}
        damping = {"Hip": 5., "Knee": 5., "Ankle": 1.}
        action_scale = 0.25
        decimation = 10

    class sim(LeggedRobotCfg.sim):
        dt = 0.001
        substeps = 1
        up_axis = 1  # 1 is z

        class physx(LeggedRobotCfg.sim.physx):
            num_threads = 10
            solver_type = 1
            num_position_iterations = 4
            num_velocity_iterations = 0
            contact_offset = 0.01
            rest_offset = 0.0
            max_depenetration_velocity = 1.0

    class commands(LeggedRobotCfg.commands):
        num_commands = 4
        resampling_time = 8.
        heading_command = True
        curriculum = True

        class ranges:
            lin_vel_x = [-0.5, 1.0]
            lin_vel_y = [-0.5, 0.5]
            ang_vel_yaw = [-0.5, 0.5]
            heading = [-3.14, 3.14]

    class rewards(LeggedRobotCfg.rewards):
        # ADDED: Parameters required by reward functions in t1.py
        base_height_target = 0.68
        min_dist = 0.05
        max_dist = 0.25
        target_joint_pos_scale = 0.17
        target_feet_height = 0.06
        cycle_time = 0.64
        max_contact_force = 700
        tracking_sigma = 5.0  # From original config
        only_positive_rewards = True

        # The keys in this dictionary MUST match the reward function names in t1.py
        class scales:
            joint_pos = 1.6
            feet_clearance = 1.0 * 2
            feet_contact_number = 1.2 * 2
            feet_air_time = 1.0
            foot_slip = -0.05
            feet_distance = 0.2
            knee_distance = 0.2
            feet_contact_forces = -0.01
            tracking_lin_vel = 1.2 * 2
            tracking_ang_vel = 1.1 * 2
            vel_mismatch_exp = 0.5
            low_speed = 0.2
            track_vel_hard = 0.5 * 2
            default_joint_pos = 0.5
            upper_body_pos = 0.0  # Set to 0 as it's not applicable
            orientation = 1.0
            base_height = 0.2
            base_acc = 0.2
            action_smoothness = -0.002
            torques = -1e-5
            dof_vel = -5e-4
            dof_acc = -1e-7
            collision = -1.0
            # Add scales for any other reward functions you might have enabled
            # e.g., termination, dof_pos_limits, etc. if they are in t1.py
            termination = -0.0
            dof_pos_limits = -10.0


class T1RoughCfgPPO(LeggedRobotCfgPPO):
    seed = 5
    runner_class_name = 'OnPolicyRunner'

    class policy:
        init_noise_std = 1.0
        actor_hidden_dims = [512, 256, 128]
        critic_hidden_dims = [768, 256, 128]

    class algorithm(LeggedRobotCfgPPO.algorithm):
        entropy_coef = 0.001
        learning_rate = 1e-5
        num_learning_epochs = 2
        gamma = 0.994
        lam = 0.9
        num_mini_batches = 4

    class runner(LeggedRobotCfgPPO.runner):
        policy_class_name = 'ActorCritic'
        algorithm_class_name = 'PPO'
        num_steps_per_env = 60
        max_iterations = 15001

        save_interval = 1000
        experiment_name = 't1_walking'
        run_name = ''
        resume = False
        load_run = -1
        checkpoint = -1
        resume_path = None