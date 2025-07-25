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


class H1_2TaskTransferCfg(LeggedRobotCfg):
    """
    Configuration class for the H1_2 humanoid robot.
    """
    class env(LeggedRobotCfg.env):
        # change the observation dim
        num_actions = 21
        frame_stack = 1
        c_frame_stack = 3
        command_dim = 9
        num_single_obs = 3 * num_actions + 6 + command_dim # see `obs_buf = torch.cat(...)` for details
        num_observations = int(frame_stack * num_single_obs)
        single_num_privileged_obs = 3 * num_actions + 39
        num_privileged_obs = int(c_frame_stack * single_num_privileged_obs)
        
        num_envs = 4096
        episode_length_s = 8  # episode length in seconds
        use_ref_actions = False

    class asset(LeggedRobotCfg.asset):
        file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/h1_2/h1_2_simplified_modified.urdf'

        name = "h1_2"
        foot_name = "ankle_roll"
        knee_name = "knee"
        elbow_name = "elbow"
        torso_name = "torso"
        wrist_name = "wrist_yaw"

        terminate_after_contacts_on = ['pelvis', 'torso', 'shoulder']
        penalize_contacts_on = ["hip", 'knee', 'pelvis', 'torso', 'shoulder', 'elbow']
        self_collisions = 0  # 1 to disable, 0 to enable...bitwise filter
        flip_visual_attachments = False
        replace_cylinder_with_capsule = False # replace collision cylinders with capsules, leads to faster/more stable simulation
        fix_base_link = False
        collapse_fixed_joints = False
        
        # table, box, etc
        # table
        front_table_dims = [0.9, 1.5, 0.05]
        front_table_offset = [1.0, 0.0, 0.975]
        back_table_dims = [0.9, 1.5, 0.05]
        back_table_offset = [-1.0, 0.0, 0.975]
        # box
        box_size = 0.1
        box_mass = 0.01
        box_range_x = [-0.45, -0.35]
        box_range_y = [-0.3, 0.3]

    class terrain(LeggedRobotCfg.terrain):
        mesh_type = 'plane'
        curriculum = False
        # mesh_type = 'trimesh'
        # curriculum = True
        
        # rough terrain only:
        measure_heights = False
        static_friction = 0.6
        dynamic_friction = 0.6
        terrain_length = 8.
        terrain_width = 8.
        num_rows = 20  # number of terrain rows (levels)
        num_cols = 20  # number of terrain cols (types)
        max_init_terrain_level = 10  # starting curriculum state
        # plane; obstacles; uniform; slope_up; slope_down, stair_up, stair_down
        terrain_proportions = [0.2, 0.2, 0.4, 0.1, 0.1, 0, 0]
        restitution = 0.

    class init_state(LeggedRobotCfg.init_state):
        pos = [0.0, 0.0, 1.05]  # x,y,z [m]
        default_joint_angles = { # = target angles [rad] when action = 0.0
            'left_hip_yaw_joint': 0,
            'left_hip_roll_joint': 0,
            'left_hip_pitch_joint': -0.4,
            'left_knee_joint': 0.8,
            'left_ankle_pitch_joint': -0.4,
            'left_ankle_roll_joint': 0.0,

            'right_hip_yaw_joint': 0,
            'right_hip_roll_joint': 0,
            'right_hip_pitch_joint': -0.4,
            'right_knee_joint': 0.8,
            'right_ankle_pitch_joint': -0.4,
            'right_ankle_roll_joint': 0.0,

            'torso_joint': 0,

            'left_shoulder_pitch_joint': 0,
            'left_shoulder_roll_joint': 0,
            'left_shoulder_yaw_joint': 0,
            'left_elbow_pitch_joint': 0,

            'right_shoulder_pitch_joint': 0,
            'right_shoulder_roll_joint': 0,
            'right_shoulder_yaw_joint': 0,
            'right_elbow_pitch_joint': 0,
        }

    class control(LeggedRobotCfg.control):
        # PD Drive parameters:
        control_type = 'P'
        # PD Drive parameters:
        stiffness = {
            'hip_yaw_joint': 200.,
            'hip_roll_joint': 200.,
            'hip_pitch_joint': 200.,
            'knee_joint': 300.,
            'ankle_pitch_joint': 40.,
            'ankle_roll_joint': 40.,
            'torso': 300,
            'shoulder': 100,
            "elbow":100,
        }  # [N*m/rad]
        damping = {
            'hip_yaw_joint': 2.5,
            'hip_roll_joint': 2.5,
            'hip_pitch_joint': 2.5,
            'knee_joint': 4,
            'ankle_pitch_joint': 2.0,
            'ankle_roll_joint': 2.0,
            'torso': 6,
            'shoulder': 2,
            "elbow":2,
        }  # [N*m/rad]  # [N*m*s/rad]
        # action scale: target angle = actionScale * action + defaultAngle
        action_scale = 0.25
        # decimation: Number of control action updates @ sim DT per policy DT
        decimation = 10

    class sim(LeggedRobotCfg.sim):
        dt = 0.001  # 1000 Hz
        substeps = 1  # 2
        up_axis = 1  # 0 is y, 1 is z

        class physx(LeggedRobotCfg.sim.physx):
            num_threads = 10
            solver_type = 1  # 0: pgs, 1: tgs
            num_position_iterations = 4
            num_velocity_iterations = 0
            contact_offset = 0.01  # [m]
            rest_offset = 0.0   # [m]
            bounce_threshold_velocity = 0.1  # [m/s]
            max_depenetration_velocity = 1.0
            max_gpu_contact_pairs = 2**23  # 2**24 -> needed for 8000 envs and more
            default_buffer_size_multiplier = 5
            # 0: never, 1: last sub-step, 2: all sub-steps (default=2)
            contact_collection = 2

    class commands(LeggedRobotCfg.commands):
        # Vers: lin_vel_x, lin_vel_y, ang_vel_yaw, heading (in heading mode ang_vel_yaw is recomputed from heading error)
        num_commands = 4
        resampling_time = 8.  # time before command are changed[s]
        heading_command = True  # if true: compute ang vel command from heading error
        curriculum = False # if true: curriculum update of commands

        class ranges:
            lin_vel_x = [-0, 0]  # min max [m/s]
            lin_vel_y = [-0, 0]   # min max [m/s]
            ang_vel_yaw = [-0, 0]    # min max [rad/s]
            heading = [-0, 0]
            # wrist pos command ranges
            wrist_max_radius = 0.25
            l_wrist_pos_x = [-0.10, 0.25]
            l_wrist_pos_y = [-0.10, 0.25]
            l_wrist_pos_z = [-0.25, 0.25]
            r_wrist_pos_x = [-0.10, 0.25]
            r_wrist_pos_y = [-0.25, 0.10]
            r_wrist_pos_z = [-0.25, 0.25]

    class rewards:
        base_height_target = 1.0
        min_dist = 0.2
        max_dist = 0.5
        # put some settings here for LLM parameter tuning
        target_joint_pos_scale = 0.17    # rad
        target_feet_height = 0.06       # m
        cycle_time = 0.64                # sec
        # if true negative total rewards are clipped at zero (avoids early termination problems)
        only_positive_rewards = True
        # tracking reward = exp(error*sigma)
        tracking_sigma = 5
        max_contact_force = 700  # forces above this value are penalized

        class scales:
            # TODO: 1. stand_still 2. joint_pos*2 3. add command input
            # reference motion tracking
            # joint_pos = 5
            # wrist_pos = 5
            box_pos = 5
            wrist_box_distance = 1
            # feet_clearance = 0
            # feet_contact_number = 0
            # # gait
            # feet_air_time = 0
            # foot_slip = -0.05
            # feet_distance = 0.5
            # knee_distance = 0.2
            # # elbow_distance = 0.4
            # # elbow_torso_distance = 0.4
            # # contact
            # feet_contact_forces = -0.01
            # # vel tracking
            # tracking_lin_vel = 0.
            # tracking_ang_vel = 0.
            # vel_mismatch_exp = 0.5  # lin_z; ang x,y
            # low_speed = 0.2
            # track_vel_hard = 0.5 * 2
            # # base pos
            # default_joint_pos = 0.5
            # upper_body_pos = 0.5
            # orientation = 1.
            # base_height = 0.2
            # base_acc = 0.2
            # energy
            # action_smoothness = -0.002
            # torques = -1e-5
            # dof_vel = -5e-4
            # dof_acc = -1e-7
            # collision = -0.2
            #### humanplus ####
            # lin_vel_z = -0.1
            # ang_vel_xy = -0.1
        
    class sensor(LeggedRobotCfg.sensor):
        enable_sensor = False
        class camera(LeggedRobotCfg.sensor.camera):
            height = 48
            width = 64
            offset = (0.1, 0.0, 0.9) # relative to root
            axis = (0, 1, 0) # camera axis
            angle = 45 # camera angle
        

class H1_2TaskTransferCfgPPO(LeggedRobotCfgPPO):
    seed = 5
    runner_class_name = 'OnPolicyRunner'   # DWLOnPolicyRunner

    class policy:
        init_noise_std = 1.0
        actor_hidden_dims = [512, 256, 128]
        critic_hidden_dims = [768, 256, 128]
        # HRL
        num_dofs = H1_2TaskTransferCfg.env.num_actions
        frame_stack = H1_2TaskTransferCfg.env.frame_stack
        command_dim = H1_2TaskTransferCfg.env.command_dim
        # Expert skills
        skill_dict = {
            'h1_2_walking': {
                "experiment_name": "h1_2_walking",
                "load_run": "0000_best",
                "checkpoint": -1,
                "low_high": (-2, 2)
            },
            'h1_2_reaching': {
                "experiment_name": "h1_2_reaching",
                "load_run": "0000_best",
                "checkpoint": -1,
                "low_high": (-1, 1)
            },
            # 'h1_2_squatting': {
            #     "experiment_name": "h1_2_squatting",
            #     "load_run": "0000_best",
            #     "checkpoint": -1,
            #     "low_high": (-1, 1)
            # },
            # 'h1_2_stepping': {
            #     "experiment_name": "h1_2_stepping",
            #     "load_run": "0000_best",
            #     "checkpoint": -1,
            #     "low_high": (-1, 1)
            # }
        }

    class algorithm(LeggedRobotCfgPPO.algorithm):
        entropy_coef = 0.001
        learning_rate = 1e-5
        num_learning_epochs = 2
        gamma = 0.994
        lam = 0.9
        num_mini_batches = 4

    class runner:
        policy_class_name = 'ActorCriticHierarchical'
        algorithm_class_name = 'PPO'
        num_steps_per_env = 60  # per iteration
        max_iterations = 70001 # 3001  # number of policy updates

        # logging
        save_interval = 1000  # check for potential saves every this many iterations
        experiment_name = 'h1_2_task_transfer'
        run_name = ''
        # load and resume
        resume = False
        load_run = -1  # -1 = last run
        checkpoint = -1  # -1 = last saved model
        resume_path = None  # updated from load_run and ckpt
