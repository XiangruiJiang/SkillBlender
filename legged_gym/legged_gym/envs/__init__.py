# SPDX-FileCopyrightText: Copyright (c) 2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
# Copyright (c) 2021 ETH Zurich, Nikita Rudin

from legged_gym import LEGGED_GYM_ROOT_DIR, LEGGED_GYM_ENVS_DIR

# H1
## Primitive Skills
### H1 Walking
from .h1.h1_walking.h1_walking import H1Walking
from .h1.h1_walking.h1_walking_config import H1WalkingCfg, H1WalkingCfgPPO
### H1 Reaching
from .h1.h1_reaching.h1_reaching import H1Reaching
from .h1.h1_reaching.h1_reaching_config import H1ReachingCfg, H1ReachingCfgPPO
### H1 Stepping
from .h1.h1_stepping.h1_stepping import H1Stepping
from .h1.h1_stepping.h1_stepping_config import H1SteppingCfg, H1SteppingCfgPPO
### H1 Squatting
from .h1.h1_squatting.h1_squatting import H1Squatting
from .h1.h1_squatting.h1_squatting_config import H1SquattingCfg, H1SquattingCfgPPO
## High-Level Tasks
### H1 BoxPush
from .h1.h1_task_box.h1_task_box import H1TaskBox
from .h1.h1_task_box.h1_task_box_config import H1TaskBoxCfg, H1TaskBoxCfgPPO
### H1 FarReach
from .h1.h1_task_reach.h1_task_reach import H1TaskReach
from .h1.h1_task_reach.h1_task_reach_config import H1TaskReachCfg, H1TaskReachCfgPPO
### H1 ButtonPress
from .h1.h1_task_button.h1_task_button import H1TaskButton
from .h1.h1_task_button.h1_task_button_config import H1TaskButtonCfg, H1TaskButtonCfgPPO
### H1 PackageLift
from .h1.h1_task_lift.h1_task_lift import H1TaskLift
from .h1.h1_task_lift.h1_task_lift_config import H1TaskLiftCfg, H1TaskLiftCfgPPO
### H1 FootballShoot
from .h1.h1_task_ball.h1_task_ball import H1TaskBall
from .h1.h1_task_ball.h1_task_ball_config import H1TaskBallCfg, H1TaskBallCfgPPO
### H1 PackageCarry
from .h1.h1_task_carry.h1_task_carry import H1TaskCarry
from .h1.h1_task_carry.h1_task_carry_config import H1TaskCarryCfg, H1TaskCarryCfgPPO
### H1 BoxTransfer
from .h1.h1_task_transfer.h1_task_transfer import H1TaskTransfer
from .h1.h1_task_transfer.h1_task_transfer_config import H1TaskTransferCfg, H1TaskTransferCfgPPO
### H1 CabinetClose
from .h1.h1_task_cabinet.h1_task_cabinet import H1TaskCabinet
from .h1.h1_task_cabinet.h1_task_cabinet_config import H1TaskCabinetCfg, H1TaskCabinetCfgPPO
# G1
## Primitive Skills
### G1 Walking
from .g1.g1_walking.g1_walking import G1Walking
from .g1.g1_walking.g1_walking_config import G1WalkingCfg, G1WalkingCfgPPO
### G1 Reaching
from .g1.g1_reaching.g1_reaching import G1Reaching
from .g1.g1_reaching.g1_reaching_config import G1ReachingCfg, G1ReachingCfgPPO
### G1 Stepping
from .g1.g1_stepping.g1_stepping import G1Stepping
from .g1.g1_stepping.g1_stepping_config import G1SteppingCfg, G1SteppingCfgPPO
### G1 Squatting
from .g1.g1_squatting.g1_squatting import G1Squatting
from .g1.g1_squatting.g1_squatting_config import G1SquattingCfg, G1SquattingCfgPPO
## High-Level Tasks
### G1 BoxPush
from .g1.g1_task_box.g1_task_box import G1TaskBox
from .g1.g1_task_box.g1_task_box_config import G1TaskBoxCfg, G1TaskBoxCfgPPO
### G1 FarReach
from .g1.g1_task_reach.g1_task_reach import G1TaskReach
from .g1.g1_task_reach.g1_task_reach_config import G1TaskReachCfg, G1TaskReachCfgPPO
### G1 ButtonPress
from .g1.g1_task_button.g1_task_button import G1TaskButton
from .g1.g1_task_button.g1_task_button_config import G1TaskButtonCfg, G1TaskButtonCfgPPO
### G1 PackageLift
from .g1.g1_task_lift.g1_task_lift import G1TaskLift
from .g1.g1_task_lift.g1_task_lift_config import G1TaskLiftCfg, G1TaskLiftCfgPPO
### G1 FootballShoot
from .g1.g1_task_ball.g1_task_ball import G1TaskBall
from .g1.g1_task_ball.g1_task_ball_config import G1TaskBallCfg, G1TaskBallCfgPPO
### G1 PackageCarry
from .g1.g1_task_carry.g1_task_carry import G1TaskCarry
from .g1.g1_task_carry.g1_task_carry_config import G1TaskCarryCfg, G1TaskCarryCfgPPO
### G1 BoxTransfer
from .g1.g1_task_transfer.g1_task_transfer import G1TaskTransfer
from .g1.g1_task_transfer.g1_task_transfer_config import G1TaskTransferCfg, G1TaskTransferCfgPPO
### G1 CabinetClose
from .g1.g1_task_cabinet.g1_task_cabinet import G1TaskCabinet
from .g1.g1_task_cabinet.g1_task_cabinet_config import G1TaskCabinetCfg, G1TaskCabinetCfgPPO
# H1_2
## Primitive Skills
### H1_2 Walking
from .h1_2.h1_2_walking.h1_2_walking import H1_2Walking
from .h1_2.h1_2_walking.h1_2_walking_config import H1_2WalkingCfg, H1_2WalkingCfgPPO
### H1_2 Reaching
from .h1_2.h1_2_reaching.h1_2_reaching import H1_2Reaching
from .h1_2.h1_2_reaching.h1_2_reaching_config import H1_2ReachingCfg, H1_2ReachingCfgPPO
### H1_2 Stepping
from .h1_2.h1_2_stepping.h1_2_stepping import H1_2Stepping
from .h1_2.h1_2_stepping.h1_2_stepping_config import H1_2SteppingCfg, H1_2SteppingCfgPPO
### H1_2 Squatting
from .h1_2.h1_2_squatting.h1_2_squatting import H1_2Squatting
from .h1_2.h1_2_squatting.h1_2_squatting_config import H1_2SquattingCfg, H1_2SquattingCfgPPO
## High-Level Tasks
### H1_2 BoxPush
from .h1_2.h1_2_task_box.h1_2_task_box import H1_2TaskBox
from .h1_2.h1_2_task_box.h1_2_task_box_config import H1_2TaskBoxCfg, H1_2TaskBoxCfgPPO
### H1_2 FarReach
from .h1_2.h1_2_task_reach.h1_2_task_reach import H1_2TaskReach
from .h1_2.h1_2_task_reach.h1_2_task_reach_config import H1_2TaskReachCfg, H1_2TaskReachCfgPPO
### H1_2 ButtonPress
from .h1_2.h1_2_task_button.h1_2_task_button import H1_2TaskButton
from .h1_2.h1_2_task_button.h1_2_task_button_config import H1_2TaskButtonCfg, H1_2TaskButtonCfgPPO
### H1_2 PackageLift
from .h1_2.h1_2_task_lift.h1_2_task_lift import H1_2TaskLift
from .h1_2.h1_2_task_lift.h1_2_task_lift_config import H1_2TaskLiftCfg, H1_2TaskLiftCfgPPO
### H1_2 FootballShoot
from .h1_2.h1_2_task_ball.h1_2_task_ball import H1_2TaskBall
from .h1_2.h1_2_task_ball.h1_2_task_ball_config import H1_2TaskBallCfg, H1_2TaskBallCfgPPO
### H1_2 PackageCarry
from .h1_2.h1_2_task_carry.h1_2_task_carry import H1_2TaskCarry
from .h1_2.h1_2_task_carry.h1_2_task_carry_config import H1_2TaskCarryCfg, H1_2TaskCarryCfgPPO
### H1_2 BoxTransfer
from .h1_2.h1_2_task_transfer.h1_2_task_transfer import H1_2TaskTransfer
from .h1_2.h1_2_task_transfer.h1_2_task_transfer_config import H1_2TaskTransferCfg, H1_2TaskTransferCfgPPO
### H1_2 CabinetClose
from .h1_2.h1_2_task_cabinet.h1_2_task_cabinet import H1_2TaskCabinet
from .h1_2.h1_2_task_cabinet.h1_2_task_cabinet_config import H1_2TaskCabinetCfg, H1_2TaskCabinetCfgPPO

# T1
from .t1.t1 import T1
from .t1.t1_config import T1RoughCfg, T1RoughCfgPPO

from legged_gym.utils.task_registry import task_registry

task_registry.register( "h1_walking", H1Walking, H1WalkingCfg(), H1WalkingCfgPPO(), 'h1/h1_walking')
task_registry.register( "h1_reaching", H1Reaching, H1ReachingCfg(), H1ReachingCfgPPO(), 'h1/h1_reaching')
task_registry.register( "h1_stepping", H1Stepping, H1SteppingCfg(), H1SteppingCfgPPO(), 'h1/h1_stepping')
task_registry.register( "h1_squatting", H1Squatting, H1SquattingCfg(), H1SquattingCfgPPO(), 'h1/h1_squatting')
task_registry.register( "h1_task_box", H1TaskBox, H1TaskBoxCfg(), H1TaskBoxCfgPPO(), 'h1/h1_task_box')
task_registry.register( "h1_task_reach", H1TaskReach, H1TaskReachCfg(), H1TaskReachCfgPPO(), 'h1/h1_task_reach')
task_registry.register( "h1_task_button", H1TaskButton, H1TaskButtonCfg(), H1TaskButtonCfgPPO(), 'h1/h1_task_button')
task_registry.register( "h1_task_lift", H1TaskLift, H1TaskLiftCfg(), H1TaskLiftCfgPPO(), 'h1/h1_task_lift')
task_registry.register( "h1_task_ball", H1TaskBall, H1TaskBallCfg(), H1TaskBallCfgPPO(), 'h1/h1_task_ball')
task_registry.register( "h1_task_carry", H1TaskCarry, H1TaskCarryCfg(), H1TaskCarryCfgPPO(), 'h1/h1_task_carry')
task_registry.register( "h1_task_transfer", H1TaskTransfer, H1TaskTransferCfg(), H1TaskTransferCfgPPO(), 'h1/h1_task_transfer')
task_registry.register( "h1_task_cabinet", H1TaskCabinet, H1TaskCabinetCfg(), H1TaskCabinetCfgPPO(), 'h1/h1_task_cabinet')
task_registry.register( "g1_walking", G1Walking, G1WalkingCfg(), G1WalkingCfgPPO(), 'g1/g1_walking')
task_registry.register( "g1_reaching", G1Reaching, G1ReachingCfg(), G1ReachingCfgPPO(), 'g1/g1_reaching')
task_registry.register( "g1_stepping", G1Stepping, G1SteppingCfg(), G1SteppingCfgPPO(), 'g1/g1_stepping')
task_registry.register( "g1_squatting", G1Squatting, G1SquattingCfg(), G1SquattingCfgPPO(), 'g1/g1_squatting')
task_registry.register( "g1_task_box", G1TaskBox, G1TaskBoxCfg(), G1TaskBoxCfgPPO(), 'g1/g1_task_box')
task_registry.register( "g1_task_reach", G1TaskReach, G1TaskReachCfg(), G1TaskReachCfgPPO(), 'g1/g1_task_reach')
task_registry.register( "g1_task_button", G1TaskButton, G1TaskButtonCfg(), G1TaskButtonCfgPPO(), 'g1/g1_task_button')
task_registry.register( "g1_task_lift", G1TaskLift, G1TaskLiftCfg(), G1TaskLiftCfgPPO(), 'g1/g1_task_lift')
task_registry.register( "g1_task_ball", G1TaskBall, G1TaskBallCfg(), G1TaskBallCfgPPO(), 'g1/g1_task_ball')
task_registry.register( "g1_task_carry", G1TaskCarry, G1TaskCarryCfg(), G1TaskCarryCfgPPO(), 'g1/g1_task_carry')
task_registry.register( "g1_task_transfer", G1TaskTransfer, G1TaskTransferCfg(), G1TaskTransferCfgPPO(), 'g1/g1_task_transfer')
task_registry.register( "g1_task_cabinet", G1TaskCabinet, G1TaskCabinetCfg(), G1TaskCabinetCfgPPO(), 'g1/g1_task_cabinet')
task_registry.register( "h1_2_walking", H1_2Walking, H1_2WalkingCfg(), H1_2WalkingCfgPPO(), 'h1_2/h1_2_walking')
task_registry.register( "h1_2_reaching", H1_2Reaching, H1_2ReachingCfg(), H1_2ReachingCfgPPO(), 'h1_2/h1_2_reaching')
task_registry.register( "h1_2_stepping", H1_2Stepping, H1_2SteppingCfg(), H1_2SteppingCfgPPO(), 'h1_2/h1_2_stepping')
task_registry.register( "h1_2_squatting", H1_2Squatting, H1_2SquattingCfg(), H1_2SquattingCfgPPO(), 'h1_2/h1_2_squatting')
task_registry.register( "h1_2_task_box", H1_2TaskBox, H1_2TaskBoxCfg(), H1_2TaskBoxCfgPPO(), 'h1_2/h1_2_task_box')
task_registry.register( "h1_2_task_reach", H1_2TaskReach, H1_2TaskReachCfg(), H1_2TaskReachCfgPPO(), 'h1_2/h1_2_task_reach')
task_registry.register( "h1_2_task_button", H1_2TaskButton, H1_2TaskButtonCfg(), H1_2TaskButtonCfgPPO(), 'h1_2/h1_2_task_button')
task_registry.register( "h1_2_task_lift", H1_2TaskLift, H1_2TaskLiftCfg(), H1_2TaskLiftCfgPPO(), 'h1_2/h1_2_task_lift')
task_registry.register( "h1_2_task_ball", H1_2TaskBall, H1_2TaskBallCfg(), H1_2TaskBallCfgPPO(), 'h1_2/h1_2_task_ball')
task_registry.register( "h1_2_task_carry", H1_2TaskCarry, H1_2TaskCarryCfg(), H1_2TaskCarryCfgPPO(), 'h1_2/h1_2_task_carry')
task_registry.register( "h1_2_task_transfer", H1_2TaskTransfer, H1_2TaskTransferCfg(), H1_2TaskTransferCfgPPO(), 'h1_2/h1_2_task_transfer')
task_registry.register( "h1_2_task_cabinet", H1_2TaskCabinet, H1_2TaskCabinetCfg(), H1_2TaskCabinetCfgPPO(), 'h1_2/h1_2_task_cabinet')
task_registry.register("t1_walking", T1, T1RoughCfg(), T1RoughCfgPPO(), 't1')
