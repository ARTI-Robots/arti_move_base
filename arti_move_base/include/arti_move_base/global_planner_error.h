/*
Created by clemens on 31.03.20.
This file is part of the software provided by ARTI
Copyright (c) 2020, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ARTI_MOVE_BASE_GLOBAL_PLANNER_ERROR_H
#define ARTI_MOVE_BASE_GLOBAL_PLANNER_ERROR_H

#include <arti_nav_core/base_global_planner.h>

namespace arti_move_base
{
struct GlobalPlannerError
{
  arti_nav_core::BaseGlobalPlanner::BaseGlobalPlannerErrorEnum error_enum;
  arti_nav_core_msgs::Pose2DWithLimits error_pose_a;
  arti_nav_core_msgs::Pose2DWithLimits error_pose_b;
};
}

#endif //ARTI_MOVE_BASE_GLOBAL_PLANNER_ERROR_H
