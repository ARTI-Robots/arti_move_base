/*
Created by clemens on 6/6/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ARTI_MOVE_BASE_PIPELINE_BASE_H
#define ARTI_MOVE_BASE_PIPELINE_BASE_H

#include <arti_nav_core_msgs/Movement2DGoalWithConstraints.h>
#include <arti_nav_core_msgs/Pose2DStampedWithLimits.h>
#include <arti_nav_core_msgs/Trajectory2DWithLimits.h>

namespace arti_move_base
{
class PipelineBuilder;

class PipelineBase
{
public:
  virtual void createPipeline(PipelineBuilder &builder) = 0;
  virtual void cancel() = 0;
  virtual void setInput(const arti_nav_core_msgs::Movement2DGoalWithConstraints &goal) = 0;
  virtual void setInput(const arti_nav_core_msgs::Pose2DStampedWithLimits& goal) = 0;
  virtual void setInput(const arti_nav_core_msgs::Trajectory2DWithLimits& goal) = 0;
};

}
#endif //ARTI_MOVE_BASE_PIPELINE_BASE_H
