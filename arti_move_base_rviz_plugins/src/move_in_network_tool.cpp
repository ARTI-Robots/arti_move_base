/// \file
/// \author Alexander Buchegger
/// \date 2021-09-12
/// \copyright ARTI - Autonomous Robot Technology GmbH. All rights reserved.
#include <arti_move_base_rviz_plugins/move_in_network_tool.h>
#include <algorithm>
#include <functional>
#include <pluginlib/class_list_macros.h>
#include <QKeyEvent>
#include <ros/console.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/display_context.h>
#include <rviz/load_resource.h>
#include <rviz/properties/string_property.h>
#include <rviz/tool_manager.h>
#include <rviz/view_controller.h>
#include <rviz/viewport_mouse_event.h>

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-parameter"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

#include <rviz/render_panel.h>

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

namespace arti_move_base_rviz_plugins
{

MoveInNetworkTool::MoveInNetworkTool()
  : action_namespace_property_(new rviz::StringProperty("Action Namespace", "/move_in_network",
                                                        "Topic namespace of the Move In Network action",
                                                        getPropertyContainer(),
                                                        SLOT(onActionNamespacePropertyChanged()), this)),
    cancel_icon_(rviz::loadPixmap("package://arti_move_base_rviz_plugins/icons/actions/cancel.png")),
    action_done_callback_(std::bind(&MoveInNetworkTool::processActionDone, this, std::placeholders::_1))
{
}

MoveInNetworkTool::~MoveInNetworkTool() = default;

void MoveInNetworkTool::onInitialize()
{
  send_goal_icon_ = icon_;
  send_goal_cursor_ = cursor_;
  PoseTool::onInitialize();
  setName("Move In Network");
  onActionNamespacePropertyChanged();
}

void MoveInNetworkTool::activate()
{
  if (action_active_)
  {
    if (action_client_)
    {
      action_client_->cancelAllGoals();
      // Deactivate tool directly after use:
      Q_EMIT close();
    }
  }
  else
  {
    PoseTool::activate();
  }
}

int MoveInNetworkTool::processMouseEvent(rviz::ViewportMouseEvent& event)
{
  if (event.modifiers == Qt::NoModifier
      && (event.acting_button == Qt::LeftButton
          || (event.acting_button == Qt::NoButton && (event.buttons_down & Qt::LeftButton))))
  {
    return PoseTool::processMouseEvent(event);
  }

  if (state_ == Orientation)
  {
    // Reset tool state on additional button press:
    state_ = Position;
    arrow_->getSceneNode()->setVisible(false);
    return Render;
  }

  rviz::ViewController* const view_controller = event.panel->getViewController();
  if (view_controller && (!(event.buttons_down & Qt::LeftButton) || event.modifiers != Qt::NoModifier))
  {
    view_controller->handleMouseEvent(event);
    setCursor((event.modifiers == Qt::NoModifier && event.buttons_down == Qt::NoButton)
              ? send_goal_cursor_ : view_controller->getCursor());
  }
  return 0;
}

int MoveInNetworkTool::processKeyEvent(QKeyEvent* event, rviz::RenderPanel* panel)
{
  if (event->type() == QEvent::KeyPress && state_ == Orientation)
  {
    // Reset tool state on key press:
    state_ = Position;
    arrow_->getSceneNode()->setVisible(false);
  }
  return PoseTool::processKeyEvent(event, panel);
}

void MoveInNetworkTool::onActionNamespacePropertyChanged()
{
  const std::string action_namespace = action_namespace_property_->getStdString();
  if (!action_namespace.empty())
  {
    try
    {
      node_handle_ = ros::NodeHandle(action_namespace);
      action_status_subscriber_ = node_handle_.subscribe("status", 1, &MoveInNetworkTool::processActionStatus, this);
      action_client_.emplace(node_handle_, "", false);
    }
    catch (const ros::Exception& e)
    {
      ROS_ERROR_STREAM("failed to create action client and status subscriber: " << e.what());
    }
  }
  else
  {
    ROS_WARN_STREAM("no action namespace given for Cancel Action tool");
    action_status_subscriber_.shutdown();
    action_client_.reset();
  }
}

void MoveInNetworkTool::onPoseSet(double x, double y, double theta)
{
  if (action_client_)
  {
    arti_move_base_msgs::MoveInNetworkGoal goal;
    goal.target_pose.header.frame_id = context_->getFixedFrame().toStdString();
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.point.x.value = x;
    goal.target_pose.pose.point.y.value = y;
    goal.target_pose.pose.theta.value = theta;
    action_client_->sendGoal(goal, action_done_callback_);
  }
}

void MoveInNetworkTool::processActionDone(const actionlib::SimpleClientGoalState& state)
{
  setStatus(getName() + ": action finished with state " + QString::fromStdString(state.toString()));
}

void MoveInNetworkTool::processActionStatus(const actionlib_msgs::GoalStatusArrayConstPtr& action_status)
{
  const bool action_active = std::any_of(action_status->status_list.begin(), action_status->status_list.end(),
                                         [](const actionlib_msgs::GoalStatus& goal_status) -> bool {
                                           return goal_status.status == actionlib_msgs::GoalStatus::PENDING
                                                  || goal_status.status == actionlib_msgs::GoalStatus::ACTIVE;
                                         });

  if (action_active_ != action_active)
  {
    action_active_ = action_active;
    setIcon(action_active_ ? cancel_icon_ : send_goal_icon_);
    // Notify tool manager about the change:
    context_->getToolManager()->refreshTool(this);
  }
}

}

PLUGINLIB_EXPORT_CLASS(arti_move_base_rviz_plugins::MoveInNetworkTool, rviz::Tool)
