/// \file
/// \author Alexander Buchegger
/// \date 2021-09-12
/// \copyright ARTI - Autonomous Robot Technology GmbH. All rights reserved.
#ifndef ARTI_MOVE_BASE_RVIZ_PLUGINS_MOVE_IN_NETWORK_TOOL_H
#define ARTI_MOVE_BASE_RVIZ_PLUGINS_MOVE_IN_NETWORK_TOOL_H

#ifndef Q_MOC_RUN // See: https://bugreports.qt-project.org/browse/QTBUG-22829

#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <arti_move_base_msgs/MoveInNetworkAction.h>
#include <boost/optional.hpp>
#include <QObject>
#include <QIcon>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <rviz/default_plugin/tools/pose_tool.h>

#endif

namespace rviz
{
class StringProperty;
}

namespace arti_move_base_rviz_plugins
{

class MoveInNetworkTool : public rviz::PoseTool
{
Q_OBJECT
public:
  MoveInNetworkTool();
  ~MoveInNetworkTool() override;
  void activate() override;
  int processMouseEvent(rviz::ViewportMouseEvent& event) override;
  int processKeyEvent(QKeyEvent* event, rviz::RenderPanel* panel) override;

protected
  Q_SLOTS:
  void onActionNamespacePropertyChanged();

protected:
  using MoveInNetworkActionClient = actionlib::SimpleActionClient<arti_move_base_msgs::MoveInNetworkAction>;

  void onInitialize() override;
  void onPoseSet(double x, double y, double theta) override;

  void processActionDone(const actionlib::SimpleClientGoalState& state);
  void processActionStatus(const actionlib_msgs::GoalStatusArrayConstPtr& action_status);

  rviz::StringProperty* action_namespace_property_;
  QIcon send_goal_icon_;
  QCursor send_goal_cursor_;
  QIcon cancel_icon_;
  ros::NodeHandle node_handle_;
  boost::optional<MoveInNetworkActionClient> action_client_;
  MoveInNetworkActionClient::SimpleDoneCallback action_done_callback_;

  ros::Subscriber action_status_subscriber_;
  bool action_active_{false};
};

}

#endif // ARTI_MOVE_BASE_RVIZ_PLUGINS_MOVE_IN_NETWORK_TOOL_H
