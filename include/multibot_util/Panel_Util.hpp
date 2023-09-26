#pragma once;

#include "multibot_ros2_interface/srv/connection.hpp"
#include "multibot_ros2_interface/srv/disconnection.hpp"
#include "multibot_ros2_interface/srv/mode_selection.hpp"

namespace PanelUtil
{
    using Connection = multibot_ros2_interface::srv::Connection;
    using Disconnection = multibot_ros2_interface::srv::Disconnection;
    using ModeSelection = multibot_ros2_interface::srv::ModeSelection;

    enum Tab
    {
        DASHBOARD,
        ROBOT
    }; // enum Tab

    enum Request
    {
        NO_REQUEST,
        PLAN_REQUEST,
        START_REQUEST
    }; // enum Request

    enum Mode
    {
        REMOTE,
        MANUAL,
        AUTO
    }; // enum Mode

    enum PlanState
    {
        READY,
        PLANNING,
        SUCCESS,
        FAIL
    }; // enum PlanState

    typedef Request Msg;
} // namespace PanelUtil;