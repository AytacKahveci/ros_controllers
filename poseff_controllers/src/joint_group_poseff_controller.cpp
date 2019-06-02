/// \author Aytaç Kahveci
#include <poseff_controllers/joint_group_poseff_controller.h>
#include <pluginlib/class_list_macros.h>

void forward_command_controller::ForwardJointGroupPosEffCommandController::starting(const ros::Time& time)
{
    std::vector<double> & commands = *commands_buffer_.readFromRT();

    for(size_t i=0; i < n_joints_; i++)
    {
        commands[i] = joints_[i].getCommandPosition();
        commands[i + n_joints_] = joints_[i].getCommandEffort();
    }
}

PLUGINLIB_EXPORT_CLASS(poseff_controllers::JointGroupPosEffController, controller_interface::ControllerBase);
