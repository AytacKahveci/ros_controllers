#include <dynamixel_current_controller/dynamixelHardwareInterfaceCurrent.h>
#include <pluginlib/class_list_macros.h>
#define PI 3.14159265
namespace dynamixel_current_controller
{
    DynamixelCurrentController::DynamixelCurrentController()
        : loop_count_(0)
    {}

    DynamixelCurrentController::~DynamixelCurrentController()
    {
        sub_command_.shutdown();
    }

    bool DynamixelCurrentController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
    {
        if(!n.getParam("joints",joint_names_))
        {
            ROS_ERROR("No joint given (namespace: %s)", n.getNamespace().c_str());
            return false;
        }
        n_joints_ = joint_names_.size();

        if(!pid_controller_.init(ros::NodeHandle(n,"pid")))
        {
            return false;
        }

        sub_command_ = n.subscribe("command",1,&DynamixelCurrentController::setCommandCB, this);

        for(size_t i=0; i<n_joints_; ++i)
        {
            joints_.push_back(robot->getHandle(joint_names_[i]));
        }

        return true;
    }

    void DynamixelCurrentController::setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min, const bool &antiwindup)
    {
        pid_controller_.setGains(p,i,d,i_max,i_min,antiwindup);
    }

    void DynamixelCurrentController::getGains(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup)
    {
        pid_controller_.getGains(p,i,d,i_max,i_min,antiwindup);
    }

    void DynamixelCurrentController::getGains(double &p, double &i, double &d, double &i_max, double &i_min)
    {
        pid_controller_.getGains(p,i,d,i_max,i_min);
    }

    void DynamixelCurrentController::printDebug()
    {
        pid_controller_.printValues();
    }

    void DynamixelCurrentController::starting(const ros::Time& time)
    {
        for (size_t i=0; i<n_joints_; i++)
        {
            commands_.push_back(0);
        }

        commands_buffer_.initRT(commands_);
        pid_controller_.reset();
    }

    void DynamixelCurrentController::update(const ros::Time& time, const ros::Duration& period)
    {
        commands_ = *(commands_buffer_.readFromRT());
        ROS_INFO("commands_ is %f",commands_[0]);
        for(size_t i=0; i<n_joints_; i++)
        {
            theta1 = (joints_[i].getPosition()*0.088)*(PI/180);
            theta2 = asin(0.171*sin(theta1) + 0.8359);
            x3 = 0.0149*cos(theta1) + 0.0873*cos(theta2);
            ROS_INFO("X3 value is %f",x3);
            error = commands_[i] - x3;
            commanded_effort = pid_controller_.computeCommand(error,period);
            ROS_INFO("commanded_effort is %f",commanded_effort);
            current_ = commanded_effort * torque_to_current_value_ratio;
            joints_[i].setCommand(current_);
        }

    }

    void DynamixelCurrentController::setCommandCB(const std_msgs::Float64MultiArrayConstPtr& msg)
    {
        if(msg->data.size() != n_joints_)
        {
            ROS_ERROR_STREAM("Dimensions of command (" << msg->data.size() << ") does not match number of joints (" << n_joints_ << ")! Not executing!");
            return;
        }

        commands_buffer_.writeFromNonRT(msg->data);
    }
}

PLUGINLIB_EXPORT_CLASS(dynamixel_current_controller::DynamixelCurrentController, controller_interface::ControllerBase)
