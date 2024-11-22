/************************************************************************
 * @file draw_traj_plugin.cc
 * @brief Draw trajectory of the robot in Gazebo
 * Modefied from Unitree Robotics
 *
 * @author Luchuanzhao
 * @version 1.0.0
 * @date 2023.6.26
 ************************************************************************/

/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/
#include <ignition/math/Color.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/Scene.hh"
#include <boost/bind.hpp>
#include "std_msgs/Float32MultiArray.h"
#include <geometry_msgs/WrenchStamped.h>

namespace gazebo
{
    class TriangleDrawPlugin : public VisualPlugin
    {
    public:
        TriangleDrawPlugin() : line(NULL) {}
        ~TriangleDrawPlugin()
        {
            this->visual->DeleteDynamicLine(this->line);
        }

        void Load(rendering::VisualPtr _parent, sdf::ElementPtr _sdf)
        {
            this->visual = _parent; // parent就是xacro文件中的gazebo reference
            this->visual_namespace = "visual/";

            if (!_sdf->HasElement("topicName"))
            { // 这里订阅的是关节的位置真值话题
                ROS_INFO("Force draw plugin missing <topicName>, defaults to /default_force_draw");
                this->topic_name = "/default_force_draw";
            }
            else
            {
                this->topic_name = _sdf->Get<std::string>("topicName");
            }
            if (!ros::isInitialized())
            {
                int argc = 0;
                char **argv = NULL;
                ros::init(argc, argv, "gazebo_visual", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
            }

            this->line = this->visual->CreateDynamicLine(rendering::RENDERING_LINE_STRIP);
#if GAZEBO_MAJOR_VERSION >= 10
            // https://github.com/osrf/gazebo/blob/gazebo11/Migration.md#gazebo-8x-to-9x
            // gazebo 9 deprecations removed on gazebo 10
            // https://github.com/osrf/gazebo/commit/0bd72b9500e7377c873e32d25b8db772e782bd6f
            this->line->AddPoint(ignition::math::Vector3d(0, 0, 0), ignition::math::Color(0, 1, 0, 1.0));
            this->line->AddPoint(ignition::math::Vector3d(1, 1, 1), ignition::math::Color(0, 1, 0, 1.0));
#else
            this->line->AddPoint(ignition::math::Vector3d(0, 0, 0), ignition::math::Color(0, 1, 0, 1.0));
            this->line->AddPoint(ignition::math::Vector3d(1, 1, 1), ignition::math::Color(0, 1, 0, 1.0));
#endif
            this->line->setMaterial("Gazebo/Green");
            this->line->setVisibilityFlags(GZ_VISIBILITY_GUI);
            this->visual->SetVisible(true);

            this->rosnode = new ros::NodeHandle(this->visual_namespace);
            this->force_sub = this->rosnode->subscribe(this->topic_name, 30, &TriangleDrawPlugin::GetForceCallback, this);
            this->update_connection = event::Events::ConnectPreRender(boost::bind(&TriangleDrawPlugin::OnUpdate, this));
            ROS_INFO("Load %s Draw Force plugin.", this->topic_name.c_str());
        }
        // 绘制线条在OnUpdate和GetForceCallback函数里面修改就行
        void OnUpdate()
        {
            this->line->Clear();
            // 点1连线到点2，点2连线到点3，点3重新连线到点1
            this->line->AddPoint(ignition::math::Vector3d(Fx1, Fy1, Fz1), ignition::math::Color(0, 0, 1, 1.0));
            this->line->AddPoint(ignition::math::Vector3d(Fx2, Fy2, Fz2), ignition::math::Color(0, 0, 1, 1.0));
            this->line->AddPoint(ignition::math::Vector3d(Fx3, Fy3, Fz3), ignition::math::Color(0, 0, 1, 1.0));
            this->line->AddPoint(ignition::math::Vector3d(Fx1, Fy1, Fz1), ignition::math::Color(0, 0, 1, 1.0));//
        }

        void GetForceCallback(const std_msgs::Float32MultiArray &data)
        {
            Fx1 = data.data[0] * 17;
            Fy1 = data.data[1] * 17;
            Fz1 = data.data[2] * 20;
            Fx2 = data.data[3] * 17;
            Fy2 = data.data[4] * 17;
            Fz2 = data.data[5] * 20;
            Fx3 = data.data[6] * 17;
            Fy3 = data.data[7] * 17;
            Fz3 = data.data[8] * 20;
        }

    private:
        ros::NodeHandle *rosnode;
        std::string topic_name;
        rendering::VisualPtr visual;
        rendering::DynamicLines *line;
        std::string visual_namespace;
        ros::Subscriber force_sub;
        double Fx1 = 0, Fy1 = 0, Fz1 = 0;
        double Fx2 = 0, Fy2 = 0, Fz2 = 0;
        double Fx3 = 0, Fy3 = 0, Fz3 = 0;
        event::ConnectionPtr update_connection;
    };
    GZ_REGISTER_VISUAL_PLUGIN(TriangleDrawPlugin)
}
