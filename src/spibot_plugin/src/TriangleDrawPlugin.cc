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
#include <nav_msgs/Odometry.h>

namespace gazebo
{
    class TriangleDrawPlugin : public VisualPlugin
    {
    public:
        TriangleDrawPlugin() : triangle(NULL) {}
        ~TriangleDrawPlugin()
        {
            this->visual->DeleteDynamicLine(this->triangle);
        }

        void Load(rendering::VisualPtr _parent, sdf::ElementPtr _sdf)
        {
            this->visual = _parent;
            this->triangle = this->visual->CreateDynamicLine(rendering::RENDERING_TRIANGLE_FAN);
            this->triangle->setMaterial("Gazebo/Green");
            this->triangle->setVisibilityFlags(GZ_VISIBILITY_GUI);
            this->visual->SetVisible(true);

            this->rosnode = new ros::NodeHandle("triangle_draw_plugin");
            this->update_connection = event::Events::ConnectPreRender(boost::bind(&TriangleDrawPlugin::OnUpdate, this));
            ROS_INFO("TriangleDrawPlugin loaded.");
        }

        void OnUpdate()
        {
            this->triangle->Clear();
            this->triangle->AddPoint(ignition::math::Vector3d(0, 0, 0), ignition::math::Color(0, 1, 0, 1.0));
            this->triangle->AddPoint(ignition::math::Vector3d(x, y, z), ignition::math::Color(0, 1, 0, 1.0));
            this->triangle->AddPoint(ignition::math::Vector3d(x/2, y/3, z/2), ignition::math::Color(0, 1, 0, 1.0));
        }

    private:
        ros::NodeHandle *rosnode;
        rendering::VisualPtr visual;
        rendering::DynamicLines *triangle;
        event::ConnectionPtr update_connection;
        double x = 1.0, y = 1.0, z = 1.0;
    };
    GZ_REGISTER_VISUAL_PLUGIN(TriangleDrawPlugin)
}