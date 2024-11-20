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
#include <geometry_msgs/Point.h>

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
            this->visual = _parent;
            this->visual_namespace = "visual/";

            if (!_sdf->HasElement("topicName"))
            {
                ROS_INFO("Triangle draw plugin missing <topicName>, defaults to /default_triangle_points");
                this->topic_name = "/default_triangle_points";
            }
            else
            {
                this->topic_name = _sdf->Get<std::string>("topicName");
            }

            if (!ros::isInitialized())
            {
                int argc = 0;
                char **argv = NULL;
                ros::init(argc, argv, "gazebo_triangle_visual", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
            }

            this->line = this->visual->CreateDynamicLine(rendering::RENDERING_LINE_STRIP);
            this->line->setMaterial("Gazebo/Green");
            this->line->setVisibilityFlags(GZ_VISIBILITY_GUI);
            this->visual->SetVisible(true);

            this->rosnode = new ros::NodeHandle(this->visual_namespace);
            this->point_sub = this->rosnode->subscribe(this->topic_name, 1, &TriangleDrawPlugin::GetPointsCallback, this);
            this->update_connection = event::Events::ConnectPreRender(boost::bind(&TriangleDrawPlugin::OnUpdate, this));
            ROS_INFO("Load %s Triangle Draw plugin.", this->topic_name.c_str());
        }

        void OnUpdate()
        {
            // Clear previous points
            this->line->Clear();
            // Add points to form a triangle
            for (const auto& point : points)
            {
                this->line->AddPoint(ignition::math::Vector3d(point.x, point.y, point.z), ignition::math::Color(1, 0, 0, 1.0));
            }
            // Close the triangle
            if (points.size() == 3)
            {
                this->line->AddPoint(ignition::math::Vector3d(points[0].x, points[0].y, points[0].z), ignition::math::Color(1, 0, 0, 1.0));
            }
        }

        void GetPointsCallback(const geometry_msgs::Point &msg)
        {
            // Store the received point
            if (points.size() < 3)
            {
                points.push_back(msg);
            }
            // If we have 3 points, we can reset for the next triangle
            if (points.size() == 3)
            {
                points.clear(); // Clear points for the next triangle
            }
        }

    private:
        ros::NodeHandle *rosnode;
        std::string topic_name;
        rendering::VisualPtr visual;
        rendering::DynamicLines *line;
        std::string visual_namespace;
        ros::Subscriber point_sub;
        std::vector<geometry_msgs::Point> points; // Store points for the triangle
        event::ConnectionPtr update_connection;
    };
    GZ_REGISTER_VISUAL_PLUGIN(TriangleDrawPlugin)
}
