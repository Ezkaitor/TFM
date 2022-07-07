#ifndef _DRONE_MOVE_HH_
#define _DRONE_MOVE_HH_

#include <gazebo-11/gazebo.hh>
#include <gazebo-11/physics/physics.hh>

namespace gazebo
{
    class DronePlugin : public ModelPlugin
    {
        // Constructor
        public: DronePlugin() {}
        public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            std__cerr << "\nThe plugin just started[" << _model->GetName() << "]\n";
        }

        // Register the plugin son Gazebo can call and load it
        GZ_REGISTER_MODEL_PLUGIN(DronePlugin)


    }
}
#endif