#include <functional>
#include <string>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include "drivers/posix_gazebo/robin_plugin.hpp"


namespace gazebo
{
	class GazeboRobinPlugin : public ModelPlugin {
	private:
		physics::ModelPtr model;
		event::ConnectionPtr updateConnection;
		event::ConnectionPtr timeResetConnection;
		event::ConnectionPtr worldResetConnection;
		
		std::string jointPrefix;
		unsigned int numMotors;
		
		std::vector<physics::JointPtr> motorJoints;
		
		Robin::RobinPlugin robin;

	public:
		void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
			// Store the pointer to the model
			this->model = _parent;

			// Listen to the update event. This event is broadcast every
			// simulation iteration.
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&GazeboRobinPlugin::OnUpdate, this));
			this->timeResetConnection = event::Events::ConnectTimeReset(std::bind(&GazeboRobinPlugin::OnReset, this));
			this->worldResetConnection = event::Events::ConnectWorldReset(std::bind(&GazeboRobinPlugin::OnReset, this));
			
			if ( _sdf->HasElement( "motorPrefix" ) ) {
				jointPrefix = _sdf->GetElement( "motorPrefix" )->Get<std::string>();
			} else {
				gzerr << "[RobinPlugin] Please specify a motorPrefix\n";
			}
			
			if ( _sdf->HasElement( "numMotors" ) ) {
				numMotors = _sdf->GetElement( "numMotors" )->Get<unsigned int>();
			} else {
				gzerr << "[RobinPlugin] Please specify numMotors\n";
			}
			
			robin.Setup();
			
			// Get the pointers to the motor joints
			/*
			for(int i=0; i < numMotors; i++) {
				std::string joint_name = jointPrefix + std::to_string(i);
				physics::JointPtr joint = model->GetJoint( joint_name );
				if(joint != NULL) {
					motorJoints.push_back( joint );
				} else {
					gzthrow( "[RobinPlugin] Couldn't find specified joint \"" << joint_name <<  "\"\n");
				}
			}
			*/
		}

		// Called by the world update start event
		void OnUpdate() {
			common::Time tick = this->model->GetWorld()->SimTime();
			robin.Run(tick.sec, tick.nsec);
			
			std::vector<double> cmds = robin.GetNormalizedMotorCommands();
			
			for(int i=0; (i<motorJoints.size() && i<cmds.size()); i++)
				motorJoints[i]->SetVelocity(0, cmds[i]);
		}

		void OnReset() {
			robin.Reset();
		}
	};

	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(GazeboRobinPlugin)
}