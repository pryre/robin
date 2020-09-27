#include <functional>
#include <string>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

#ifdef __cplusplus
extern "C" {
#endif

#include "run.h"
#include "drivers/posix_common/runtime.h"
#include "drivers/drv_pwm.h"

static bool _soft_reset;
static bool _sim_time_sec;
static bool _sim_time_nsec;

void posix_soft_reset( void ) {
	_soft_reset = true;
}

void posix_get_sim_time( uint32_t *secs, uint32_t *nsecs ) {
	*secs = _sim_time_sec;
	*nsecs = _sim_time_nsec;
}

#ifdef __cplusplus
}
#endif

namespace gazebo
{
	class RobinPlugin : public ModelPlugin {
	private:
		physics::ModelPtr model;
		event::ConnectionPtr updateConnection;
		event::ConnectionPtr timeResetConnection;
		event::ConnectionPtr worldResetConnection;
		
		std::string jointPrefix;
		unsigned int numMotors;
		
		std::vector<physics::JointPtr> motorJoints;

	public:
		void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
			// Store the pointer to the model
			this->model = _parent;

			// Listen to the update event. This event is broadcast every
			// simulation iteration.
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&RobinPlugin::OnUpdate, this));
			this->timeResetConnection = event::Events::ConnectTimeReset(std::bind(&RobinPlugin::OnReset, this));
			this->worldResetConnection = event::Events::ConnectWorldReset(std::bind(&RobinPlugin::OnReset, this));
			
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
			
			// Get the pointers to the motor joints
			for(int i=0; i < numMotors; i++) {
				std::string joint_name = jointPrefix + std::to_string(i);
				physics::JointPtr joint = model->GetJoint( joint_name );
				if(joint != NULL) {
					motorJoints.push_back( joint );
				} else {
					gzthrow( "[RobinPlugin] Couldn't find specified joint \"" << joint_name );
				}
			}
		}

		// Called by the world update start event
		void OnUpdate() {
			common::Time tick = model->GetWorld()->SimTime();
			_sim_time_sec = tick.sec;
			_sim_time_nsec = tick.nsec;
		
			if(_soft_reset) {
				setup();
				_soft_reset = false;
			}
		
			loop();
			
			for(int i=0; i<motorJoints.size(); i++) {
				uint32_t pwm = drv_pwm_get_current(i);
				double vnrom = 0.0;
				
				if( (pwm >= DRV_PWM_MIN) && (pwm <= DRV_PWM_MAX) )
					vnrom = (pwm - DRV_PWM_MIN) / (DRV_PWM_MAX - DRV_PWM_MIN);

				motorJoints[i]->SetVelocity(0, vnrom);
			}
		}

		void OnReset() {
			posix_soft_reset();
		}
	};

	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(RobinPlugin)
}