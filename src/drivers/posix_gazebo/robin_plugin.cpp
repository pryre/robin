#include <vector>
#include "drivers/posix_gazebo/robin_plugin.hpp"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "run.h"
#include "drivers/posix_common/runtime.h"
#include "drivers/drv_system.h"
#include "drivers/drv_pwm.h"

static bool _soft_reset;
static uint32_t _sim_time_sec;
static uint32_t _sim_time_nsec;

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

namespace Robin
{
	void RobinPlugin::Setup() {		
		setup();
		system_debug_print("Finished loading Robin!");
	}

	void RobinPlugin::Run(uint32_t secs, uint32_t nsecs) {
		_sim_time_sec = secs;
		_sim_time_nsec = nsecs;
	
		if(_soft_reset) {
			setup();
			_soft_reset = false;
		}
	
		loop();
	}

	void RobinPlugin::Reset() {
		posix_soft_reset();
	}

	std::vector<double> RobinPlugin::GetNormalizedMotorCommands() {
		std::vector<double> cmds;
		cmds.reserve(DRV_PWM_MAX_OUTPUTS);
	
		for(int i=0; i<DRV_PWM_MAX_OUTPUTS; i++) {
			double pwm = drv_pwm_get_current(i);
			double vnrom = 0.0;
		
			if( (pwm >= DRV_PWM_MIN) && (pwm <= DRV_PWM_MAX) )
				vnrom = (pwm - DRV_PWM_MIN) / (DRV_PWM_MAX - DRV_PWM_MIN);

			cmds[i] = vnrom;
		}
		
		return cmds;
	}
}