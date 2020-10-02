#include <vector>
#include <stdint.h>

namespace Robin
{
	class RobinPlugin {
	public:
		void Setup();

		void Run(uint32_t secs, uint32_t nsecs);

		void Reset();

		std::vector<double> GetNormalizedMotorCommands();
	};
}