#include "Motors.hpp"

void
VulcanMixingInterfaceTest::Run()
{
pthread_mutex_lock(&_node_mutex);
_mixing_output.update();
_mixing_output.updateSubscriptions(false);
pthread_mutex_unlock(&_node_mutex);
}


bool
VulcanMixingInterfaceTest::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated)
{



	return false;
}
