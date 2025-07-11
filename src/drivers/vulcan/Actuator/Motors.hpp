#ifndef __VULCAN_MOTORS_
#define __VULCAN_MOTORS_
#include <lib/mixer_module/mixer_module.hpp>

#include "../Dispatcher.hpp"
#include "uavcan/driver/can.hpp"
#include "uavcan/transport/can_io.hpp"
#include "uavcan/util/method_binder.hpp"
// #include "../vulcan_main.hpp"

class VulcanNode;


bool vulcan_interface_test_catchfn(const uavcan::CanRxFrame& msg);

class VulcanMixingInterfaceTest : public OutputModuleInterface
{

private:
	friend class VulcanNode;
	VulcanNode* _node;
	pthread_mutex_t &_node_mutex;
	MixingOutput _mixing_output{"VULCAN_SV", 8, *this, MixingOutput::SchedulingPolicy::Auto, false, false};
public:
	void data_sub_cb(const uavcan::CanRxFrame& msg);


public:
	VulcanMixingInterfaceTest(VulcanNode* node,pthread_mutex_t &node_mutex);

	bool updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated) override;

	// void mixerChanged() override;

	MixingOutput &mixingOutput() { return _mixing_output; }


	typedef uavcan::MethodBinder < VulcanMixingInterfaceTest *,
		void (VulcanMixingInterfaceTest::*)
		(const  uavcan::CanRxFrame&) >
		CbBinder;




protected:
	uavcan::Subscriber<CbBinder> _suber;

	void Run() override;


};
#endif
