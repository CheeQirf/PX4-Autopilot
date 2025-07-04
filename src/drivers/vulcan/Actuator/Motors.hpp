#include <lib/mixer_module/mixer_module.hpp>

#include "../Dispatcher.hpp"

#include "uavcan/driver/can.hpp"
#include "uavcan/transport/can_io.hpp"
#include "uavcan/util/method_binder.hpp"

bool vulcan_interface_test_catchfn(const uavcan::CanRxFrame& msg)
{
	if(msg.isExtended()==true){
		if(msg.id==0x111){
			return true;
		}
	}
	return false;
}


class VulcanMixingInterfaceTest : public OutputModuleInterface
{
public:
	void data_sub_cb(const uavcan::CanRxFrame& msg);


public:
	VulcanMixingInterfaceTest(pthread_mutex_t &node_mutex)
		: OutputModuleInterface(MODULE_NAME "-actuators-test", px4::wq_configurations::uavcan),
		  _node_mutex(node_mutex),
		  _suber(vulcan_interface_test_catchfn,CbBinder(this,&VulcanMixingInterfaceTest::data_sub_cb))
		  {};

	bool updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated) override;

	void mixerChanged() override;

	MixingOutput &mixingOutput() { return _mixing_output; }


	typedef uavcan::MethodBinder < VulcanMixingInterfaceTest *,
		void (VulcanMixingInterfaceTest::*)
		(const  uavcan::CanRxFrame&) >
		CbBinder;




protected:
	void Run() override;


private:
	friend class VulcanNode;
	pthread_mutex_t &_node_mutex;
	MixingOutput _mixing_output{"VULCAN_TEST", 8, *this, MixingOutput::SchedulingPolicy::Auto, false, false};
	uavcan::Subscriber<CbBinder> _suber;
};
