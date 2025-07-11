#include "Motors.hpp"
#include "../vulcan_main.hpp" // <--- ADD THIS LINE TO INCLUDE THE FULL DEFINITION OF VULCANNNODE



bool vulcan_interface_test_catchfn(const uavcan::CanRxFrame& msg)
{
	if(msg.isExtended()==true){
		if(msg.id==0x111){
			return true;
		}
	}
	return false;
}




VulcanMixingInterfaceTest::VulcanMixingInterfaceTest(VulcanNode* node,pthread_mutex_t &node_mutex)
    : OutputModuleInterface(MODULE_NAME "-actuators-test", px4::wq_configurations::uavcan),
_node(node),
      _node_mutex(node_mutex),
      _suber(vulcan_interface_test_catchfn,CbBinder(this,&VulcanMixingInterfaceTest::data_sub_cb))
{

}

void
VulcanMixingInterfaceTest::Run()
{
pthread_mutex_lock(&_node_mutex);
_mixing_output.update();
_mixing_output.updateSubscriptions(false);
pthread_mutex_unlock(&_node_mutex);
}

void
VulcanMixingInterfaceTest::data_sub_cb(const uavcan::CanRxFrame& msg)
{

}



bool
VulcanMixingInterfaceTest::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated)
{


//    return _node.send(frame, tx_deadline, blocking_deadline, qos,
        //      flags, iface_mask);
	uavcan::CanFrame frame;
	frame.id = 0x111;
	for(int i = 0 ;  i < 8 ;++i)
	{
		frame.data[i] = outputs[i];
		// printf("Motor%d:%d ",i,outputs[i]);
	}
	// printf("\n");
	frame.dlc = 8;



	_node->send(frame,uavcan::MonotonicTime::fromMSec(1),uavcan::MonotonicTime::fromMSec(1),uavcan::CanTxQueue::Qos::Persistent,0,1);

	return true;
}
