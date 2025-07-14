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
//              flags, iface_mask);
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


bool vulcan_interface_m3508_catchfn(const uavcan::CanRxFrame& msg){
	if(msg.isExtended()==false){
		if(msg.id >= 0x201 && msg.id <= 0x204){
			printf("true 3508\n");
			return true;
		}
	}
	return false;
}


VulcanMixingInterfaceM3508::VulcanMixingInterfaceM3508(VulcanNode* node,pthread_mutex_t &node_mutex)
    : OutputModuleInterface(MODULE_NAME "-actuators-m3508", px4::wq_configurations::uavcan),
_node(node),
      _node_mutex(node_mutex),
      _suber(vulcan_interface_m3508_catchfn,CbBinder(this,&VulcanMixingInterfaceM3508::data_sub_cb))
{
	this->_node->add_subscriber(&_suber);
	(void)param_get(param_find("M3508_ENABLE"), &_m3508_enable);
	if(_m3508_enable){
		updateParams();
	}

}
void VulcanMixingInterfaceM3508::Run()
{
	if(_m3508_enable == 0) {
        	return;
   	}
	pthread_mutex_lock(&_node_mutex);
	_mixing_output.update();
	_mixing_output.updateSubscriptions(false);
	pthread_mutex_unlock(&_node_mutex);
}
void VulcanMixingInterfaceM3508::data_sub_cb(const uavcan::CanRxFrame& msg)
{
	if(_m3508_enable == 0) {
        	return;
   	}
	if (msg.dlc == 8) {
        int motor_index = -1;
        if (msg.id == 0x201) { motor_index = 0; }
        else if (msg.id == 0x202) { motor_index = 1; }
        else if (msg.id == 0x203) { motor_index = 2; }
        else if (msg.id == 0x204) { motor_index = 3; }

		if (motor_index != -1) {

		//int16_t mechanical_angle = static_cast<int16_t>(msg.data[0] | (msg.data[1] << 8));
		int16_t speed_rpm_raw        = static_cast<int16_t>(msg.data[2] | (msg.data[3] << 8));
		int16_t actual_current_raw   = static_cast<int16_t>(msg.data[4] | (msg.data[5] << 8));
		//int16_t temperature_c    = static_cast<int16_t>(msg.data[6] | (msg.data[7] << 8));

		//pthread_mutex_lock(&_node_mutex);
		_current_speed_rpm[motor_index] = (float)speed_rpm_raw;
		_actual_current_ma[motor_index] = (float)actual_current_raw;
		//pthread_mutex_unlock(&_node_mutex);
		// printf(" Angle: %d \n", mechanical_angle);
		printf(" Speed: %d RPM\n", speed_rpm_raw);
		printf(" Current: %d mA\n", actual_current_raw);
		// printf(" Temp: %d C\n", temperature_c);
		}
	}
}
bool VulcanMixingInterfaceM3508::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated)
{
	if(_m3508_enable == 0) {
        	return true;
   	}
	hrt_abstime current_time_us = hrt_absolute_time();
	float dt_s =(_last_update_outputs_time_us==0)?0.0f:(float)(current_time_us-_last_update_outputs_time_us)/1000000.0f;
	_last_update_outputs_time_us = current_time_us;
	if(stop_motors){
		uavcan::CanFrame stop_frame;
		stop_frame.id = 0x200;
		stop_frame.dlc = 8;
		for(int i =0;i<8;++i){
			stop_frame.data[i]=0;
		}
		_node->send(stop_frame,uavcan::MonotonicTime::fromMSec(1),uavcan::MonotonicTime::fromMSec(1),uavcan::CanTxQueue::Persistent,0,1);
		return true;
	}

	uavcan::CanFrame send_frame;
	send_frame.id = 0x200;
	send_frame.dlc = 8;
	//pthread_mutex_lock(&_node_mutex);
	//具体最大转速需要调整
	const float M3508_MAX_TARGET_RPM = 469.0f;

	for (unsigned i = 0; i < NUM_M3508_MOTORS_PER_FRAME; ++i) {
		float target_speed_rpm = 0.0f;
		if (i < num_outputs) {
			float scaled_px4_output = static_cast<float>(outputs[i]) / PX4_OUTPUT_MAX_VAL;
            		target_speed_rpm = (scaled_px4_output * 2.0f - 1.0f) * M3508_MAX_TARGET_RPM;
		}
		float acutual_speed_rpm = _current_speed_rpm[i];
		float acutual_current_ma = _actual_current_ma[i];
		int16_t final_current_command =0;
		if(dt_s <= 0.0f){
			final_current_command = static_cast<int16_t>(_current_pid[i].last_output);
		}else{
			//速度 PID (外环)
			float target_current_from_speed_pid = pid_calculate(
				&_speed_pid[i],
				target_speed_rpm,
				acutual_speed_rpm,
				0.0f,
				dt_s
			);
			//电流 PID (内环)
			float final_current_command_float = pid_calculate(
				&_current_pid[i],
				target_current_from_speed_pid,
				acutual_current_ma,
				0.0f,
				dt_s
			);
			final_current_command = static_cast<int16_t>(final_current_command_float);
		}
		if(final_current_command >M3508_MAX_CURRENT){
			final_current_command = M3508_MAX_CURRENT;
		}else if(final_current_command <M3508_MIN_CURRENT){
			final_current_command = M3508_MIN_CURRENT;
		}
		 send_frame.data[i * 2]     = static_cast<uint8_t>(final_current_command & 0xFF);
		 send_frame.data[i * 2 + 1] = static_cast<uint8_t>((final_current_command >> 8) & 0xFF);
	}
	//pthread_mutex_unlock(&_node_mutex);

    _node->send(send_frame,
                uavcan::MonotonicTime::fromMSec(1),
                uavcan::MonotonicTime::fromMSec(1),
                uavcan::CanTxQueue::Qos::Persistent,
                0,
                1);

    return true;

}
void VulcanMixingInterfaceM3508::init_pid_controllers() {
    for (unsigned i = 0; i < NUM_M3508_MOTORS_PER_FRAME; ++i) {
        pid_init(&_speed_pid[i], PID_MODE_DERIVATIV_CALC, 0.000001f);
        float speed_integral_limit = 1000.0f;
        float speed_output_limit = (float)M3508_MAX_CURRENT;
        pid_set_parameters(&_speed_pid[i], _speed_kp, _speed_ki, _speed_kd, speed_integral_limit, speed_output_limit);

        pid_init(&_current_pid[i], PID_MODE_DERIVATIV_CALC, 0.000001f);
        float current_integral_limit = (float)M3508_MAX_CURRENT;
        float current_output_limit = (float)M3508_MAX_CURRENT;
        pid_set_parameters(&_current_pid[i], _current_kp, _current_ki, _current_kd, current_integral_limit, current_output_limit);
    }
}


void VulcanMixingInterfaceM3508::updateParams() {

    (void)param_get(param_find("M3508_ENABLE"), &_m3508_enable);
    (void)param_get(param_find("M3508_SPEED_KP"), &_speed_kp);
    (void)param_get(param_find("M3508_SPEED_KI"), &_speed_ki);
    (void)param_get(param_find("M3508_SPEED_KD"), &_speed_kd);
    (void)param_get(param_find("M3508_CURRENT_KP"), &_current_kp);
    (void)param_get(param_find("M3508_CURRENT_KI"), &_current_ki);
    (void)param_get(param_find("M3508_CURRENT_KD"), &_current_kd);


    if (_m3508_enable) {
        init_pid_controllers();
    }


}
