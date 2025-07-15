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
			//printf("true 3508\n");
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
	updateParams();

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
		motor_index = msg.id -0x201;

		if (motor_index != -1) {

		//int16_t mechanical_angle = static_cast<int16_t>(msg.data[0] | (msg.data[1] << 8));
		float speed_rpm_raw        = static_cast<int16_t>(msg.data[2] | (msg.data[3] << 8));
		float actual_current_raw   = static_cast<int16_t>(msg.data[4] | (msg.data[5] << 8));
		//int16_t temperature_c    = static_cast<int16_t>(msg.data[6] | (msg.data[7] << 8));

		//pthread_mutex_lock(&_node_mutex);
		_current_speed_rpm[motor_index] = (float)speed_rpm_raw;
		_actual_current_ma[motor_index] = (float)actual_current_raw;
		//pthread_mutex_unlock(&_node_mutex);
		// printf(" Angle: %d \n", mechanical_angle);
		printf(" Speed: %f RPM\n", (double)speed_rpm_raw);
		printf(" Current: %f mA\n", (double)actual_current_raw);
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




bool vulcan_interface_gim6010_catchfn(const uavcan::CanRxFrame& msg){
	//捕获can帧，准备进入绑定的回调函数
	if(msg.isExtended()==false){
		uint16_t node_id = (msg.id>>5)& 0x3F;
		uint8_t cmd_id = msg.id & 0x1F;
		if(node_id<=3)
			return (cmd_id ==0x009 || cmd_id==0x01C);
	}
	return false;
}
VulcanMixingInterfaceGIM6010::VulcanMixingInterfaceGIM6010(VulcanNode* node,pthread_mutex_t &node_mutex)
	:px4::ScheduledWorkItem(MODULE_NAME "-actuators-gim6010",px4::wq_configurations::uavcan),
	ModuleParams(node),
	_node(node),
      _node_mutex(node_mutex),
      _suber(vulcan_interface_gim6010_catchfn,CbBinder(this,&VulcanMixingInterfaceGIM6010::data_sub_cb))
{
	this->_node->add_subscriber(&_suber);
	for (unsigned i=0;i<NUM_GIM6010_MOTORS_PER_FRAME;++i) {
		_node_ids[i] = i;
		//_current_input_mode[i] = 0;
    	}
	memset(&_feedback,0,sizeof(_feedback));
	for(unsigned i=0;i<NUM_GIM6010_MOTORS_PER_FRAME;++i){
		send_axis_state(i, 8);
		send_control_mode(i, 3);//位置控制，位置滤波模式
	}
}
void VulcanMixingInterfaceGIM6010::Run()
{
	pthread_mutex_lock(&_node_mutex);
	//订阅和发布消息
	gim6010_command_s cmd;
	if(_command_sub.update(&cmd)){
		for(unsigned i=0;i<NUM_GIM6010_MOTORS_PER_FRAME;++i){
			float position_setpoint = cmd.position[i];
			int16_t velocity_setpoint = cmd.velocity[i];
			int16_t torque_setpoint = cmd.torque[i];
			send_setpoint(i,position_setpoint,velocity_setpoint,torque_setpoint);
		}
	}
	pthread_mutex_unlock(&_node_mutex);
}
void VulcanMixingInterfaceGIM6010::data_sub_cb(const uavcan::CanRxFrame& msg)
{
	//将反馈信息发布到uorb话题中，以便其他模块使用
	uint16_t node_id = (msg.id>>5)& 0x3F;
	uint8_t cmd_id = msg.id & 0x1F;
	if(msg.dlc==8){
		if(cmd_id==0x009){
			float position;
			float velocity;
			memcpy(&position, &msg.data[0], sizeof(float));
			memcpy(&velocity, &msg.data[4], sizeof(float));
			_feedback.position[node_id] = position;
			_feedback.velocity[node_id] = velocity;

		}else if(cmd_id==0x01C){
			float torque;
			memcpy(&torque, &msg.data[4], sizeof(float));
			_feedback.torque[node_id] = torque;
		}
	}

	_feedback.timestamp = hrt_absolute_time();
	_feedback_pub.publish(_feedback);

}
void VulcanMixingInterfaceGIM6010::send_axis_state(uint8_t motor_idx, uint8_t state)
{
	uavcan::CanFrame frame;
	frame.id = (_node_ids[motor_idx]<<5) | 0x007;
	frame.dlc = 4;
	uint32_t state_val = state;
    	memcpy(&frame.data[0], &state_val, sizeof(uint32_t));
	_node->send(frame,
	    uavcan::MonotonicTime::fromMSec(1),
	    uavcan::MonotonicTime::fromMSec(1),
 	    uavcan::CanTxQueue::Qos::Persistent,
	    0,
	    1);
}
void VulcanMixingInterfaceGIM6010::send_control_mode(uint8_t motor_idx, uint8_t input_mode)
{
    uavcan::CanFrame frame;
    frame.id = (_node_ids[motor_idx] << 5) | 0x00B;
    frame.dlc = 8;
    uint32_t control_mode_val = 3; // 固定为位置控制模式
    uint32_t input_mode_val = input_mode;
    memcpy(&frame.data[0], &control_mode_val, sizeof(uint32_t));
    memcpy(&frame.data[4], &input_mode_val, sizeof(uint32_t));
    _node->send(frame,
	uavcan::MonotonicTime::fromMSec(1),
	uavcan::MonotonicTime::fromMSec(1),
	uavcan::CanTxQueue::Qos::Persistent,
	0,
	1);
}
void VulcanMixingInterfaceGIM6010::send_setpoint(uint8_t motor_idx, float position,int16_t velocity,int16_t torque)
{
    uavcan::CanFrame frame;
    frame.id = (_node_ids[motor_idx] << 5) | 0x00C;
    frame.dlc = 8;
    memcpy(&frame.data[0], &position, sizeof(float));
    memcpy(&frame.data[4], &velocity, sizeof(int16_t));
    memcpy(&frame.data[6], &torque, sizeof(int16_t));
    _node->send(frame,
	uavcan::MonotonicTime::fromMSec(1),
	uavcan::MonotonicTime::fromMSec(1),
	uavcan::CanTxQueue::Qos::Persistent,
	0,
	1);
}
