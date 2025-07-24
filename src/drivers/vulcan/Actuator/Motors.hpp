#ifndef __VULCAN_MOTORS_
#define __VULCAN_MOTORS_
#include <lib/mixer_module/mixer_module.hpp>

#include "../Dispatcher.hpp"
#include "uavcan/driver/can.hpp"
#include "uavcan/transport/can_io.hpp"
#include "uavcan/util/method_binder.hpp"
// #include "../vulcan_main.hpp"
#include "lib/pid/pid.h"
#include <uORB/topics/parameter_update.h>
// #include <px4_platform_common/module_params.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/gim6010_feed_back.h>
#include <uORB/topics/gim6010_command.h>
//#include <uORB/SubscriptionInterval.hpp>
const int16_t M3508_MAX_CURRENT = 16384;
const int16_t M3508_MIN_CURRENT = -16384;
const uint16_t PX4_OUTPUT_MAX_VAL = 8191;
const unsigned NUM_M3508_MOTORS_PER_FRAME = 4;
const unsigned NUM_GIM6010_MOTORS_PER_FRAME = 4;
const float POSITION_TOLERANCE = 0.01f;
const uint32_t GIM6010_SEND_INTERVAL_US = 2000;
class VulcanNode;


bool vulcan_interface_test_catchfn(const uavcan::CanRxFrame& msg);
bool vulcan_interface_m3508_catchfn(const uavcan::CanRxFrame& msg);
bool vulcan_interface_gim6010_catchfn(const uavcan::CanRxFrame& msg);

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

class VulcanMixingInterfaceM3508 : public OutputModuleInterface
{

private:
	friend class VulcanNode;
	VulcanNode* _node;
	pthread_mutex_t &_node_mutex;
	MixingOutput _mixing_output{"M3508", 8, *this, MixingOutput::SchedulingPolicy::Auto, false, false};

	PID_t _speed_pid[NUM_M3508_MOTORS_PER_FRAME];
	PID_t _current_pid[NUM_M3508_MOTORS_PER_FRAME];

	float _current_speed_rpm[NUM_M3508_MOTORS_PER_FRAME]={0.0f};
	float _actual_current_ma[NUM_M3508_MOTORS_PER_FRAME]={0.0f};

	hrt_abstime _last_update_outputs_time_us= 0;


	float _speed_kp[NUM_M3508_MOTORS_PER_FRAME];
	float _speed_ki[NUM_M3508_MOTORS_PER_FRAME];
	float _speed_kd[NUM_M3508_MOTORS_PER_FRAME];
	float _current_kp[NUM_M3508_MOTORS_PER_FRAME];
	float _current_ki[NUM_M3508_MOTORS_PER_FRAME];
	float _current_kd[NUM_M3508_MOTORS_PER_FRAME];
	int32_t _m3508_enable = 1; // 默认启用

public:
	void data_sub_cb(const uavcan::CanRxFrame& msg);
	void init_pid_controllers();

	VulcanMixingInterfaceM3508(VulcanNode* node,pthread_mutex_t &node_mutex);

	bool updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated) override;

	// void mixerChanged() override;

	MixingOutput &mixingOutput() { return _mixing_output; }
        void print_info();

	typedef uavcan::MethodBinder < VulcanMixingInterfaceM3508 *,
		void (VulcanMixingInterfaceM3508::*)
		(const  uavcan::CanRxFrame&) >
		CbBinder;
	 void updateParams() override;



protected:
	uavcan::Subscriber<CbBinder> _suber;

	void Run() override;


};

class VulcanMixingInterfaceGIM6010 : public px4::ScheduledWorkItem, public ModuleParams
{

private:
	bool init_commands_sent = false;
	friend class VulcanNode;
	VulcanNode* _node;
	pthread_mutex_t &_node_mutex;

	uORB::Subscription _command_sub{ORB_ID(gim6010_command)};
    	uORB::Publication<gim6010_feed_back_s> _feedback_pub{ORB_ID(gim6010_feed_back)};
    	gim6010_feed_back_s _feedback;
	void send_axis_state(uint8_t motor_idx, uint8_t state);
	void send_control_mode(uint8_t motor_idx, uint8_t input_mode);
   	void send_setpoint(uint8_t motor_idx, float position,int16_t velocity,int16_t torque);
	void send_linear_count(uint8_t motor_idx,int32_t count);
	hrt_abstime _last_gim6010_send_time_us[NUM_GIM6010_MOTORS_PER_FRAME];
	uint8_t _current_motor_idx_to_send{0};

public:
	void data_sub_cb(const uavcan::CanRxFrame& msg);

	VulcanMixingInterfaceGIM6010(VulcanNode* node,pthread_mutex_t &node_mutex);


	typedef uavcan::MethodBinder < VulcanMixingInterfaceGIM6010 *,
		void (VulcanMixingInterfaceGIM6010::*)
		(const  uavcan::CanRxFrame&) >
		CbBinder;

	//  void updateParams() override;

protected:
	uavcan::Subscriber<CbBinder> _suber;

	void Run() override;


};
#endif
