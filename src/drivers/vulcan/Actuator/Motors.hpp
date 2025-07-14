#ifndef __VULCAN_MOTORS_
#define __VULCAN_MOTORS_
#include <lib/mixer_module/mixer_module.hpp>

#include "../Dispatcher.hpp"
#include "uavcan/driver/can.hpp"
#include "uavcan/transport/can_io.hpp"
#include "uavcan/util/method_binder.hpp"
// #include "../vulcan_main.hpp"
//#include "../../../lib/pid/pid.h"
#include "lib/pid/pid.h"
#include "../../drv_hrt.h"
#include <uORB/topics/parameter_update.h>
#include <px4_platform_common/module_params.h>


const int16_t M3508_MAX_CURRENT = 10000;
const int16_t M3508_MIN_CURRENT = -10000;
const uint16_t PX4_OUTPUT_MAX_VAL = 10000;
const unsigned NUM_M3508_MOTORS_PER_FRAME = 4;

class VulcanNode;


bool vulcan_interface_test_catchfn(const uavcan::CanRxFrame& msg);
bool vulcan_interface_m3508_catchfn(const uavcan::CanRxFrame& msg);

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


	float _speed_kp = 0.05f;
	float _speed_ki = 0.01f;
	float _speed_kd = 0.001f;
	float _current_kp = 1.0f;
	float _current_ki = 0.1f;
	float _current_kd = 0.005f;
	int32_t _m3508_enable = 1; // 默认启用

public:
	void data_sub_cb(const uavcan::CanRxFrame& msg);
	void init_pid_controllers();

	VulcanMixingInterfaceM3508(VulcanNode* node,pthread_mutex_t &node_mutex);

	bool updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated) override;

	// void mixerChanged() override;

	MixingOutput &mixingOutput() { return _mixing_output; }


	typedef uavcan::MethodBinder < VulcanMixingInterfaceM3508 *,
		void (VulcanMixingInterfaceM3508::*)
		(const  uavcan::CanRxFrame&) >
		CbBinder;
	 void updateParams() override;



protected:
	uavcan::Subscriber<CbBinder> _suber;

	void Run() override;


};
#endif
