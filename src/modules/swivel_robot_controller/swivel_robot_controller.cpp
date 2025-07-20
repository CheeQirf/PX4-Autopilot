#include "swivel_robot_controller.hpp"
#include <px4_platform_common/log.h>

#include "swivel_robot_pids/swivel_robot_pids.hpp"

#include <matrix/math.hpp> // 用于四元数转欧拉角、cosf/sinf
#include <drivers/drv_hrt.h> // 用于 hrt_absolute_time()
using namespace time_literals;
// 构造函数：初始化参数和订阅
swivelDrive::swivelDrive() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
	// 初始化默认参数
	updateParams();
}

// 初始化模块：启动定时任务调度，指定任务周期为 5 毫秒（100Hz）
bool swivelDrive::init()
{
	ScheduleOnInterval(1_ms);  // 调度周期：500Hz
	return true;
}

// 更新参数：从参数存储中获取并更新模块的配置参数
void swivelDrive::updateParams()
{
	// 更新所有模块参数
	ModuleParams::updateParams();

	// 设置差动驱动相关的最大速度、角速度等参数
	// _max_speed = _param_rdd_wheel_speed.get() * _param_rdd_wheel_radius.get();
	// _max_angular_velocity = _max_speed / (_param_rdd_wheel_base.get() / 2.f);
}

// 主要任务：获取遥控器数据并打印
void swivelDrive::Run()
{

	// 检查是否需要退出
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
	}



	float _current_global_vx = 0.0f;
    float _current_global_vy = 0.0f;
    float _current_global_yaw = 0.0f;


	float degrees = 0.f;
	bool mode = 0;//mode = 0:前进模式  mode = 1:旋转模式

//--------------------------------------------------参数部分-------------------------------------------------------------

	//获取参数更新，并且调用updateParams函数进行参数更新
	if (_parameter_update_sub.updated()) {
		parameter_update_s parameter_update;
		_parameter_update_sub.copy(&parameter_update);
		updateParams();
	}

	// 处理遥控器输入数据
	if (_manual_control_setpoint_sub.updated()) {
		manual_control_setpoint_s manual_control_setpoint{};

		if (_manual_control_setpoint_sub.copy(&manual_control_setpoint)) {

            if(static_cast<float>(manual_control_setpoint.roll) < 0.3f && static_cast<float>(manual_control_setpoint.pitch) <0.3f &&
            static_cast<float>(manual_control_setpoint.roll) > -0.3f && static_cast<float>(manual_control_setpoint.pitch) > -0.3f){

                if(static_cast<float>(manual_control_setpoint.yaw) > 0.75f){
                    lunzi_jiaodu[1] = math::radians(135.f);
					lunzi_jiaodu[2] = math::radians(45.f);
					lunzi_jiaodu[3] = math::radians(-45.f);
					lunzi_jiaodu[4] = math::radians(-135.f);
					mode = 1;
                }
                else if(static_cast<float>(manual_control_setpoint.yaw) < -0.75f){
                    lunzi_jiaodu[1] = math::radians(-45.f);
					lunzi_jiaodu[2] = math::radians(-135.f);
					lunzi_jiaodu[3] = math::radians(135.f);
					lunzi_jiaodu[4] = math::radians(45.f);
					mode = 1;
                }
                else{
                    lunzi_jiaodu[1] = math::radians(0.f);
					lunzi_jiaodu[2] = math::radians(0.f);
					lunzi_jiaodu[3] = math::radians(0.f);
					lunzi_jiaodu[4] = math::radians(0.f);
					mode = 0;
                }
            }
            else {

                float roll = static_cast<float>(manual_control_setpoint.roll);
                float pitch = static_cast<float>(manual_control_setpoint.pitch);

				mode = 0;

                if (std::fabs(roll) < 0.01f && std::fabs(pitch) < 0.01f) {

                    for(int i = 1; i < 5; i++) {
                        lunzi_jiaodu[i] = math::radians(0.f);
                    }
                } else {

                    float radians = atan2f(roll, pitch);

                    degrees = radians;

                    for(int i = 1; i < 5; i++) {
                        lunzi_jiaodu[i] = degrees;
                    }
                }
            }
            youmen = (static_cast<float>(manual_control_setpoint.throttle) + 1)/2;
			(void)youmen;
			(void)lunzi_jiaodu;
			//printf("[time:%lld]youmen = %.2f,1:%3.1f|2:%3.1f|3:%3.1f|4:%3.1f\n",now,
			//		static_cast<double>(youmen),static_cast<double>(lunzi_jiaodu[1]),static_cast<double>(lunzi_jiaodu[2]),static_cast<double>(lunzi_jiaodu[3]),static_cast<double>(lunzi_jiaodu[4]));
		}
	}

	//获取机器人速度
	if (_local_position_sub.updated()) {
        vehicle_local_position_s local_pos;
        if (_local_position_sub.copy(&local_pos)) {
            _current_global_vx = local_pos.vx; // X轴速度
            _current_global_vy = local_pos.vy; // Y轴速度
        }
    }

	//获取yaw角度
	if (_attitude_sub.updated()) {
        vehicle_attitude_s att;
        if (_attitude_sub.copy(&att)) {
            matrix::Quatf q(att.q);
            matrix::Eulerf euler = q;
            _current_global_yaw = euler(2); // Yaw角度（弧度）
        }
    }

	//获取yaw角速度
	if (_angular_velocity_sub.updated()) {
        vehicle_angular_velocity_s angular_vel;
        if (_angular_velocity_sub.copy(&angular_vel)) {
            float yaw_rate = angular_vel.xyz[2]; // Yaw轴角速度
            // 使用 yaw_rate ...
			(void)yaw_rate;
        }
    }

	// 如果当前是前进模式，并且上一帧是旋转模式，说明刚从旋转切换过来
    if (mode == 0 && _previous_mode == 1) {
        // 捕获当前的偏航角，作为新的目标偏航角
        _desired_global_yaw = _current_global_yaw;
        //PX4_INFO("New Yaw Target Locked: %.2f degrees", static_cast<double>(math::degrees(_desired_global_yaw)));
    }
    _previous_mode = mode; // 更新上一帧模式记录

	// 1. 期望的前进速度 (全局X轴方向，单位：m/s)
	const float SPEED_MAX = 2.f;
    const float DESIRED_GLOBAL_VX = SPEED_MAX * youmen;

	// 2. 获取当前时间戳
    // 这是 PID 控制器计算时间步长 (dt) 所必需的
	hrt_abstime current_time_us = hrt_absolute_time();

	float wheel_powers[4];

	mode = 3;///1111111111111111111111111111111111111111111111111111111111111111111111111

	if(mode == 0){

		// 4. 当前轮子的转向角 (与机器人X轴的夹角，单位：弧度)
    	// 这个角度通常由另一个更上层的控制逻辑来设定
    	// 示例：如果轮子与车体X轴对齐（向前），则为 0 弧度
    	const float CURRENT_WHEEL_STEER_ANGLE = degrees;

		calculate_four_wheel_steer_drive_powers_advanced(
			DESIRED_GLOBAL_VX,
			_current_global_vx,
			_current_global_vy,
			_current_global_yaw,
			_desired_global_yaw,
			current_time_us,
			CURRENT_WHEEL_STEER_ANGLE,
			wheel_powers
		);
	}
	else if(mode == 1){
		// --- 旋转模式 ---
        // 在旋转模式下，轮子角度已经设定好，我们只需要给所有轮子相同的动力
        // 动力大小由油门杆决定
    		const float rotation_power = DESIRED_GLOBAL_VX;

        	for (int i = 0; i < 4; ++i) {
            		wheel_powers[i] = rotation_power;
        	}
	}

	for (int i = 0; i < 4; ++i) {
            	wheel_powers[i] = youmen;
        }

	(void)mode;

	// --- 1. 发布舵机角度指令 (gim6010_command) ---
	//    (已使用正确的字段名 'position' 进行修正)
	gim6010_command_s gim_cmd{};
	gim_cmd.timestamp = hrt_absolute_time();

	// 将计算出的轮子角度 (lunzi_jiaodu) 填充到消息的 'position' 数组中
	gim_cmd.position[0] = lunzi_jiaodu[1];
	gim_cmd.position[1] = lunzi_jiaodu[2];
	gim_cmd.position[2] = lunzi_jiaodu[3];
	gim_cmd.position[3] = lunzi_jiaodu[4];

	// 您可以根据需要选择性地填充速度和力矩，如果不需要则保持默认值0即可
	// gim_cmd.velocity[0] = ...;
	// gim_cmd.torque[0] = ...;

	// 使用在 .hpp 中定义的 publisher 发布消息
	_gim6010_command_pub.publish(gim_cmd);

	// printf("%f %f %f %f\n",(double)lunzi_jiaodu[1],(double)lunzi_jiaodu[2],(double)lunzi_jiaodu[3],(double)lunzi_jiaodu[4]);
	// --- 2. 发布电机动力指令 (actuator_motors) ---
	actuator_motors_s motors_cmd{};
	motors_cmd.timestamp = hrt_absolute_time();
	motors_cmd.timestamp_sample = motors_cmd.timestamp;
	motors_cmd.reversible_flags = 0b00001111;

	for (int i = 0; i < 4; ++i) {
		motors_cmd.control[i] = wheel_powers[i];
	}

	for (int i = 4; i < actuator_motors_s::NUM_CONTROLS; ++i) {
		motors_cmd.control[i] = NAN;
	}

	_actuator_motors_pub.publish(motors_cmd);


//------------------------------------------------------------------------------------------------------------------------
}

// 任务启动函数：任务初始化及创建
int swivelDrive::task_spawn(int argc, char *argv[])
{
	swivelDrive *instance = new swivelDrive();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

// 自定义命令处理函数
int swivelDrive::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

// 打印模块的使用说明
int swivelDrive::print_usage(const char *reason)
{
	if (reason) {
		PX4_ERR("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Module to read manual control input and print it out.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("swivelDrive", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

// 主函数：启动模块
extern "C" __EXPORT int swivel_robot_controller_main(int argc, char *argv[]) // <--- 修改函数名
{
    return swivelDrive::main(argc, argv);
}
