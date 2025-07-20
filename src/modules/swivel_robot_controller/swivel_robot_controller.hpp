#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/differential_drive_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>

#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/gim6010_command.h>
#include <uORB/topics/actuator_motors.h>


class swivelDrive : public ModuleBase<swivelDrive>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	swivelDrive();
	~swivelDrive() override = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

protected:
	void updateParams() override;

private:
	void Run() override;
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};

	uORB::Subscription _local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};

	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Publication<differential_drive_setpoint_s> _differential_drive_setpoint_pub{ORB_ID(differential_drive_setpoint)};
	uORB::Publication<gim6010_command_s> _gim6010_command_pub{ORB_ID(gim6010_command)};
	uORB::Publication<actuator_motors_s> _actuator_motors_pub{ORB_ID(actuator_motors)};

	bool _manual_driving = false;
	bool _mission_driving = false;
	bool _acro_driving = false;
	hrt_abstime _time_stamp_last{0}; /**< time stamp when task was last updated */

	float youmen{0.f};
	float _max_speed{0.f};
	float _max_angular_velocity{0.f};

	float _desired_global_yaw{0.0f}; // 存储目标偏航角，初始化为0
    	int _previous_mode{0};           // 存储上一帧的模式
	float lunzi_jiaodu[5];
	// DEFINE_PARAMETERS(
	// 	(ParamFloat<px4::params::RDD_ANG_SCALE>) _param_rdd_ang_velocity_scale,
	// 	(ParamFloat<px4::params::RDD_SPEED_SCALE>) _param_rdd_speed_scale,
	// 	(ParamFloat<px4::params::RDD_WHEEL_BASE>) _param_rdd_wheel_base,
	// 	(ParamFloat<px4::params::RDD_WHEEL_SPEED>) _param_rdd_wheel_speed,
	// 	(ParamFloat<px4::params::RDD_WHEEL_RADIUS>) _param_rdd_wheel_radius,
	// 	(ParamFloat<px4::params::COM_SPOOLUP_TIME>) _param_com_spoolup_time
	// )
};
