#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/drivers/device/Device.hpp>
#include "allocator.hpp"
#include "uavcan_driver.hpp"
#include <lib/mixer_module/mixer_module.hpp>
#include <lib/perf/perf_counter.h>


#include "Actuator/Motors.hpp"


#include "Dispatcher.hpp"
#include <uavcan/driver/can.hpp>
#include <uavcan/time.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/can_interface_status.h>
#include <uORB/topics/parameter_update.h>



class VulcanNode: public px4::ScheduledWorkItem, public ModuleParams{
	static constexpr unsigned MaxBitRatePerSec	= 1000000;
	static constexpr unsigned bitPerFrame		= 148;
	static constexpr unsigned FramePerSecond	= MaxBitRatePerSec / bitPerFrame;
	static constexpr unsigned FramePerMSecond	= ((FramePerSecond / 1000) + 1);

	static constexpr unsigned ScheduleIntervalMs		= 3;

	static constexpr unsigned RxQueueLenPerIface	= FramePerMSecond * ScheduleIntervalMs; // At




public:
	typedef UAVCAN_DRIVER::CanInitHelper<RxQueueLenPerIface> CanInitHelper;
	VulcanNode(uavcan::ICanDriver &can_driver, uavcan::ISystemClock &system_clock);
	virtual		~VulcanNode();

	static int	start(uint32_t bitrate);
	void		print_info();
	static void busevent_signal_trampoline();
	static VulcanNode	*instance() { return _instance; }
	int send(const uavcan::CanFrame& frame, uavcan::MonotonicTime tx_deadline, uavcan::MonotonicTime blocking_deadline, uavcan::CanTxQueue::Qos qos,
             uavcan::CanIOFlags flags, uint8_t iface_mask);




private:
	void Run() override;
	int		init( UAVCAN_DRIVER::BusEvent &bus_events);
	void 		update_params();


	uavcan_node::Allocator _pool_allocator;


	static VulcanNode	*_instance;			///< singleton pointer
	uavcan::Dispatcher    _node;
	bool                    _node_init{false};
	perf_counter_t			_cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle time")};
	perf_counter_t			_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": cycle interval")};
	pthread_mutex_t			_node_mutex;
	hrt_abstime _last_can_status_pub{0};


	orb_advert_t _can_status_pub_handles[UAVCAN_NUM_IFACES] = {nullptr};


	uORB::PublicationMulti<can_interface_status_s> _can_status_pub{ORB_ID(can_interface_status)};
	uORB::SubscriptionInterval	_parameter_update_sub{ORB_ID(parameter_update), 1_s};
	// uORB::Subscription _param_request_sub{ORB_ID(uavcan_parameter_request)};


	px4::atomic_bool	_task_should_exit{false};	///< flag to indicate to tear down the CAN driver




	//mixer
	VulcanMixingInterfaceTest _test_motor;

};
