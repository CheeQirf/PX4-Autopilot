#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/drivers/device/Device.hpp>
#include <lib/mixer_module/mixer_module.hpp>
#include <lib/perf/perf_counter.h>

#include "Dispatcher.hpp"

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

	static int	start(uavcan::NodeID node_id, uint32_t bitrate);
	void		print_info();
	int			 list_params(int remote_node_id);
	int			 save_params(int remote_node_id);
	int			 set_param(int remote_node_id, const char *name, char *value);
	int			 get_param(int remote_node_id, const char *name);
	int			 reset_node(int remote_node_id);
private:
	void Run() override;
	int		init();
	void 		update_params();
	static VulcanNode	*_instance;			///< singleton pointer
	uavcan::Dispatcher    _node;
	bool                    _node_init{false};
	perf_counter_t			_cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle time")};
	perf_counter_t			_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": cycle interval")};
	pthread_mutex_t			_node_mutex;
	hrt_abstime _last_can_status_pub{0};
	orb_advert_t _can_status_pub_handles[UAVCAN_NUM_IFACES] = {nullptr};
	px4::atomic_bool	_task_should_exit{false};	///< flag to indicate to tear down the CAN driver

};
