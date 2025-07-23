#include "vulcan_main.hpp"
// 静态成员变量定义

using namespace uavcan;
VulcanNode* VulcanNode::_instance = nullptr;
static VulcanNode::CanInitHelper * can = nullptr;
// 构造函数
VulcanNode::VulcanNode(uavcan::ICanDriver& can_driver, uavcan::ISystemClock& system_clock)
    : px4::ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::uavcan),
      ModuleParams(nullptr),
      _node(can_driver, _pool_allocator, system_clock),
      _node_init(false) ,
          _test_motor(this,_node_mutex),
	  _m3508_motor(this,_node_mutex),
	  _gim6010_motor(this,_node_mutex)

{


	PX4_INFO("Vulcan node instance init");
    int res = pthread_mutex_init(&_node_mutex, nullptr);
	int32_t uavcan_enable = 1;
	(void)param_get(param_find("VULCAN_ENABLE"), &uavcan_enable);
	if (res < 0) {
		PX4_ERR("error create vulcan pthread mutex");
		std::abort();
	}
	_test_motor.mixingOutput().setMaxTopicUpdateRate(1000000 / 400);
	_m3508_motor.mixingOutput().setMaxTopicUpdateRate(1000000 / 125);

}

// 析构函数
VulcanNode::~VulcanNode() {
        if (_instance) {

		/* tell the task we want it to go away */
		_task_should_exit.store(true);
		ScheduleNow();

		unsigned i = 10;

		do {
			/* wait 5ms - it should wake every 10ms or so worst-case */
			usleep(5000);

			if (--i == 0) {
				break;
			}

		} while (_instance);
	}

    	pthread_mutex_destroy(&_node_mutex);

	perf_free(_cycle_perf);
	perf_free(_interval_perf);


}

// 静态启动函数（单例入口）
int VulcanNode::start(uint32_t bitrate) {
    if (_instance != nullptr) {
        PX4_WARN("VulcanNode already started");
        return -1;
    }
    if(can == nullptr){
	PX4_INFO("ready to create can instance");
	can = new CanInitHelper(board_get_can_interfaces());
	PX4_INFO("Create can instance");
		if (can == nullptr) {  // We don't have exceptions so bad_alloc cannot be thrown
			PX4_ERR("Out of memory");
			return -1;
		}
    }

    _instance = new VulcanNode(can->driver,UAVCAN_DRIVER::SystemClock::instance());
	if (_instance == nullptr) {
		PX4_ERR("Out of memory");
		return -1;
	}
    _instance->ScheduleOnInterval(ScheduleIntervalMs * 1000);



    return 0;
}

// 调度工作项的主函数
void VulcanNode::Run() {
    if (!_node_init) {
	PX4_INFO("vulcan node init in Run impl");
	int32_t bitrate = 1000000;
	(void)param_get(param_find("VULCAN_BITRATE"), &bitrate);
	const int can_init_res = can->init(bitrate);

		if (can_init_res != 0) {
			while(1){PX4_ERR("CAN driver init failed %i", can_init_res);}
		}

		_instance->init(can->driver.updateEvent());

		_node_init = true;
		    //_instance->_test_motor.ScheduleNow();
		    _instance->_m3508_motor.ScheduleNow();
		     _instance->_gim6010_motor.ScheduleNow();
		     _instance->_gim6010_motor.ScheduleOnInterval(20_ms);
    }

	perf_begin(_cycle_perf);
	perf_count(_interval_perf);
	pthread_mutex_lock(&_node_mutex);
	_node.spinOnce();   // 执行调度器（处理 CAN 消息）
    	pthread_mutex_unlock(&_node_mutex);

	constexpr hrt_abstime status_pub_interval = 100_ms;

    //发布can状态消息
  if (hrt_absolute_time() - _last_can_status_pub >= status_pub_interval) {
		_last_can_status_pub = hrt_absolute_time();

		for (int i = 0; i < _node.getCanIOManager().getCanDriver().getNumIfaces(); i++) {
			if (i > UAVCAN_NUM_IFACES) {
				break;
			}

			auto iface = _node.getCanIOManager().getCanDriver().getIface(i);

			if (!iface) {
				continue;
			}

			auto iface_perf_cnt = _node.getCanIOManager().getIfacePerfCounters(i);
			can_interface_status_s status{
				.timestamp = hrt_absolute_time(),
				.io_errors = iface_perf_cnt.errors,
				.frames_tx = iface_perf_cnt.frames_tx,
				.frames_rx = iface_perf_cnt.frames_rx,
				.interface = static_cast<uint8_t>(i),
			};

			if (_can_status_pub_handles[i] == nullptr) {
				int instance{0};
				_can_status_pub_handles[i] = orb_advertise_multi(ORB_ID(can_interface_status), nullptr, &instance);
			}

			(void)orb_publish(ORB_ID(can_interface_status), _can_status_pub_handles[i], &status);
		}
	}
	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		update_params();
	}
	perf_end(_cycle_perf);



	if (_task_should_exit.load()) {

		ScheduleClear();
		_instance = nullptr;
	}
}

// 初始化节点逻辑
int VulcanNode::init(UAVCAN_DRIVER::BusEvent &bus_events) {
    // 1. 初始化 CAN 驱动
    bus_events.registerSignalCallback(VulcanNode::busevent_signal_trampoline);

    return OK;
}

// 打印调试信息
void VulcanNode::print_info() {
    	(void)pthread_mutex_lock(&_node_mutex);

	// Memory status
	printf("Pool allocator status:\n");
	printf("\tCapacity hard/soft: %" PRIu16 "/%" PRIu16 " blocks\n",
	       _pool_allocator.getBlockCapacityHardLimit(), _pool_allocator.getBlockCapacity());
	printf("\tReserved:  %" PRIu16 " blocks\n", _pool_allocator.getNumReservedBlocks());
	printf("\tAllocated: %" PRIu16 " blocks\n", _pool_allocator.getNumAllocatedBlocks());

	printf("\n");

	// UAVCAN node perfcounters
	// printf("VULCAN node status:\n");
	// printf("\tInternal failures: %" PRIu64 "\n", _node.getInternalFailureCount());
	// printf("\tTransfer errors:   %" PRIu64 "\n", _node.getTransferPerfCounter().getErrorCount());
	// printf("\tRX transfers:      %" PRIu64 "\n", _node.getTransferPerfCounter().getRxTransferCount());
	// printf("\tTX transfers:      %" PRIu64 "\n", _node.getTransferPerfCounter().getTxTransferCount());
	// auto timer = UAVCAN_DRIVER::SystemClock::instance();
	printf("Vulcan TimeBase MonoticTime: %lld ticks",UAVCAN_DRIVER::clock::getMonotonic().toUSec());
	printf("\n");

	// CAN driver status
	for (unsigned i = 0; i < _node.getCanIOManager().getCanDriver().getNumIfaces(); i++) {
		printf("CAN%u status:\n", unsigned(i + 1));

		auto iface = _node.getCanIOManager().getCanDriver().getIface(i);

		if (iface) {
			printf("\tHW errors: %" PRIu64 "\n", iface->getErrorCount());

			auto iface_perf_cnt = _node.getCanIOManager().getIfacePerfCounters(i);
			printf("\tIO errors: %" PRIu64 "\n", iface_perf_cnt.errors);
			printf("\tRX frames: %" PRIu64 "\n", iface_perf_cnt.frames_rx);
			printf("\tTX frames: %" PRIu64 "\n", iface_perf_cnt.frames_tx);
		}
	}

	printf("\n");

	_test_motor._mixing_output.printStatus();
	_m3508_motor._mixing_output.printStatus();
	perf_print_counter(_cycle_perf);
	perf_print_counter(_interval_perf);



	_m3508_motor.print_info();



    (void)pthread_mutex_unlock(&_node_mutex);

}

// // 列出远程节点参数
void
VulcanNode::update_params()
{

	_test_motor.updateParams();
	_m3508_motor.updateParams();
}


void
VulcanNode::busevent_signal_trampoline()
{
	if (_instance) {
		// trigger the work queue (Note, this is called from IRQ context)
		_instance->ScheduleNow();
	}
}
int
VulcanNode::send(const CanFrame& frame, MonotonicTime tx_deadline, MonotonicTime blocking_deadline, CanTxQueue::Qos qos,
             CanIOFlags flags, uint8_t iface_mask)
{
    return _node.send(frame, tx_deadline, blocking_deadline, qos,
             flags, iface_mask);

}

static void print_usage()
{
	PX4_INFO("usage: \n"
		 "\tuavcan {start|status|stop|shrink|update}\n"
		 "\t        param [set|get|list|save] <node-id> <name> <value>|reset <node-id>");
}


extern "C" __EXPORT int vulcan_main(int argc, char *argv[])
{
	//VulcanNode *const inst = VulcanNode::instance();
	if (argc < 2) {
		print_usage();
		::exit(1);
	}

	if (!std::strcmp(argv[1], "start")) {
		if (VulcanNode::instance()) {
			// Already running, no error
			PX4_INFO("already started");
			::exit(0);
		}

		// CAN bitrate
		int32_t bitrate = 1000000;
		(void)param_get(param_find("VULCAN_BITRATE"), &bitrate);

		// Start
		PX4_INFO("Vulcan start bitrate %" PRIu32,bitrate);
		return VulcanNode::start(bitrate);
	}

	/* commands below require the app to be started */
	VulcanNode *const inst = VulcanNode::instance();

	if (!inst) {
		errx(1, "application not running");
	}

	// if (!std::strcmp(argv[1], "update")) {
	// 	if (UavcanNode::instance() == nullptr) {
	// 		errx(1, "firmware server is not running");
	// 	}

	// 	UavcanNode::instance()->requestCheckAllNodesFirmwareAndUpdate();
	// 	::exit(0);
	// }

	if (!std::strcmp(argv[1], "status") || !std::strcmp(argv[1], "info")) {
		inst->print_info();
		::exit(0);
	}

	// if (!std::strcmp(argv[1], "shrink")) {
	// 	inst->shrink();
	// 	::exit(0);
	// }

	/*
	 * Parameter setting commands
 *
	 *  uavcan param list <node>
	 *  uavcan param save <node>
	 *  uavcan param get <node> <name>
	 *  uavcan param set <node> <name> <value>
	 *
	 */
	// int node_arg = !std::strcmp(argv[1], "reset") ? 2 : 3;


	if (!std::strcmp(argv[1],"gim6010")){
		inst->publishGim6010Command(-1.57f,0.0f, 0.0f);
		::exit(0);
	}

	if (!std::strcmp(argv[1], "stop")) {
		delete inst;
		::exit(0);
	}

	print_usage();
	::exit(1);
}
