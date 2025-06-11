#include <vulcan_main.hpp>
// 静态成员变量定义
VulcanNode* VulcanNode::_instance = nullptr;
static VulcanNode::CanInitHelper * can = nullptr;
// 构造函数
VulcanNode::VulcanNode(uavcan::ICanDriver& can_driver, uavcan::ISystemClock& system_clock)
    : px4::ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::uavcan),
      ModuleParams(nullptr),
      _dispatcher(can_driver, *can_driver.getAllocator(), system_clock),
      _node_init(false) {
	// int res = pthread_mutex_init()

}

// 析构函数
VulcanNode::~VulcanNode() {
    if (_instance) {
        _task_should_exit.store(true);
	SchedualNow();
			do {
			/* wait 5ms - it should wake every 10ms or so worst-case */
			usleep(5000);

			if (--i == 0) {
				break;
			}

		} while (_instance);
    }
	perf_free(_cycle_perf);
	perf_free(_interval_perf);


}

// 静态启动函数（单例入口）
int VulcanNode::start(uavcan::NodeID node_id, uint32_t bitrate) {
    if (_instance != nullptr) {
        PX4_WARN("VulcanNode already started");
        return -1;
    }
    if(can == nullptr){
	can = new CanInitHelper(board_get_can_interfaces());
		if (can == nullptr) {  // We don't have exceptions so bad_alloc cannot be thrown
			PX4_ERR("Out of memory");
			return -1;
		}
    }
    // 获取 CAN 驱动实例（假设已定义）
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
	int32_t bitrate = 1000000;
	(void)param_get(param_find("UAVCAN_BITRATE"), &bitrate);
	const int can_init_res = can->init(bitrate);

		if (can_init_res < 0) {
			PX4_ERR("CAN driver init failed %i", can_init_res);
		}

		_instance->init(node_id, can->driver.updateEvent());

		_node_init = true;

    }
    	pthread_mutex_lock(&_node_mutex);

	perf_begin(_cycle_perf);
	perf_count(_interval_perf);
	_node.spinOnce();
    // 执行调度器（处理 CAN 消息）
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

			auto iface_perf_cnt = _node..getCanIOManager().getIfacePerfCounters(i);
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

	pthread_mutex_unlock(&_node_mutex);


	if (_task_should_exit.load()) {

		ScheduleClear();
		_instance = nullptr;
	}
}

// 初始化节点逻辑
int VulcanNode::init() {
    // 1. 初始化 CAN 驱动
    int res = CanInitHelper::init(_dispatcher.getCanDriver(), _dispatcher.getSystemClock());
    if (res < 0) {
        PX4_ERR("CAN initialization failed: %d", res);
        return res;
    }

    // 2. 初始化节点信息
    uavcan::protocol::NodeInfo node_info;
    node_info.setName("org.pixhawk.vulcan_node");
    node_info.setSoftwareVersionMajor(1);
    node_info.setHardwareVersionMajor(1);
    _dispatcher.setNodeInfo(node_info);

    // 3. 启动参数服务
    if (_dispatcher.startParamServer() < 0) {
        PX4_ERR("Failed to start parameter server");
        return -1;
    }

    // 4. 注册回调函数（示例：参数更新）
    if (_dispatcher.registerParamUpdateHandler(
            [this](uavcan::protocol::param::GetSet::Request& req, uavcan::protocol::param::GetSet::Response& resp) {
                this->set_param_handler(req, resp);
            }) < 0) {
        PX4_ERR("Failed to register parameter update handler");
        return -1;
    }

    PX4_INFO("VulcanNode initialized");
    return 0;
}

// 打印调试信息
void VulcanNode::print_info() {
    PX4_INFO("VulcanNode Status:");
    PX4_INFO(" - Node ID: %d", _dispatcher.getNodeID().get());
    PX4_INFO(" - CAN Bitrate: %lu bps", static_cast<unsigned long>(_dispatcher.getBitrate()));
    PX4_INFO(" - Parameters: %d registered", _dispatcher.getNumParams());
}

// 列出远程节点参数
int VulcanNode::list_params(int remote_node_id) {
    if (!_dispatcher.isNodeOnline(remote_node_id)) {
        PX4_ERR("Remote node %d is offline", remote_node_id);
        return -ENODEV;
    }

    // 请求远程节点的参数列表
    uavcan::protocol::param::GetSet::Request req;
    req.name = "list"; // 假设参数名称为 "list" 表示请求参数列表
    uavcan::protocol::param::GetSet::Response resp;
    int res = _dispatcher.getParam(remote_node_id, req, resp);
    if (res < 0) {
        PX4_ERR("Failed to list parameters for node %d: %d", remote_node_id, res);
        return res;
    }

    // 解析并打印参数
    PX4_INFO("Parameters for node %d:", remote_node_id);
    for (const auto& param : resp.params) {
        PX4_INFO("  %s = %s", param.name.c_str(), param.value.toString().c_str());
    }

    return 0;
}

// 保存远程节点参数
int VulcanNode::save_params(int remote_node_id) {
    // 发送保存命令（假设参数名称为 "save_all"）
    uavcan::protocol::param::GetSet::Request req;
    req.name = "save_all";
    uavcan::protocol::param::GetSet::Response resp;
    int res = _dispatcher.getParam(remote_node_id, req, resp);
    if (res < 0) {
        PX4_ERR("Failed to save parameters for node %d: %d", remote_node_id, res);
        return res;
    }

    PX4_INFO("Parameters for node %d saved", remote_node_id);
    return 0;
}

// 设置远程节点参数
int VulcanNode::set_param(int remote_node_id, const char* name, char* value) {
    if (!name || !value) {
        PX4_ERR("Invalid parameter name or value");
        return -EINVAL;
    }

    // 构造参数请求
    uavcan::protocol::param::GetSet::Request req;
    req.name = name;
    req.value.fromString(value); // 假设支持字符串赋值

    uavcan::protocol::param::GetSet::Response resp;
    int res = _dispatcher.getParam(remote_node_id, req, resp);
    if (res < 0) {
        PX4_ERR("Failed to set parameter %s on node %d: %d", name, remote_node_id, res);
        return res;
    }

    PX4_INFO("Parameter %s set to %s", name, value);
    return 0;
}

// 获取远程节点参数
int VulcanNode::get_param(int remote_node_id, const char* name) {
    if (!name) {
        PX4_ERR("Invalid parameter name");
        return -EINVAL;
    }

    // 构造参数请求
    uavcan::protocol::param::GetSet::Request req;
    req.name = name;
    uavcan::protocol::param::GetSet::Response resp;
    int res = _dispatcher.getParam(remote_node_id, req, resp);
    if (res < 0) {
        PX4_ERR("Failed to get parameter %s on node %d: %d", name, remote_node_id, res);
        return res;
    }

    PX4_INFO("Parameter %s = %s", name, resp.value.toString().c_str());
    return 0;
}

// 重置远程节点
int VulcanNode::reset_node(int remote_node_id) {
    // 使用 UAVCAN 协议发送重置命令（假设存在 Reset 服务）
    uavcan::protocol::Restart::Request req;
    uavcan::protocol::Restart::Response resp;
    int res = _dispatcher.callService(remote_node_id, req, resp);
    if (res < 0) {
        PX4_ERR("Failed to reset node %d: %d", remote_node_id, res);
        return res;
    }

    PX4_INFO("Node %d reset successfully", remote_node_id);
    return 0;
}

// 参数更新回调处理
void VulcanNode::set_param_handler(uavcan::protocol::param::GetSet::Request& req,
                                  uavcan::protocol::param::GetSet::Response& resp) {
    if (req.name == "example_param") {
        // 更新本地参数
        if (req.value.isInteger()) {
            int32_t val = req.value.toInteger();
            PX4_INFO("Received param update: %s = %d", req.name.c_str(), val);
            // 应用新值到模块参数
            // 示例：param_set_int(..., val);
        } else {
            PX4_WARN("Invalid parameter type for %s", req.name.c_str());
        }
    } else {
        PX4_WARN("Unknown parameter: %s", req.name.c_str());
    }
}
