#include "M3508.hpp"




void
VulcanMixingInterfaceM3508::update_outputs(bool stop_motors,uint16_t outputs[VulcanMixingInterfaceM3508::MAX_ACTUATORS],unsigned num_outputs)
{




}
void VulcanMixingInterfaceM3508::Run()
{
	pthread_mutex_lock(&_node_mutex);
	_mixing_output.update();
	_mixing_output.updateSubscriptions(false);
	pthread_mutex_unlock(&_node_mutex);
}
void UavcanMixingInterfaceESC::mixerChanged()
{
	int rotor_count = 0;

	for (unsigned i = 0; i < MAX_ACTUATORS; ++i) {
		rotor_count += _mixing_output.isFunctionSet(i);

		if (i < esc_status_s::CONNECTED_ESC_MAX) {
			_esc_controller.esc_status().esc[i].actuator_function = (uint8_t)_mixing_output.outputFunction(i);
		}
	}

	_rotor_count = rotor_count;
}
void VulcanMixingInterfaceM3508::m3508_status_cb(const uavcan::CanRxFrame &frame)
{
    // 在这里添加处理电机状态反馈的逻辑
    // 例如：解析 frame.data, 更新 _esc_status, 然后发布 uORB 消息
    PX4_INFO("Received M3508 status on CAN ID: 0x%X", frame.id);

    // 示例逻辑:
    // const int16_t speed = (int16_t)(frame.data[2] << 8 | frame.data[3]);
    // _esc_status.esc[some_index].esc_rpm = speed;
    // ... etc ...
}
