#pragma onece
#include <lib/mixer_module/mixer_module.hpp>

#include "../Dispatcher.hpp"

class VulcanMixingInterfaceM3508 : public  OutputModuleInterface
{
	static constexpr int MAX_ACTUATORS = 8;

    private:
        auto catch_m3508_fn =  [](const uavcan::CanRxFrame& frame){
            if(frame.id==0x201 || frame.id==0x202 || frame.id==0x203 || frame.id==0x204 || frame.id==0x205||frame.id==0x206||frame.id==0x207||frame.id==0x208)
                {return true;}
                else{
                    return false;
                }

        };

    typedef uavcan::MethodBinder<VulcanMixingInterfaceM3508 *,
		void (VulcanMixingInterfaceM3508::*)(const CanRxFrame &)> StatusCbBinder;


    uavcan::Subscriber<StatusCbBinder> _m3508_subscriber;

	public:
        VulcanMixingInterfaceM3508(pthread_mutex_t &node_mutex)
		: OutputModuleInterface(MODULE_NAME "-actuators-M3508", px4::wq_configurations::uavcan),
		  _node_mutex(node_mutex),
_m3508_subscriber(catch_m3508_fn,                                               // <--- 捕获函数
                        StatusCbBinder(this, &VulcanMixingInterfaceM3508::m3508_status_cb)) ,// <--- 回调函数绑定
         {



          }
	    bool updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated) override;

	    // void mixerChanged() override;

	    MixingOutput &mixingOutput() { return _mixing_output; }

    protected:
        void Run() override;

    private:
    	friend class VulcanNode;

    	pthread_mutex_t &_node_mutex;

	MixingOutput _mixing_output{"UAVCAN_M3508", MAX_ACTUATORS, *this, MixingOutput::SchedulingPolicy::Auto, false, false};
    	uint8_t		_rotor_count{0};
	uORB::PublicationMulti<esc_status_s> _esc_status_pub{ORB_ID(esc_status)};
	esc_status_s	_esc_status{};

    void m3508_status_cb(const uavcan::CanRxFrame &frame); // <--- 1. 声明回调函数




};
