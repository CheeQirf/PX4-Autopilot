#include <uavcan/error.hpp>
#include <uavcan/std.hpp>
#include <uavcan/build_config.hpp>
#include <uavcan/transport/can_io.hpp>
#include <uavcan/util/linked_list.hpp>
#include <uavcan/util/method_binder.hpp>
#include <uavcan/driver/can.hpp>
namespace uavcan {

// Forward declare Dispatcher
class Dispatcher;

class BaseSubscriber : public LinkedListNode<BaseSubscriber> {
public:
    virtual ~BaseSubscriber() = default;
    virtual void handleMessage(const CanRxFrame& frame) = 0;
};

template<typename _CallbackFn = void(*)(const CanRxFrame&),typename _CatchFn = bool(*)(const CanRxFrame&)>
class Subscriber : public BaseSubscriber {
public:
    typedef _CatchFn _Catch;
    typedef _CallbackFn _Callback;

    // The constructor parameters `catchFn` and `callback` no longer shadow the member variables.
    Subscriber(_Catch catchFn, _Callback callback) : catchFn_(catchFn), callbackFn_(callback) {}


    int start(Dispatcher* instance);

    void handleMessage(const CanRxFrame& frame) override {
        // Use the member variables with the underscore suffix
        if(catchFn_==nullptr || !callbackFn_)return;
        if (catchFn_(frame)) {
            callbackFn_(frame);
        }
    }

private:
    // **FIX:** Rename member variables to use a trailing underscore, matching project style.
    _Catch catchFn_;
    _Callback callbackFn_;
};

class UAVCAN_EXPORT Dispatcher : Noncopyable {
    CanIOManager canio_;
    ISystemClock& sysclock_;
    // TransferPerfCounter perf_;

    LinkedListRoot<BaseSubscriber> _root;

    void handleFrame(const CanRxFrame& can_frame) {
        auto node = _root.get();
        while (node != nullptr) {
            node->handleMessage(can_frame);
            node = node->getNextListNode();
        }
    };

    void handleLoopbackFrame(const CanRxFrame& can_frame) {
        // Implementation
    };

public:
    Dispatcher(ICanDriver& driver, IPoolAllocator& allocator, ISystemClock& sysclock)
        : canio_(driver, allocator, sysclock), sysclock_(sysclock) {}

    int spinOnce();

    int add_subscriber(BaseSubscriber* suber) {
        _root.insert(suber);
        return 0;
    };
    int send(const CanFrame& frame, MonotonicTime tx_deadline, MonotonicTime blocking_deadline, CanTxQueue::Qos qos,
             CanIOFlags flags, uint8_t iface_mask);








	const ISystemClock& getSystemClock() const { return sysclock_; }
    	ISystemClock& getSystemClock() { return sysclock_; }

  	  const CanIOManager& getCanIOManager() const { return canio_; }
   	 CanIOManager& getCanIOManager() { return canio_; }







};

// The definition of start() remains after the full definition of Dispatcher.
template<typename _CatchFn, typename _CallbackFn>
int Subscriber<_CatchFn, _CallbackFn>::start(Dispatcher* instance) {
    return instance->add_subscriber(static_cast<BaseSubscriber*>(this));





}

} // namespace uavcan
