#include <uavcan/error.hpp>
#include <uavcan/std.hpp>
#include <uavcan/build_config.hpp>
#include <uavcan/transport/can_io.hpp>
#include <uavcan/util/linked_list.hpp>
#include <uavcan/util/method_binder.hpp>
#include <uavcan/transport/perf_counter.hpp>
namespace uavcan{
class Dispatcher;


class BaseSubscriber :public LinkedListNode<BaseSubscriber>{
public:
    virtual ~BaseSubscriber() = default;
    // virtual int start() = 0;  // 统一接口
    virtual void handleMessage(const CanRxFrame& frame) = 0; // 新增虚函数

};
template<typename _CatchFn = bool(*)(const CanRxFrame&),typename _CallbackFn = void(*)(const CanRxFrame&)>
class Subscriber: public BaseSubscriber {

    public:
    typedef _CatchFn _Catch;
    typedef _CallbackFn _Callback;
        Subscriber(_Catch catchFn,_Callback callback);

    int  start(Dispatcher* instance){
        return instance->add_subscriber((static_cast<BaseSubscriber*> this));

    }

    void handleMessage(const CanRxFrame& frame) override {
        if (catchFn(frame)) { // 调用过滤器
            callbackFn(frame); // 调用回调
        }
    }

    private:
    _Catch catchFn;
    _Callback callbackFn;






};
class UAVCAN_EXPORT Dispatcher : Noncopyable
{
    CanIOManager canio_;
    ISystemClock& sysclock_;
    TransferPerfCounter perf_;




    LinkedListRoot<BaseSubscriber> _root;








    void handleFrame(const CanRxFrame& can_frame){
            auto node = _root.get();
            while(node!=nullptr){
                node->handleMessage(can_frame);
                node = node->getNextListNode();
            }
    };

    void handleLoopbackFrame(const CanRxFrame& can_frame){

    };

    // void notifyRxFrameListener(const CanRxFrame& can_frame, CanIOFlags flags);

public:
    Dispatcher(ICanDriver& driver, IPoolAllocator& allocator, ISystemClock& sysclock)
        : canio_(driver, allocator, sysclock)
        , sysclock_(sysclock)

    { }

    /**
     * This version returns strictly when the deadline is reached.
     */
    // int spin(MonotonicTime deadline);

    /**
     * This version does not return until all available frames are processed.
     */
    int spinOnce();

    /**
     * Refer to CanIOManager::send() for the parameter description
    //  */
    // int send();

    // void cleanup(MonotonicTime ts);

    int add_subscriber(BaseSubscriber* suber){
            _root.insert(suber);
            return 0;
    };



};


}
