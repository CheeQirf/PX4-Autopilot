#include "Dispatcher.hpp"



namespace uavcan{

constexpr uint16_t bit_cmp_array[]={2048,1042,512,256,128,64,32,16,8,4,2,1,0};





int
Dispatcher::spinOnce()
{
    int num_frames_processed = 0;

    while (true)
    {
        CanIOFlags flags = 0;
        CanRxFrame frame;
        const int res = canio_.receive(frame, MonotonicTime(), flags);
        if (res < 0)
        {
            return res;
        }
        else if (res > 0)
        {
            if (flags & CanIOFlagLoopback)
            {
                handleLoopbackFrame(frame);
            }
            else
            {
                num_frames_processed++;
                handleFrame(frame);
            }
            // notifyRxFrameListener(frame, flags);
        }
        else
        {
            break;      // No frames left
        }
    }

    return num_frames_processed;



}


int
Dispatcher::send(const CanFrame& frame, MonotonicTime tx_deadline, MonotonicTime blocking_deadline, CanTxQueue::Qos qos,
             CanIOFlags flags, uint8_t iface_mask){
        return canio_.send(frame, tx_deadline, blocking_deadline, iface_mask, qos, flags);

             }


};


