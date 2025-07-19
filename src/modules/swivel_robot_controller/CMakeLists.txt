

px4_add_module(
    MODULE modules__swivel_robot_controller
    MAIN swivel_robot_controller # <--- 确保这里与 C++ 中的新函数名一致
    SRCS
        swivel_robot_controller.cpp
        swivel_robot_controller.hpp
        swivel_robot_pids/swivel_robot_pids.cpp
        swivel_robot_pids/swivel_robot_pids.hpp
    DEPENDS
        px4_work_queue
        modules__control_allocator # for parameter CA_R_REV
    MODULE_CONFIG
        module.yaml
)