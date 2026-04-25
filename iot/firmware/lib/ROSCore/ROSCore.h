#pragma once

#include <micro_ros_platformio.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

/**
 * @class ROSCore
 * @brief Gerencia o ciclo de vida base do micro-ROS (Nó, Suporte, Alocador e
 * Executor).
 */
class ROSCore
{
    public:
        ROSCore();
        bool begin(const char* nodeName);
        void end();
        bool initExecutor(size_t numHandles);
        void spinSome(const uint32_t timeoutMs);

        rcl_node_t* getNode() { return &_node; }
        rclc_executor_t* getExecutor() { return &_executor; }

    private:
        rclc_support_t _support;
        rcl_allocator_t _allocator;
        rcl_node_t _node;
        rclc_executor_t _executor;
};
