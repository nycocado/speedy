#include "ROSCore.h"

ROSCore::ROSCore() {}

bool ROSCore::begin(const char* nodeName)
{
    _allocator = rcl_get_default_allocator();

    if (rclc_support_init(&_support, 0, NULL, &_allocator) != RCL_RET_OK)
    {
        return false;
    }

    if (rclc_node_init_default(&_node, nodeName, "", &_support) != RCL_RET_OK)
    {
        return false;
    }

    return true;
}

bool ROSCore::initExecutor(size_t numHandles)
{
    if (rclc_executor_init(
            &_executor, &_support.context, numHandles, &_allocator
        ) != RCL_RET_OK)
    {
        return false;
    }

    return true;
}

void ROSCore::end()
{
    rclc_executor_fini(&_executor);
    rcl_node_fini(&_node);
    rclc_support_fini(&_support);
}

void ROSCore::spinSome(const uint32_t timeoutMs)
{
    rclc_executor_spin_some(&_executor, RCL_MS_TO_NS(timeoutMs));
}
