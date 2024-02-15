#pragma once
#include "core_config.h"

#if ACSOCKET_CONTROLLER_DEBUG
#define DEBUG_LN_ACSOCKET_CONTROLLER(FORMAT, ...) \
    {                                             \
        DEBUG_LN(FORMAT, __VA_ARGS__)             \
    }
#define DEBUG_ACSOCKET_CONTROLLER(FORMAT, ...) \
    {                                          \
        DEBUG(FORMAT, __VA_ARGS__)             \
    }
#else
#define DEBUG_LN_ACSOCKET_CONTROLLER(FORMAT, ...) \
    {                                             \
    }
#define DEBUG_ACSOCKET_CONTROLLER(FORMAT, ...) \
    {                                          \
    }
#endif