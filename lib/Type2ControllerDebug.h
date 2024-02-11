#pragma once
#include "core_config.h"
#include "MTDebug.h"

#if TYPE2_CONTROLLER_DEBUG
#define DEBUG_LN_TYPE2_CONTROLLER(FORMAT, ...) \
    {                                          \
        DEBUG_LN(FORMAT, __VA_ARGS__)          \
    }
#define DEBUG_TYPE2_CONTROLLER(FORMAT, ...) \
    {                                       \
        DEBUG(FORMAT, __VA_ARGS__)          \
    }
#else
#define DEBUG_LN_TYPE2_CONTROLLER(FORMAT, ...) \
    {                                          \
    }
#define DEBUG_TYPE2_CONTROLLER(FORMAT, ...) \
    {                                       \
    }
#endif