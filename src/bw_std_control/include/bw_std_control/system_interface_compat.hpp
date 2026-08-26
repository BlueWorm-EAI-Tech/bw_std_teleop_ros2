#ifndef BW_STD_CONTROL__SYSTEM_INTERFACE_COMPAT_HPP_
#define BW_STD_CONTROL__SYSTEM_INTERFACE_COMPAT_HPP_

#include "hardware_interface/version.h"

// Humble(2.x) 导出裸值接口；Jazzy(4.x) 使用框架托管的共享接口。
#if HARDWARE_INTERFACE_VERSION_MAJOR >= 4
#define BW_STD_CONTROL_JAZZY_API 1
#else
#define BW_STD_CONTROL_JAZZY_API 0
#endif

#endif  // BW_STD_CONTROL__SYSTEM_INTERFACE_COMPAT_HPP_
