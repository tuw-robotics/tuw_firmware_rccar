#ifndef YAW_CORRECTION_USED_H
#define YAW_CORRECTION_USED_H

#include <sdkconfig.h>

#ifdef CONFIG_YAW_CORRECTION
#define YAW_CORRECTION CONFIG_YAW_CORRECTION
#else
// we set to 0 as when it is set to false it is not defined at all in sdkconfig
#define YAW_CORRECTION 0
#endif

#endif // YAW_CORRECTION_USED_H