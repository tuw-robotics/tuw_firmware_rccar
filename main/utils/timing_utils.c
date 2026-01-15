#include "timing_utils.h"

#include <stdint.h>
#include <sys/time.h>

builtin_interfaces__msg__Time time_delta(const builtin_interfaces__msg__Time *start, const builtin_interfaces__msg__Time *end) {
    builtin_interfaces__msg__Time delta;
    delta.sec = end->sec - start->sec;
    int64_t delta_nanosec = (int64_t)end->nanosec - (int64_t)start->nanosec;

    if (delta_nanosec < 0) {
        delta.sec--;
        delta_nanosec += 1000000000LL;
    }
    delta.nanosec = (uint32_t)delta_nanosec;

    return delta;
}

builtin_interfaces__msg__Time time_now() {
    wallclock_timestamp_t timestamp_local_wallclock;
    gettimeofday(&timestamp_local_wallclock, NULL);
    builtin_interfaces__msg__Time timestamp_local;
    timestamp_local.sec = timestamp_local_wallclock.tv_sec;
    timestamp_local.nanosec = US_TO_NS(timestamp_local_wallclock.tv_usec);
    return timestamp_local;
}
