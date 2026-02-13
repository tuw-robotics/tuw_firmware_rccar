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

builtin_interfaces__msg__Time time_add(const builtin_interfaces__msg__Time *summand_1, const builtin_interfaces__msg__Time *summand_2) {
    builtin_interfaces__msg__Time sum;
    sum.sec = summand_1->sec + summand_2->sec;
    int64_t sum_nanosec = (int64_t)summand_1->nanosec + (int64_t)summand_2->nanosec;

    if (sum_nanosec >= 1000000000LL) {
        sum.sec++;
        sum_nanosec -= 1000000000LL;
    }
    sum.nanosec = (uint32_t)sum_nanosec;

    return sum;
}

builtin_interfaces__msg__Time time_ns_to_time_msg(uint64_t nanoseconds) {
    builtin_interfaces__msg__Time time;
    time.sec = 0;

    while (nanoseconds >= 1000000000LL) {
        time.sec++;
        nanoseconds -= 1000000000LL;
    }
    time.nanosec = (uint32_t)nanoseconds;

    return time;
}

builtin_interfaces__msg__Time time_max(const builtin_interfaces__msg__Time *time_1, const builtin_interfaces__msg__Time *time_2) {
    if (time_1->sec > time_2->sec)
        return *time_1;
    if (time_1->sec < time_2->sec)
        return *time_2;
    if (time_1->nanosec > time_2->nanosec)
        return *time_1;
    if (time_1->nanosec < time_2->nanosec)
        return *time_2;
    return *time_1;
}

uint64_t time_to_us(const builtin_interfaces__msg__Time *time) { return (uint64_t)time->sec * 1000000LL + (uint64_t)time->nanosec / 1000LL; }
