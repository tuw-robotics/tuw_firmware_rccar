
#ifndef TIMING_UTILS_H
#define TIMING_UTILS_H

#include <builtin_interfaces/msg/time.h>
#include <stdint.h>
#include <time.h>

// Synchronized time
typedef struct timeval wallclock_timestamp_t;

// Un-synchronized time
typedef uint64_t monotonic_timestamp_t;

#define NO_WAIT 0
#define WAIT_FOREVER portMAX_DELAY
#define ONE_TICK 1

#define S_TO_MS(x) ((x) * 1000LL)
#define S_TO_US(x) ((x) * 1000000LL)
#define MS_TO_US(x) ((x) * 1000LL)
#define US_TO_MS(x) ((x) / 1000LL)
#define US_TO_NS(x) ((x) * 1000LL)
#define US_TO_TICKS(x) (US_TO_MS(x) / portTICK_PERIOD_MS)
#define US_TO_S(x) ((x) * 1e-6f)

#define NS_TO_S(ns) ((ns) / 1000000000LL)
#define NS_SUBS_TO_USEC(ns) (((ns) % 1000000000LL) / 1000LL)

/**
 * @brief Calculates the time difference between two Time messages
 *
 * @param start Pointer to the start Time message
 * @param end Pointer to the end Time message
 * @return builtin_interfaces__msg__Time The time difference as a Time message
 */
builtin_interfaces__msg__Time time_delta(const builtin_interfaces__msg__Time *start, const builtin_interfaces__msg__Time *end);

/**
 * @brief Gets the current wallclock time as a Time message
 *
 * @return builtin_interfaces__msg__Time The current wallclock time
 */
builtin_interfaces__msg__Time time_now();

/**
 * @brief Add two time msgs
 *
 * @param summand_1 time msg 1
 * @param summand_2 time msg 2
 * @return builtin_interfaces__msg__Time
 */
builtin_interfaces__msg__Time time_add(const builtin_interfaces__msg__Time *summand_1, const builtin_interfaces__msg__Time *summand_2);

/**
 * @brief Creates out of nanoseconds a time msg.
 *
 * @param nanoseconds
 * @return builtin_interfaces__msg__Time
 */
builtin_interfaces__msg__Time time_ns_to_time_msg(uint64_t nanoseconds);

/**
 * @brief Max of two time msgs
 *
 * @param time_1
 * @param time_2
 * @return builtin_interfaces__msg__Time
 */
builtin_interfaces__msg__Time time_max(const builtin_interfaces__msg__Time *time_1, const builtin_interfaces__msg__Time *time_2);

/**
 * @brief Returns time msg as microseconds
 *
 * @param time
 * @return uint64_t
 */
uint64_t time_to_us(const builtin_interfaces__msg__Time *time);

#endif // TIMING_UTILS_H
