// Stubs for platform-dependent functions needed by motor library desktop tests

#include <time/time_control.h>

// --- TimeControl stubs ---

namespace ungula {

    void TimeControl::delayMs(time_ms_t ms) {
        (void)ms;
    }
    void TimeControl::delayUs(time_us_t us) {
        (void)us;
    }
    TimeControl::ms_tick_t TimeControl::millis() {
        return 0;
    }
    TimeControl::us_tick_t TimeControl::micros() {
        return 0;
    }
    void TimeControl::delayUntilMs(ms_tick_t& ref, time_ms_t periodMs) {
        ref += periodMs;
    }
    void TimeControl::delayUntilUs(us_tick_t& ref, time_us_t periodUs) {
        ref += periodUs;
    }
    bool TimeControl::hasReachedMs(ms_tick_t now, ms_tick_t target) {
        return now >= target;
    }
    bool TimeControl::hasReachedUs(us_tick_t now, us_tick_t target) {
        return now >= target;
    }

}  // namespace ungula
