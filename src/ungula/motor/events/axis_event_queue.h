// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cstdint>
#include "ungula/hal/sync/critical_section.h"
#include "ungula/motor/result.h"
#include "ungula/motor/events/axis_event.h"

namespace ungula::motor
{

/// Bounded, lock-protected event queue. Producer (Axis service, possibly
/// running close to the ISR boundary) calls `enqueue()` with a compact
/// payload; consumer (host task) calls `drain()` and gets every pending
/// event in order, then invokes registered listeners outside the locked
/// region.
///
/// Listeners are notified from the drain context — never from ISR. The
/// queue capacity is fixed at compile time; overflow returns
/// `ErrorCode::QueueFull` rather than dropping events silently. Hosts
/// that can't drain fast enough should bump `CAPACITY` or move drain to
/// a higher-priority task.
inline constexpr uint8_t MAX_AXIS_EVENT_LISTENERS = 4;

template <uint8_t CAPACITY = 32> class AxisEventQueue {
    public:
        AxisEventQueue() = default;

        AxisEventQueue(const AxisEventQueue &) = delete;
        AxisEventQueue &operator=(const AxisEventQueue &) = delete;

        /// Producer side. ISR-safe via the HAL critical section.
        Status enqueue(const AxisEvent &ev) UNGULA_ISR_ATTR
        {
                ungula::hal::sync::ScopedLock lock(mux_);
                if (size_ >= CAPACITY) {
                        droppedEvents_++;
                        return Status::Err(ErrorCode::QueueFull);
                }
                ring_[head_] = ev;
                head_ = static_cast<uint8_t>((head_ + 1U) % CAPACITY);
                size_++;
                return Status::Ok();
        }

        /// Consumer side. Drains every pending event and invokes registered
        /// listeners outside the lock. Returns the number of events drained.
        uint32_t drain()
        {
                uint32_t drained = 0;
                while (true) {
                        AxisEvent ev;
                        {
                                ungula::hal::sync::ScopedLock lock(mux_);
                                if (size_ == 0U)
                                        break;
                                ev = ring_[tail_];
                                tail_ = static_cast<uint8_t>((tail_ + 1U) % CAPACITY);
                                size_--;
                        }
                        for (uint8_t i = 0; i < listenerCount_; ++i) {
                                if (listeners_[i] != nullptr) {
                                        listeners_[i]->onAxisEvent(ev);
                                }
                        }
                        drained++;
                }
                return drained;
        }

        Status subscribe(IAxisEventListener *listener)
        {
                if (listener == nullptr)
                        return Status::Err(ErrorCode::InvalidConfig);
                if (listenerCount_ >= MAX_AXIS_EVENT_LISTENERS) {
                        return Status::Err(ErrorCode::QueueFull);
                }
                listeners_[listenerCount_++] = listener;
                return Status::Ok();
        }

        uint8_t listenerCount() const
        {
                return listenerCount_;
        }

        /// Pending event count. Locked: a producer ISR may be writing `size_`
        /// at any moment, and reading without the lock would risk a stale or
        /// torn value (size_ is `uint8_t` so single-byte access is atomic on
        /// every supported target, but the lock also serialises a coherent
        /// view of `head_`/`tail_`/`size_` together).
        uint8_t pending() const
        {
                ungula::hal::sync::ScopedLock lock(mux_);
                return size_;
        }

        /// Total events dropped on `enqueue` when the queue was full. Locked
        /// for the same reason as `pending()`.
        uint32_t droppedEvents() const
        {
                ungula::hal::sync::ScopedLock lock(mux_);
                return droppedEvents_;
        }

    private:
        AxisEvent ring_[CAPACITY]{};
        uint8_t head_ = 0;
        uint8_t tail_ = 0;
        uint8_t size_ = 0;

        IAxisEventListener *listeners_[MAX_AXIS_EVENT_LISTENERS]{};
        uint8_t listenerCount_ = 0;

        uint32_t droppedEvents_ = 0;

        mutable ungula::hal::sync::CriticalSection mux_{};
};

} // namespace ungula::motor
