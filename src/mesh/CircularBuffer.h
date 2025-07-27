#pragma once

#include <memory>
#ifdef ARDUINO_ARCH_RP2040
// See below #define GUARD_LOCK
#else
#include <bits/std_mutex.h>
#endif

#ifdef ARDUINO_ARCH_RP2040
#define GUARD_LOCK
/**
 * We disable locks for interrupt based systems
 * See: https://www.freertos.org/Documentation/02-Kernel/02-Kernel-features/02-Queues-mutexes-and-semaphores/04-Mutexes
 * Mutexes should not be used from an interrupt because:
 * <> They include a priority inheritance mechanism which only makes sense if the mutex is given and taken from a task, not an interrupt.
 * <> An interrupt cannot block to wait for a resource that is guarded by a mutex to become available.
 */
#else
#define GUARD_LOCK std::lock_guard lock(mutex)
#endif


template<class T>
class CircularBuffer {
public:
    //We allocate room for the specified size plus the end item to allow use for null terminated strings
    explicit CircularBuffer(uint16_t size): buffer(std::unique_ptr<T[]>(new T[size + 1])),
                                            fixed_size(size) {
    }

    void write(const T item) {
        GUARD_LOCK;

        will_clear_if_empty = false;
        buffer[head] = item;
        buffer[head + 1] = end_item;

        if (maxed) {
            tail = (head + 2) % fixed_size;
        }
        head = (head + 1) % fixed_size;
        maxed = (head == tail - 1);
    }

    void write(const T *items) {
        GUARD_LOCK;

        will_clear_if_empty = false;
        uint16_t pos = 0;
        uint16_t count = 0;
        while (items[count] != end_item) {
            pos = (head + count) % fixed_size;
            buffer[pos] = items[count++];
            if (((pos + 1) % fixed_size) == tail - 1 || (tail == 0 && pos == fixed_size - 1)) {
                maxed = true;
            }
        }
        buffer[pos + 1] = end_item;

        if (maxed) {
            tail = (head + count + 1) % fixed_size;
        }
        head = (head + count) % fixed_size;
        maxed = (head == tail - 1);
    }

    void clear() {
        GUARD_LOCK;

        head = 0;
        tail = 0;
        buffer[tail] = end_item;
        maxed = false;
    }

    void clear_if_empty() {
        will_clear_if_empty = true;
        GUARD_LOCK;

        // ReSharper disable once CppDFAConstantConditions - may be cleared by write from interrupt
        if (empty() && will_clear_if_empty) {
            head = tail = 0;
            maxed = false;
        }
    }

    [[nodiscard]] bool empty() const {
        return head == tail;
    }

    [[nodiscard]] bool full() const {
        return maxed;
    }

    T consume() {
        GUARD_LOCK;

        if (empty()) {
            return T();
        }

        auto to_consume = buffer[tail];
        tail = (tail + 1) % fixed_size;
        maxed = false;

        return to_consume;
    }

    std::pair<const T *, uint16_t> consume_block() {
        GUARD_LOCK;

        uint16_t consumable = (head <= tail && !empty() ? fixed_size : head) - tail;
        T *to_consume = reinterpret_cast<T *>(&buffer[tail]);
        if (head <= tail && !empty()) {
            tail = 0;
        } else {
            tail = head;
        }
        maxed = false;
        return std::make_pair(to_consume, consumable);
    }

    uint16_t consume_block(T *block, uint16_t block_size) {
        GUARD_LOCK;

        uint16_t consumable = (head <= tail && !empty() ? fixed_size : head) - tail;
        uint16_t will_consume = std::min(block_size, consumable);
        T *to_consume = reinterpret_cast<T *>(&buffer[tail]);
        if (block_size < consumable) {
            tail = (tail + will_consume) % fixed_size;
        } else if (head <= tail && !empty()) {
            tail = 0;
        } else {
            tail = head;
        }
        maxed = false;
        memcpy(block, to_consume, will_consume);
        return will_consume;
    }

    uint16_t consume_line(T *line, uint16_t line_size) {
        // intentionally do nothing in the general case
        return 0;
    }

    void writeF(const T *format, ...) {
        // intentionally do nothing in the general case
    }

private:
    const T end_item = 0;
#ifdef ARDUINO_ARCH_RP2040
    //See above comment - we cannot have mutexes in freeRTOS as we could be within ourselves whilst an interrupt comes in
#else
    std::mutex mutex;
#endif
    std::unique_ptr<T[]> buffer;
    const uint16_t fixed_size;
    uint16_t head = 0;
    uint16_t tail = 0;
    bool maxed = false;
    bool will_clear_if_empty = false;
};

template<>
inline void CircularBuffer<char>::writeF(const char *format, ...) {
    static char printBuf[600];
    va_list arg;
    va_start(arg, format);
    size_t len = vsnprintf(printBuf, sizeof(printBuf), format, arg);
    va_end(arg);
    assert(len < sizeof(printBuf));
    write(printBuf);
}

template<>
inline uint16_t CircularBuffer<char>::consume_line(char *line, uint16_t line_size) {
    uint16_t consumable = (head <= tail && !empty() ? fixed_size : head) - tail;
    uint16_t will_consume = std::min(line_size, consumable);
    auto to_consume = reinterpret_cast<char *>(&buffer[tail]);
    auto line_end = strchr(to_consume, '\n');
    uint16_t written_out = 0;
    if (line_end == nullptr) {
        memcpy(line, to_consume, will_consume);
        written_out = will_consume;
        if (consumable == will_consume) {
            if (head <= tail && !empty()) {
                tail = 0;
                to_consume = reinterpret_cast<char *>(&buffer[0]);
                line_end = strchr(to_consume, '\n');
                if (line_end == nullptr) {
                    maxed = false;
                    line[written_out] = '\0';
                    return written_out;
                } else {
                    uint16_t extra = line_end - to_consume + 1;
                    extra = std::min(static_cast<uint16_t>(line_size - written_out), extra);
                    memcpy(&line[written_out], to_consume, extra);
                    maxed = false;
                    tail = (tail + extra) % fixed_size;
                    line[written_out + extra] = '\0';
                    return written_out + extra;
                }
            }
            tail = head;
            maxed = false;
            line[will_consume] = '\0';
            return will_consume;
        }
        tail += written_out;
        consumable = head;
        will_consume = std::min(static_cast<uint16_t>(line_size - written_out), consumable);
        to_consume = reinterpret_cast<char *>(&buffer[tail]);
        line_end = strchr(to_consume, '\n');
        if (line_end == nullptr) {
            memcpy(&line[written_out], to_consume, will_consume);
            maxed = false;
            if ((written_out + will_consume) < consumable) {
                tail = (tail + will_consume) % fixed_size;
            } else if (head <= tail && !empty()) {
                tail = written_out + will_consume;
            } else {
                tail = head;
            }
            tail += will_consume;
            line[written_out + will_consume] = '\0';
            return written_out + will_consume;
        }
    }
    uint16_t line_extra = std::min(will_consume, static_cast<uint16_t>(line_end - to_consume + 1));
    memcpy(&line[written_out], to_consume, line_extra);
    maxed = false;
    if ((written_out + line_extra) < consumable) {
        tail = (tail + line_extra) % fixed_size;
    } else if (head <= tail && !empty()) {
        tail = line_size;
    } else {
        tail = head;
    }
    line[written_out + line_extra] = '\0';
    return written_out + line_extra;
}
