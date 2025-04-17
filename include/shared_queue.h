#ifndef SHARED_QUEUE_H
#define SHARED_QUEUE_H

#pragma once
#include <type_traits>
#include <boost/lockfree/spsc_queue.hpp>
#include <optional>

template<typename T, std::size_t Capacity>
class SharedQueue {
  static_assert(std::is_trivially_copyable_v<T>,
                "SharedQueue<T> requires T to be trivially copyable");
public:
  // Producer: returns false if queue is full
  bool write(const T& data) {
    return queue_.push(data);
  }

  // Consumer: returns std::nullopt if queue is empty
  std::optional<T> read() {
    T temp{};
    if (queue_.pop(temp)) {
      return temp;
    }
    return std::nullopt;
  }

  // Optional: check if data is available
  bool empty() const {
    return queue_.empty();
  }

private:
  // Fixed-size, lock‑free SPSC buffer
  boost::lockfree::spsc_queue<T, boost::lockfree::capacity<Capacity>> queue_;
};

#endif // SHARED_QUEUE_H
