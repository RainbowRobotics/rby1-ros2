#pragma once

// Lock-free triple buffer for one writer thread and one reader thread.
//
// The writer fills its back buffer and publishes it; the reader takes the most recent published
// buffer without blocking. Neither side waits for the other, and with buffers of a fixed size
// (e.g. vectors sized once) neither side allocates.

#include <array>
#include <atomic>
#include <cstdint>
#include <utility>

namespace rby1_hardware_interface
{

template <typename T>
class TripleBuffer
{
public:
  TripleBuffer() = default;
  explicit TripleBuffer(const T & initial) {reset(initial);}

  // Not thread safe: call while neither side runs.
  void reset(const T & initial)
  {
    for (auto & b : buffers_) {
      b = initial;
    }
    back_ = 0;
    middle_.store(1, std::memory_order_relaxed);
    front_ = 2;
    published_.store(false, std::memory_order_release);
  }

  // Writer: the buffer to fill. Contents are those of an older sample; overwrite all fields.
  T & back() {return buffers_[back_];}

  // Writer: make back() the newest sample.
  void publish()
  {
    const uint8_t previous = middle_.exchange(static_cast<uint8_t>(back_ | kFresh), std::memory_order_acq_rel);
    back_ = previous & kIndex;
    published_.store(true, std::memory_order_release);
  }

  // Reader: whether anything was published since reset().
  bool has_data() const {return published_.load(std::memory_order_acquire);}

  // Reader: the newest published sample (unchanged if nothing new was published).
  const T & read()
  {
    if (middle_.load(std::memory_order_acquire) & kFresh) {
      const uint8_t previous = middle_.exchange(front_, std::memory_order_acq_rel);
      front_ = previous & kIndex;
    }
    return buffers_[front_];
  }

private:
  static constexpr uint8_t kIndex = 0x3;
  static constexpr uint8_t kFresh = 0x4;

  std::array<T, 3> buffers_{};
  uint8_t back_{0};                 // writer only
  std::atomic<uint8_t> middle_{1};  // index | kFresh
  uint8_t front_{2};                // reader only
  std::atomic<bool> published_{false};
};

}  // namespace rby1_hardware_interface
