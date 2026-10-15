#include <gtest/gtest.h>

#include <atomic>
#include <thread>
#include <vector>

#include "rby1_hardware_interface/triple_buffer.hpp"

namespace rby1_hardware_interface
{
namespace
{

struct Sample
{
  uint64_t sequence{0};
  std::vector<uint64_t> payload;  // every element equals sequence when the sample is consistent
};

TEST(TripleBuffer, ReturnsTheNewestPublishedSample)
{
  TripleBuffer<int> buffer(0);
  EXPECT_FALSE(buffer.has_data());
  buffer.back() = 1;
  buffer.publish();
  buffer.back() = 2;
  buffer.publish();
  EXPECT_TRUE(buffer.has_data());
  EXPECT_EQ(buffer.read(), 2);
  EXPECT_EQ(buffer.read(), 2);  // unchanged without a new publish
  buffer.back() = 3;
  buffer.publish();
  EXPECT_EQ(buffer.read(), 3);
}

TEST(TripleBuffer, ConcurrentReaderNeverSeesATornSample)
{
  Sample initial;
  initial.payload.assign(64, 0);
  TripleBuffer<Sample> buffer(initial);
  std::atomic<bool> stop{false};
  std::thread writer([&]() {
      for (uint64_t k = 1; !stop; ++k) {
        auto & b = buffer.back();
        b.sequence = k;
        for (auto & v : b.payload) {
          v = k;
        }
        buffer.publish();
      }
    });
  uint64_t last = 0;
  for (int i = 0; i < 200000; ++i) {
    const auto & s = buffer.read();
    for (const auto v : s.payload) {
      ASSERT_EQ(v, s.sequence);
    }
    ASSERT_GE(s.sequence, last);  // never goes back in time
    last = s.sequence;
  }
  stop = true;
  writer.join();
  EXPECT_GT(last, 0u);
}

}  // namespace
}  // namespace rby1_hardware_interface
