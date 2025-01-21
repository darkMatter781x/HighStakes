#include <array>
#include <cstddef>
#include <memory>
#ifdef GTEST

#include <vector>
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "pf/resamplers/systematic.h"

using ::testing::MockFunction;

template <auto _t> struct WrapValue {
    static constexpr auto t = _t;
};

template <typename T>
concept Wraps =
    std::is_same_v<decltype(std::declval<WrapValue<4u>>().t), size_t>;

template <class __N> class SystematicResamplerTest : public testing::Test {
  protected:
    static constexpr size_t _N = __N::t;

    SystematicResamplerTest() {
      // q0_ remains empty
    }

    // ~QueueTest() override = default;

    MockFunction<float()> randomMock;
    resamplers::Systematic<_N> resampler {randomMock.AsStdFunction()};
    std::array<float, _N> weights {};
    std::unique_ptr<std::array<size_t, _N>> indices =
        std::make_unique<std::array<size_t, _N>>();
};

typedef ::testing::Types<WrapValue<4u>> Implementations;
TYPED_TEST_SUITE(SystematicResamplerTest, Implementations);

TYPED_TEST(SystematicResamplerTest, sampleEach) {
  constexpr size_t N = TestFixture::_N;
  std::fill(this->weights.begin(), this->weights.end(), 1.f / N);

  EXPECT_CALL(this->randomMock, Call()).WillOnce(testing::Return(0.f));
  this->indices =
      this->resampler.resample(this->weights, std::move(this->indices));

  for (size_t i = 0; i < N; ++i) EXPECT_EQ(this->indices->at(i), i);
}

TYPED_TEST(SystematicResamplerTest, sampleLastOnly) {
  constexpr size_t N = TestFixture::_N;
  std::fill(this->weights.begin(), this->weights.end(), 0.f);
  this->weights.back() = 1.f;

  EXPECT_CALL(this->randomMock, Call()).WillOnce(testing::Return(0.f));
  this->indices =
      this->resampler.resample(this->weights, std::move(this->indices));

  for (size_t i = 0; i < N; ++i) EXPECT_EQ(this->indices->at(i), N - 1);
}

TYPED_TEST(SystematicResamplerTest, sampleLastOnlyWithOffset) {
  constexpr size_t N = TestFixture::_N;
  std::fill(this->weights.begin(), this->weights.end(), 0.f);
  this->weights.back() = 1.f;

  EXPECT_CALL(this->randomMock, Call()).WillOnce(testing::Return(1.f));
  this->indices =
      this->resampler.resample(this->weights, std::move(this->indices));

  for (size_t i = 0; i < N; ++i)
    EXPECT_EQ(this->indices->at(i), N - 1) << "failed at index: " << i;
}

#endif