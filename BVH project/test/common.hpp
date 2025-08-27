#ifndef __COMMON_HPP__
#define __COMMON_HPP__

#include "shapes.hpp"
#include "logging.hpp"
#include <gtest/gtest.h>

inline char const* TestName() { return ::testing::UnitTest::GetInstance()->current_test_info()->name(); }
inline char const* TestSuiteName() { return ::testing::UnitTest::GetInstance()->current_test_info()->test_suite_name(); }

namespace testing {
    namespace internal {
        AssertionResult DoubleNearPredFormat(const char* expr1, const char* expr2, const char* absErrorExpr, glm::vec2 const& val1, glm::vec2 const& val2, double absError);
        AssertionResult DoubleNearPredFormat(const char* expr1, const char* expr2, const char* absErrorExpr, glm::vec3 const& val1, glm::vec3 const& val2, double absError);
        AssertionResult DoubleNearPredFormat(const char* expr1, const char* expr2, const char* absErrorExpr, CS350::Aabb const& val1, CS350::Aabb const& val2, double absError);
        AssertionResult DoubleNearPredFormat(const char* expr1, const char* expr2, const char* absErrorExpr, CS350::Triangle const& val1, CS350::Triangle const& val2, double absError);
    }
}

#endif // __COMMON_HPP__
