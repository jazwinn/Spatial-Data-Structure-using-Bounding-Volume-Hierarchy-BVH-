#include "common.hpp"

namespace testing {
    namespace internal {
        AssertionResult DoubleNearPredFormat(const char* expr1, const char* expr2, const char* absErrorExpr, glm::vec3 const& val1, glm::vec3 const& val2, double absError)
        {
            auto x_res = DoubleNearPredFormat(expr1, expr2, absErrorExpr, val1.x, val2.x, absError);
            auto y_res = DoubleNearPredFormat(expr1, expr2, absErrorExpr, val1.y, val2.y, absError);
            auto z_res = DoubleNearPredFormat(expr1, expr2, absErrorExpr, val1.z, val2.z, absError);

            if (!x_res || !y_res || !z_res) {
                return AssertionFailure()
                       << "The difference between " << expr1 << " and " << expr2
                       << " exceeds " << absErrorExpr << ", where\n"
                       << expr1 << " evaluates to " << val1 << ",\n"
                       << expr2 << " evaluates to " << val2 << ", and\n"
                       << absErrorExpr << " evaluates to " << absError << ".";
            }

            return AssertionSuccess();
        }

        AssertionResult DoubleNearPredFormat(const char* expr1, const char* expr2, const char* absErrorExpr, glm::vec2 const& val1, glm::vec2 const& val2, double absError)
        {
            auto x_res = DoubleNearPredFormat(expr1, expr2, absErrorExpr, val1.x, val2.x, absError);
            auto y_res = DoubleNearPredFormat(expr1, expr2, absErrorExpr, val1.y, val2.y, absError);

            if (!x_res || !y_res) {
                return AssertionFailure()
                       << "The difference between " << expr1 << " and " << expr2
                       << " exceeds " << absErrorExpr << ", where\n"
                       << expr1 << " evaluates to " << val1 << ",\n"
                       << expr2 << " evaluates to " << val2 << ", and\n"
                       << absErrorExpr << " evaluates to " << absError << ".";
            }

            return AssertionSuccess();
        }

        AssertionResult DoubleNearPredFormat(const char* expr1, const char* expr2, const char* absErrorExpr, CS350::Aabb const& val1, CS350::Aabb const& val2, double absError)
        {
            auto a_res = DoubleNearPredFormat(expr1, expr2, absErrorExpr, val1.min, val2.min, absError);
            auto b_res = DoubleNearPredFormat(expr1, expr2, absErrorExpr, val1.max, val2.max, absError);

            if (!a_res || !b_res) {
                return AssertionFailure()
                       << "The difference between " << expr1 << " and " << expr2
                       << " exceeds " << absErrorExpr << ", where\n"
                       << expr1 << " evaluates to [" << val1.min << "], [" << val1.max << "],\n"
                       << expr2 << " evaluates to [" << val2.min << "], [" << val2.max << "], and\n"
                       << absErrorExpr << " evaluates to " << absError << ".";
            }

            return AssertionSuccess();
        }

        AssertionResult DoubleNearPredFormat(const char* expr1, const char* expr2, const char* absErrorExpr, CS350::Triangle const& val1, CS350::Triangle const& val2, double absError)
        {
            auto a_res = DoubleNearPredFormat(expr1, expr2, absErrorExpr, val1[0], val2[0], absError);
            auto b_res = DoubleNearPredFormat(expr1, expr2, absErrorExpr, val1[1], val2[1], absError);
            auto c_res = DoubleNearPredFormat(expr1, expr2, absErrorExpr, val1[2], val2[2], absError);

            if (!a_res || !b_res || !c_res) {
                return AssertionFailure()
                       << "The difference between " << expr1 << " and " << expr2
                       << " exceeds " << absErrorExpr << ", where\n"
                       << expr1 << " evaluates to " << val1[0] << ", " << val1[1] << ", " << val1[2] << ",\n"
                       << expr2 << " evaluates to " << val2[0] << ", " << val2[1] << ", " << val2[2] << ", and\n"
                       << absErrorExpr << " evaluates to " << absError << ".";
            }

            return AssertionSuccess();
        }
    }

}
