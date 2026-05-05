#pragma once

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

#include "rosuite/core/core.hpp"

namespace rosuite::filters::test_support
{

    struct TestContext
    {
        int failures = 0;

        /// Asserts that a condition is true.
        void expect_true(const bool condition, const std::string &message)
        {
            if (!condition)
            {
                std::cerr << "FAIL: " << message << '\n';
                ++failures;
            }
        }

        /// Asserts that two scalar values are close.
        void expect_near(
            const core::Scalar actual,
            const core::Scalar expected,
            const core::Scalar tolerance,
            const std::string &message)
        {
            expect_true(std::abs(actual - expected) <= tolerance, message);
        }
    };

    /// Completes the test executable with a summary message.
    inline int finish(TestContext &context, const char *success_message)
    {
        if (context.failures != 0)
        {
            std::cerr << context.failures << " test(s) failed.\n";
            return EXIT_FAILURE;
        }

        std::cout << success_message << '\n';
        return EXIT_SUCCESS;
    }

} // namespace rosuite::filters::test_support
