#include <gtest/gtest.h>
#include "placeholder_utility.h"

TEST(Utilities, Foo)
{
    // Google tests provide no handlers for exceptions
    try
    {
        int a = tfr_utilities::Foo();
        EXPECT_EQ(a, 5);
    }
    catch (std::exception e)
    {
        // An unhandled exception was thrown; fail the test
        ASSERT_TRUE(false);
    }
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}