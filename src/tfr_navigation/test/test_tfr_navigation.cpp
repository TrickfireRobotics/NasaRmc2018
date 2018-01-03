#include <gtest/gtest.h>
#include "navigation_helper.h"

TEST(Navigation, NavHelper)
{
    // Google tests provide no handlers for exceptions
    try
    {
        tfr_navigation::NavHelper helper;
        EXPECT_STREQ("Navigation System Online", helper.GetEcho().c_str());
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