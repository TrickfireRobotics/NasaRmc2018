#include <gtest/gtest.h>
#include "dumping_helper.h"

TEST(Dumping, DumpingHelper)
{
    // Google tests provide no handlers for exceptions
    try
    {
        tfr_dumping::DumpingHelper helper;
        EXPECT_STREQ("Dumping System Online", helper.GetEcho().c_str());
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