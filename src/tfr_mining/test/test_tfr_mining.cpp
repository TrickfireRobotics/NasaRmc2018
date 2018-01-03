#include <gtest/gtest.h>
#include "mining_helper.h"

TEST(Mining, MiningHelper)
{
    // Google tests provide no handlers for exceptions
    try
    {
        tfr_mining::MiningHelper helper;
        EXPECT_STREQ("Mining System Online", helper.GetEcho().c_str());
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