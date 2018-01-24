#include <gtest/gtest.h>

/*
TEST(Communication, CommunicationHelper)
{
    // Google tests provide no handlers for exceptions
    try
    {
        tfr_communication::CommunicationHelper helper;
        EXPECT_STREQ("Communication System Online", helper.GetEcho().c_str());
    }
    catch (std::exception e)
    {
        // An unhandled exception was thrown; fail the test
        ASSERT_TRUE(false);
    }
}
*/

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}