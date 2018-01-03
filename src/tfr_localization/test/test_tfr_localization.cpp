#include <gtest/gtest.h>
#include "localization_helper.h"

TEST(Localization, LocalizationHelper)
{
    // Google tests provide no handlers for exceptions
    try
    {
        tfr_localization::LocalizationHelper helper;
        EXPECT_STREQ("Localization System Online", helper.GetEcho().c_str());
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