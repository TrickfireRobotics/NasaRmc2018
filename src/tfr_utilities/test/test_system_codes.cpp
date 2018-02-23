#include <gtest/gtest.h>
#include "status_code.h"

TEST(SystemCodes, Basic)
{

	std::string message = getStatusMessage(StatusCode::NAV_OK, 0);
	ASSERT_EQ(message, "Navigation System OK");

}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
