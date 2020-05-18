// utils.cpp
#include <gtest/gtest.h>
#include "utils.h"

int uadd(int a, int b)
{
  return a+b;
}

TEST(UtilsTest, Uadd) {
  ASSERT_EQ(uadd(1,2), 3);
}

int main(int argc, char *argv[])
{  
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
