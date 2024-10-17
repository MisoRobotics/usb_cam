#include"../nodes/usb_cam_node.cpp"

#include <gtest/gtest.h>

TEST(UsbCamNode, TestSerialNumber)
{
  std::string config_sn = "SN0001";
  std::string found_sn = "IRLED_SN00015";
  std::size_t found_pos = config_sn.find(found_sn);
  bool equal_sn = (found_pos != std::string::npos);
  EXPECT_TRUE(equal_sn);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
