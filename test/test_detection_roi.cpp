#include <gtest/gtest.h>

#include <usb_cam/detection_roi.h>

using usb_cam::DetectionRoi;
using usb_cam::deriveDetectionRoiFromBlacklist;
using Polygon = std::vector<std::pair<int, int>>;

namespace {
Polygon rect(int x0, int y0, int x1, int y1) {
  return {{x0, y0}, {x0, y1}, {x1, y1}, {x1, y0}};
}
} // namespace

TEST(DetectionRoi, HorizontalBandsCropRows) {
  // fryer_cam_1: bands at the bottom and the top, full width.
  const std::vector<Polygon> regions = {rect(0, 969, 1920, 1080),
                                        rect(0, 0, 1920, 187)};
  DetectionRoi roi;
  ASSERT_TRUE(deriveDetectionRoiFromBlacklist(regions, 1920, 1080, 100, roi));
  EXPECT_EQ(0, roi.x);
  EXPECT_EQ(1920, roi.width);
  EXPECT_EQ(88, roi.y);
  EXPECT_EQ(981, roi.height);
}

TEST(DetectionRoi, HorizontalBandsMarginIsClampedToImage) {
  // fryer_cam_3: the bottom margin runs past the last row.
  const std::vector<Polygon> regions = {rect(0, 0, 1920, 280),
                                        rect(0, 1020, 1920, 1080)};
  DetectionRoi roi;
  ASSERT_TRUE(deriveDetectionRoiFromBlacklist(regions, 1920, 1080, 100, roi));
  EXPECT_EQ(181, roi.y);
  EXPECT_EQ(1079 - 181 + 1, roi.height);
  EXPECT_EQ(0, roi.x);
  EXPECT_EQ(1920, roi.width);
}

TEST(DetectionRoi, VerticalStripsCropColumns) {
  const std::vector<Polygon> regions = {rect(0, 0, 400, 1079),
                                        rect(1500, 0, 1919, 1079)};
  DetectionRoi roi;
  ASSERT_TRUE(deriveDetectionRoiFromBlacklist(regions, 1920, 1080, 100, roi));
  EXPECT_EQ(301, roi.x);
  EXPECT_EQ(1599 - 301 + 1, roi.width);
  EXPECT_EQ(0, roi.y);
  EXPECT_EQ(1080, roi.height);
}

TEST(DetectionRoi, BandsAndStripsCropBothAxes) {
  const std::vector<Polygon> regions = {rect(0, 0, 1920, 200),
                                        rect(0, 0, 300, 1079),
                                        rect(1600, 0, 1919, 1079)};
  DetectionRoi roi;
  ASSERT_TRUE(deriveDetectionRoiFromBlacklist(regions, 1920, 1080, 0, roi));
  EXPECT_EQ(301, roi.x);
  EXPECT_EQ(1599 - 301 + 1, roi.width);
  // The full-width band is a y span, so the strips don't restrict rows.
  EXPECT_EQ(201, roi.y);
  EXPECT_EQ(1080 - 201, roi.height);
}

TEST(DetectionRoi, PlaceholderPolygonsKeepTheFullFrame) {
  // fryer_cam_4 ships all-zero polygons until the blacklist is configured.
  const std::vector<Polygon> regions = {rect(0, 0, 0, 0), rect(0, 0, 0, 0)};
  DetectionRoi roi;
  ASSERT_TRUE(deriveDetectionRoiFromBlacklist(regions, 1920, 1080, 100, roi));
  EXPECT_EQ(0, roi.x);
  EXPECT_EQ(1920, roi.width);
  EXPECT_EQ(0, roi.y);
  EXPECT_EQ(1080, roi.height);
}

TEST(DetectionRoi, FullyBlockedFails) {
  const std::vector<Polygon> regions = {rect(0, 0, 1920, 1080)};
  DetectionRoi roi;
  EXPECT_FALSE(deriveDetectionRoiFromBlacklist(regions, 1920, 1080, 100, roi));
}

TEST(DetectionRoi, NoRegionsFails) {
  DetectionRoi roi;
  EXPECT_FALSE(deriveDetectionRoiFromBlacklist({}, 1920, 1080, 100, roi));
  EXPECT_FALSE(deriveDetectionRoiFromBlacklist({Polygon()}, 1920, 1080, 100,
                                               roi));
  EXPECT_FALSE(deriveDetectionRoiFromBlacklist({rect(0, 0, 10, 10)}, 0, 1080,
                                               100, roi));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
