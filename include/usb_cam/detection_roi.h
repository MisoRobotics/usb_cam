#ifndef USB_CAM_DETECTION_ROI_H
#define USB_CAM_DETECTION_ROI_H

#include <algorithm>
#include <utility>
#include <vector>

namespace usb_cam
{
/** Horizontal band of the image (full height) that aruco_detect searches for markers. */
struct DetectionRoi
{
  int x = 0;
  int width = 0;
};

namespace detail
{
inline void mergeInclusiveIntervals(std::vector<std::pair<int, int>>& spans)
{
  if (spans.empty())
    return;
  std::sort(spans.begin(), spans.end());
  std::vector<std::pair<int, int>> merged;
  merged.push_back(spans[0]);
  for (size_t i = 1; i < spans.size(); ++i)
  {
    auto cur = spans[i];
    auto& last = merged.back();
    if (cur.first <= last.second + 1)
      last.second = std::max(last.second, cur.second);
    else
      merged.push_back(cur);
  }
  spans.swap(merged);
}
}  // namespace detail

/**
 * Same computation as deriveDetectionRoiFromBlacklist() in aruco_detect (fiducials/aruco_detect/src/aruco_detect.cpp),
 * so the band published by usb_cam matches the band aruco_detect runs detectMarkers on.
 *
 * Blacklist polygons occupy left/right x-bands (image-pixel coords). Merge their x-intervals, pick the widest gap,
 * then expand it by margin_px.
 *
 * @param region_xs     x coordinates of each blacklist polygon's points (one vector per polygon).
 * @param image_width   Image width in pixels.
 * @param image_height  Image height in pixels.
 * @param margin_px     detection_roi_margin_px.
 * @param out           Resulting band (full image height).
 * @return false if no band could be derived (aruco_detect then uses the full frame).
 */
inline bool deriveDetectionRoiFromBlacklist(const std::vector<std::vector<int>>& region_xs, int image_width,
                                            int image_height, int margin_px, DetectionRoi& out)
{
  if (image_width <= 0 || image_height <= 0 || region_xs.empty())
    return false;

  std::vector<std::pair<int, int>> spans;
  spans.reserve(region_xs.size());

  for (const auto& region : region_xs)
  {
    if (region.empty())
      continue;
    int xmin = region[0];
    int xmax = region[0];
    for (int x : region)
    {
      xmin = std::min(xmin, x);
      xmax = std::max(xmax, x);
    }
    const int xa = std::max(0, std::min(image_width - 1, xmin));
    const int xb = std::max(0, std::min(image_width - 1, xmax));
    if (xa > xb)
      continue;
    spans.emplace_back(xa, xb);
  }

  if (spans.empty())
    return false;

  detail::mergeInclusiveIntervals(spans);

  int best_lo = 0;
  int best_hi = -1;
  int best_w = -1;
  int prev_end = -1;

  for (const auto& seg : spans)
  {
    const int gap_lo = prev_end + 1;
    const int gap_hi = seg.first - 1;
    const int gw = gap_hi - gap_lo + 1;
    if (gw > best_w)
    {
      best_w = gw;
      best_lo = gap_lo;
      best_hi = gap_hi;
    }
    prev_end = std::max(prev_end, seg.second);
  }
  {
    const int gap_lo = prev_end + 1;
    const int gap_hi = image_width - 1;
    const int gw = gap_hi - gap_lo + 1;
    if (gw > best_w)
    {
      best_w = gw;
      best_lo = gap_lo;
      best_hi = gap_hi;
    }
  }

  if (best_w <= 0 || best_hi < best_lo)
    return false;

  const int margin = std::max(0, margin_px);
  const int lo = std::max(0, best_lo - margin);
  const int hi = std::min(image_width - 1, best_hi + margin);
  const int w = hi - lo + 1;
  if (w <= 0)
    return false;

  out.x = lo;
  out.width = w;
  return true;
}

}  // namespace usb_cam

#endif  // USB_CAM_DETECTION_ROI_H
