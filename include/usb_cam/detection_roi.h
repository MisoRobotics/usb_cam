#ifndef USB_CAM_DETECTION_ROI_H
#define USB_CAM_DETECTION_ROI_H

#include <algorithm>
#include <utility>
#include <vector>

namespace usb_cam {
/** Rectangle of the image that aruco_detect searches for markers. */
struct DetectionRoi {
  int x = 0;
  int y = 0;
  int width = 0;
  int height = 0;
};

namespace detail {
inline void mergeInclusiveIntervals(std::vector<std::pair<int, int>> &spans) {
  if (spans.empty())
    return;
  std::sort(spans.begin(), spans.end());
  std::vector<std::pair<int, int>> merged;
  merged.push_back(spans[0]);
  for (size_t i = 1; i < spans.size(); ++i) {
    auto cur = spans[i];
    auto &last = merged.back();
    if (cur.first <= last.second + 1)
      last.second = std::max(last.second, cur.second);
    else
      merged.push_back(cur);
  }
  spans.swap(merged);
}

/**
 * Widest gap between the (inclusive) blocked spans along an axis of the given
 * length, expanded by margin_px and clamped to the axis. Without any span the
 * whole axis is free.
 *
 * @return false if the spans leave no gap.
 */
inline bool widestFreeInterval(std::vector<std::pair<int, int>> spans,
                               int length, int margin_px, int &lo_out,
                               int &hi_out) {
  if (spans.empty()) {
    lo_out = 0;
    hi_out = length - 1;
    return true;
  }
  mergeInclusiveIntervals(spans);

  int best_lo = 0;
  int best_hi = -1;
  int best_w = -1;
  int prev_end = -1;
  for (const auto &seg : spans) {
    const int gap_lo = prev_end + 1;
    const int gap_hi = seg.first - 1;
    const int gw = gap_hi - gap_lo + 1;
    if (gw > best_w) {
      best_w = gw;
      best_lo = gap_lo;
      best_hi = gap_hi;
    }
    prev_end = std::max(prev_end, seg.second);
  }
  {
    const int gap_lo = prev_end + 1;
    const int gap_hi = length - 1;
    const int gw = gap_hi - gap_lo + 1;
    if (gw > best_w) {
      best_w = gw;
      best_lo = gap_lo;
      best_hi = gap_hi;
    }
  }
  if (best_w <= 0 || best_hi < best_lo)
    return false;

  const int margin = std::max(0, margin_px);
  lo_out = std::max(0, best_lo - margin);
  hi_out = std::min(length - 1, best_hi + margin);
  return hi_out >= lo_out;
}
} // namespace detail

/**
 * Same computation as deriveDetectionRoiFromBlacklist() in aruco_detect
 * (fiducials/aruco_detect/src/aruco_detect.cpp), so the rectangle published by
 * usb_cam matches the one aruco_detect runs detectMarkers on.
 *
 * Blacklist polygons are either horizontal bands (they span the full image
 * width, e.g. the top and bottom of the frame) or vertical strips (left/right
 * x-bands). The y-intervals of the horizontal bands and the x-intervals of all
 * other polygons are each merged; the widest gap on each axis, expanded by
 * margin_px, is the ROI. An axis without blocked intervals is kept whole.
 *
 * @param region_points x and y coordinates of each blacklist polygon's points
 * (one vector per polygon).
 * @param image_width   Image width in pixels.
 * @param image_height  Image height in pixels.
 * @param margin_px     detection_roi_margin_px.
 * @param out           Resulting rectangle.
 * @return false if no rectangle could be derived (aruco_detect then uses the
 * full frame).
 */
inline bool deriveDetectionRoiFromBlacklist(
    const std::vector<std::vector<std::pair<int, int>>> &region_points,
    int image_width, int image_height, int margin_px, DetectionRoi &out) {
  if (image_width <= 0 || image_height <= 0 || region_points.empty())
    return false;

  std::vector<std::pair<int, int>> x_spans;
  std::vector<std::pair<int, int>> y_spans;
  bool have_polygon = false;

  for (const auto &region : region_points) {
    if (region.empty())
      continue;
    int xmin = region[0].first;
    int xmax = region[0].first;
    int ymin = region[0].second;
    int ymax = region[0].second;
    for (const auto &p : region) {
      xmin = std::min(xmin, p.first);
      xmax = std::max(xmax, p.first);
      ymin = std::min(ymin, p.second);
      ymax = std::max(ymax, p.second);
    }
    const int xa = std::max(0, std::min(image_width - 1, xmin));
    const int xb = std::max(0, std::min(image_width - 1, xmax));
    const int ya = std::max(0, std::min(image_height - 1, ymin));
    const int yb = std::max(0, std::min(image_height - 1, ymax));
    if (xa > xb || ya > yb)
      continue;
    have_polygon = true;
    if (xa == 0 && xb == image_width - 1)
      y_spans.emplace_back(ya, yb);
    else
      x_spans.emplace_back(xa, xb);
  }

  if (!have_polygon)
    return false;

  int x_lo, x_hi, y_lo, y_hi;
  if (!detail::widestFreeInterval(x_spans, image_width, margin_px, x_lo,
                                  x_hi) ||
      !detail::widestFreeInterval(y_spans, image_height, margin_px, y_lo,
                                  y_hi))
    return false;

  out.x = x_lo;
  out.y = y_lo;
  out.width = x_hi - x_lo + 1;
  out.height = y_hi - y_lo + 1;
  return true;
}

} // namespace usb_cam

#endif // USB_CAM_DETECTION_ROI_H
