#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace perception
{

namespace
{

struct Poly2Model
{
	bool valid{false};
	// x = a2*y^2 + a1*y + a0
	double a2{0.0};
	double a1{0.0};
	double a0{0.0};
	int n{0};
};

inline double clampd(const double v, const double lo, const double hi)
{
	return std::max(lo, std::min(v, hi));
}

inline double eval_x(const Poly2Model & p, const double y)
{
	return p.a2 * y * y + p.a1 * y + p.a0;
}

int argmax_range(const cv::Mat & hist_1xw, const int x0, const int x1)
{
	if (x1 <= x0) {
		return x0;
	}

	int best_x = x0;
	int best_v = std::numeric_limits<int>::min();
	for (int x = x0; x < x1; ++x) {
		const int v = hist_1xw.at<int>(0, x);
		if (v > best_v) {
			best_v = v;
			best_x = x;
		}
	}
	return best_x;
}

Poly2Model fit_poly_x_as_fn_of_y(const std::vector<cv::Point> & pts)
{
	Poly2Model out;
	out.n = static_cast<int>(pts.size());
	if (pts.size() < 80U) {
		return out;
	}

	cv::Mat A(static_cast<int>(pts.size()), 3, CV_64F);
	cv::Mat b(static_cast<int>(pts.size()), 1, CV_64F);

	for (int i = 0; i < static_cast<int>(pts.size()); ++i) {
		const double y = static_cast<double>(pts[i].y);
		A.at<double>(i, 0) = y * y;
		A.at<double>(i, 1) = y;
		A.at<double>(i, 2) = 1.0;
		b.at<double>(i, 0) = static_cast<double>(pts[i].x);
	}

	cv::Mat x;
	if (!cv::solve(A, b, x, cv::DECOMP_SVD)) {
		return out;
	}

	out.a2 = x.at<double>(0, 0);
	out.a1 = x.at<double>(1, 0);
	out.a0 = x.at<double>(2, 0);
	out.valid = std::isfinite(out.a2) && std::isfinite(out.a1) && std::isfinite(out.a0);
	return out;
}

std::vector<cv::Point> sliding_window_collect(
	const cv::Mat & binary,
	const std::vector<cv::Point> & nz,
	int base_x,
	const int margin,
	const int n_windows,
	const int minpix)
{
	std::vector<cv::Point> out;
	if (binary.empty() || base_x < 0) {
		return out;
	}

	const int h = binary.rows;
	const int w = binary.cols;
	const int win_h = std::max(1, h / n_windows);
	int x_current = clampd(static_cast<double>(base_x), 0.0, static_cast<double>(w - 1));

	// Bucket non-zero points by window (y-range) to avoid O(n_windows * nz.size()) scanning.
	std::vector<std::vector<cv::Point>> bins(static_cast<size_t>(n_windows));
	for (auto & b : bins) {
		b.reserve(nz.size() / std::max(1, n_windows));
	}
	for (const auto & p : nz) {
		const int win = (h - 1 - p.y) / win_h;  // 0: bottom window
		if (win < 0 || win >= n_windows) {
			continue;
		}
		bins[static_cast<size_t>(win)].push_back(p);
	}

	for (int win = 0; win < n_windows; ++win) {
		const auto & candidates = bins[static_cast<size_t>(win)];
		std::vector<cv::Point> in_win;
		in_win.reserve(256);
		for (const auto & p : candidates) {
			if (std::abs(p.x - x_current) <= margin) {
				in_win.push_back(p);
			}
		}

		if (!in_win.empty()) {
			out.insert(out.end(), in_win.begin(), in_win.end());
		}

		if (static_cast<int>(in_win.size()) > minpix) {
			double sx = 0.0;
			for (const auto & p : in_win) {
				sx += static_cast<double>(p.x);
			}
			x_current = static_cast<int>(std::lround(sx / static_cast<double>(in_win.size())));
			x_current = std::max(0, std::min(w - 1, x_current));
		}
	}

	return out;
}

}  // namespace

// Helper: Search for lane pixels around a predicted polynomial
std::vector<cv::Point> search_around_poly(
	const cv::Mat & edges,
	const std::vector<cv::Point> & all_nonzero,
	const Poly2Model & predicted_poly,
	const int margin,
	const int minpix)
{
	std::vector<cv::Point> result;
	
	if (!predicted_poly.valid || all_nonzero.empty()) {
		return result;
	}
	
	// For each y in image, predict x from polynomial and collect nearby points
	for (const auto & pt : all_nonzero) {
		const int y = pt.y;
		const double x_pred = eval_x(predicted_poly, static_cast<double>(y));
		const int x = pt.x;
		
		// Check if point is within margin of predicted x
		if (std::abs(static_cast<double>(x) - x_pred) < margin) {
			result.push_back(pt);
		}
	}
	
	return result;
}

bool detect_lane(
	const cv::Mat & bgr,
	double & lateral_error,
	double & heading_error,
	cv::Mat * debug_out)
{
	// Static variables for lane continuity
	static double prev_center_bottom = -1.0;
	static int continuity_count = 0;
	
	// Static variables for previous polynomial tracking
	static Poly2Model prev_left_poly;
	static Poly2Model prev_right_poly;
	static bool has_prev_polys = false;
	static int frame_count = 0;  // Track frame number for method selection
	
	lateral_error = 0.0;
	heading_error = 0.0;

	if (bgr.empty() || bgr.cols < 64 || bgr.rows < 64) {
		if (debug_out != nullptr) {
			*debug_out = bgr.clone();
		}
		return false;
	}

	const int w = bgr.cols;
	const int h = bgr.rows;

	// 1) Perspective Transform
	cv::Point2f src_pts[4] = {
		cv::Point2f(w * 0.40f, h * 0.65f),
		cv::Point2f(w * 0.60f, h * 0.65f),
		cv::Point2f(w * 0.95f, h * 1.0f),
		cv::Point2f(w * 0.05f, h * 1.0f)};
	cv::Point2f dst_pts[4] = {
		cv::Point2f(w * 0.20f, 0.0f),
		cv::Point2f(w * 0.80f, 0.0f),
		cv::Point2f(w * 0.80f, static_cast<float>(h)),
		cv::Point2f(w * 0.20f, static_cast<float>(h))};
	const cv::Mat M = cv::getPerspectiveTransform(src_pts, dst_pts);

	cv::Mat warped;
	cv::warpPerspective(bgr, warped, M, cv::Size(w, h), cv::INTER_LINEAR);

	// 2) Color Threshold (white + yellow)
	cv::Mat hsv;
	cv::cvtColor(warped, hsv, cv::COLOR_BGR2HSV);

	cv::Mat white_mask;
	cv::Mat yellow_mask;
	cv::inRange(hsv, cv::Scalar(0, 0, 170), cv::Scalar(180, 60, 255), white_mask);
	cv::inRange(hsv, cv::Scalar(15, 60, 80), cv::Scalar(40, 255, 255), yellow_mask);

	cv::Mat color_bin;
	cv::bitwise_or(white_mask, yellow_mask, color_bin);

	const cv::Mat k3 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	const cv::Mat k5 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
	cv::morphologyEx(color_bin, color_bin, cv::MORPH_CLOSE, k5, cv::Point(-1, -1), 1);
	cv::morphologyEx(color_bin, color_bin, cv::MORPH_OPEN, k3, cv::Point(-1, -1), 1);

	// 3) Canny Edge
	cv::Mat edges;
	cv::Canny(color_bin, edges, 50, 150);
	// Keep edges only where the color mask is active (reduces noise + CPU downstream).
	cv::bitwise_and(edges, color_bin, edges);

	// 4) Lane Detection - Frame 1 uses sliding window, Frame 2+ uses search around polynomial
	const int y_hist = static_cast<int>(0.5 * static_cast<double>(h));
	cv::Mat hist;
	cv::reduce(edges.rowRange(y_hist, h), hist, 0, cv::REDUCE_SUM, CV_32S);

	const int mid_x = w / 2;
	const int left_base = argmax_range(hist, 0, mid_x);
	const int right_base = argmax_range(hist, mid_x, w);

	std::vector<cv::Point> nz;
	cv::findNonZero(edges, nz);

	const int margin = std::max(35, static_cast<int>(0.08 * static_cast<double>(w)));
	const int n_windows = 9;
	const int minpix = 35;

	std::vector<cv::Point> left_pts, right_pts;

	// Frame 1 (frame_count == 0): Use sliding window to establish lanes
	if (frame_count == 0) {
		left_pts = sliding_window_collect(edges, nz, left_base, margin, n_windows, minpix);
		right_pts = sliding_window_collect(edges, nz, right_base, margin, n_windows, minpix);
	} else {
		// Frame 2+: Try search around polynomial first, fallback to sliding window
		if (has_prev_polys && prev_left_poly.valid && prev_right_poly.valid) {
			left_pts = search_around_poly(edges, nz, prev_left_poly, margin, minpix);
			right_pts = search_around_poly(edges, nz, prev_right_poly, margin, minpix);
			
			// If search around poly found too few points, fallback to sliding window
			if (left_pts.size() < static_cast<size_t>(minpix) || right_pts.size() < static_cast<size_t>(minpix)) {
				left_pts = sliding_window_collect(edges, nz, left_base, margin, n_windows, minpix);
				right_pts = sliding_window_collect(edges, nz, right_base, margin, n_windows, minpix);
			}
		} else {
			// No previous polys, use sliding window
			left_pts = sliding_window_collect(edges, nz, left_base, margin, n_windows, minpix);
			right_pts = sliding_window_collect(edges, nz, right_base, margin, n_windows, minpix);
		}
	}

	// 5) Polynomial Fit
	Poly2Model left_poly = fit_poly_x_as_fn_of_y(left_pts);
	Poly2Model right_poly = fit_poly_x_as_fn_of_y(right_pts);

	const double img_center_x = 0.5 * static_cast<double>(w);
	const double y_ref_bottom = static_cast<double>(h - 5);
	const double y_ref_top = static_cast<double>(std::max(0, h / 2));

	bool have_left = left_poly.valid;
	bool have_right = right_poly.valid;

	if (have_left && have_right) {
		const double xl_b = eval_x(left_poly, y_ref_bottom);
		const double xr_b = eval_x(right_poly, y_ref_bottom);
		const double xl_t = eval_x(left_poly, y_ref_top);
		const double xr_t = eval_x(right_poly, y_ref_top);
		const double wb = xr_b - xl_b;
		const double wt = xr_t - xl_t;

		// Relaxed width constraints for better curve handling
		const bool width_ok =
			(wb > 0.10 * w && wb < 0.95 * w) &&
			(wt > 0.10 * w && wt < 0.95 * w) &&
			(xl_b < xr_b) && (xl_t < xr_t);

		if (!width_ok) {
			// In curves, be more lenient - prefer larger polynomial fits
			if (left_poly.n > right_poly.n) {
				have_right = false;
			} else {
				have_left = false;
			}
		}
	}

	if (!have_left && !have_right) {
		if (debug_out != nullptr) {
			*debug_out = warped.clone();
			cv::putText(
				*debug_out, "Lane: not found", cv::Point(20, 40),
				cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
		}
		return false;
	}

	// 6) Lane Center
	const double nominal_half_lane_px = 0.3 * static_cast<double>(w);

	auto center_x_at_y = [&](const double y) {
		if (have_left && have_right) {
			return 0.5 * (eval_x(left_poly, y) + eval_x(right_poly, y));
		}
		if (have_left) {
			return eval_x(left_poly, y) + nominal_half_lane_px;
		}
		return eval_x(right_poly, y) - nominal_half_lane_px;
	};

	const double center_bottom = center_x_at_y(y_ref_bottom);
	const double center_top = center_x_at_y(y_ref_top);

	const double half_w = 0.5 * static_cast<double>(w);
	
	// Lane continuity check: reject detections that jump to adjacent lanes
	const double lane_jump_threshold = 0.3 * w;  // Max allowed jump between frames
	if (prev_center_bottom >= 0.0 && std::abs(center_bottom - prev_center_bottom) > lane_jump_threshold) {
		// Lane jumped too much - likely wrong detection due to camera rotation
		// Reset continuity counter
		continuity_count = 0;
		if (debug_out != nullptr) {
			*debug_out = bgr.clone();
			cv::putText(
				*debug_out, "Lane Jump Detected", cv::Point(20, 40),
				cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
		}
		return false;
	}
	
	// Update previous center for next frame
	prev_center_bottom = center_bottom;
	continuity_count++;
	
	lateral_error = clampd((center_bottom - img_center_x) / std::max(half_w, 1.0), -1.0, 1.0);

	const double dy_center = y_ref_bottom - y_ref_top;
	const double dx_center = center_top - center_bottom;
	const double angle_rad = std::atan2(dx_center, std::max(dy_center, 1.0));
	constexpr double kMaxHeadingRad = 0.4363323129985824;  // 25 deg
	heading_error = clampd(-angle_rad / kMaxHeadingRad, -1.0, 1.0);  // NEGATED: correct the sign convention

	// NEW: Calculate polynomial curvature for better curve handling
	// For center lane polynomial, estimate curvature at bottom point
	// This helps vehicle anticipate and lean into curves
	if (have_left && have_right) {
		// Average the curvatures of left and right polynomials
		const double k_left = left_poly.a2;   // d²x/dy² = 2*a2 for quadratic
		const double k_right = right_poly.a2;
		const double curvature = (k_left + k_right) * 0.5;
		// Minimal curvature multiplier - don't anticipate curves, follow them when needed
		heading_error += clampd(curvature * 0.1, -0.30, 0.30);
	}

	if (debug_out != nullptr) {
		// Draw overlays in warped (bird's-eye) space, then project them back onto the original camera view.
		cv::Mat overlay_warp = cv::Mat::zeros(warped.size(), warped.type());

		auto draw_poly_on = [&](const Poly2Model & poly, const bool valid, const cv::Scalar & color) {
			if (!valid) {
				return;
			}
			std::vector<cv::Point> pts;
			for (int y = static_cast<int>(std::lround(y_ref_top)); y <= static_cast<int>(std::lround(y_ref_bottom)); y += 8) {
				const int x = static_cast<int>(std::lround(eval_x(poly, static_cast<double>(y))));
				if (x >= 0 && x < w && y >= 0 && y < h) {
					pts.emplace_back(x, y);
				}
			}
			if (pts.size() >= 2U) {
				cv::polylines(overlay_warp, pts, false, color, 3);
			}
		};

		draw_poly_on(left_poly, have_left, cv::Scalar(255, 0, 0));
		draw_poly_on(right_poly, have_right, cv::Scalar(0, 255, 0));

		for (const auto & p : left_pts) {
			overlay_warp.at<cv::Vec3b>(p.y, p.x) = cv::Vec3b(255, 0, 0);
		}
		for (const auto & p : right_pts) {
			overlay_warp.at<cv::Vec3b>(p.y, p.x) = cv::Vec3b(0, 255, 0);
		}

		const cv::Point p_bottom(
			static_cast<int>(std::lround(center_bottom)),
			static_cast<int>(std::lround(y_ref_bottom)));
		const cv::Point p_top(
			static_cast<int>(std::lround(center_top)),
			static_cast<int>(std::lround(y_ref_top)));
		cv::line(overlay_warp, p_bottom, p_top, cv::Scalar(0, 255, 255), 3);

		const cv::Point img_center_pt(
			static_cast<int>(std::lround(img_center_x)),
			static_cast<int>(std::lround(y_ref_bottom)));
		cv::circle(overlay_warp, img_center_pt, 5, cv::Scalar(0, 0, 255), -1);
		cv::circle(overlay_warp, p_bottom, 5, cv::Scalar(0, 255, 255), -1);

		char text1[128];
		char text2[128];
		std::snprintf(text1, sizeof(text1), "lateral=%.3f", lateral_error);
		std::snprintf(text2, sizeof(text2), "heading=%.3f", heading_error);
		// Project overlay back to original camera view.
		const cv::Mat Minv = cv::getPerspectiveTransform(dst_pts, src_pts);
		cv::Mat overlay_orig;
		cv::warpPerspective(overlay_warp, overlay_orig, Minv, cv::Size(w, h), cv::INTER_LINEAR);

		cv::Mat out = bgr.clone();
		// Alpha blend: only where overlay has non-zero pixels.
		cv::Mat overlay_gray;
		cv::cvtColor(overlay_orig, overlay_gray, cv::COLOR_BGR2GRAY);
		cv::Mat mask;
		cv::threshold(overlay_gray, mask, 1, 255, cv::THRESH_BINARY);
		cv::Mat out_bg;
		out.copyTo(out_bg);
		overlay_orig.copyTo(out_bg, mask);
		out = out_bg;

		cv::putText(
			out, text1, cv::Point(20, 35),
			cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
		cv::putText(
			out, text2, cv::Point(20, 65),
			cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);

		*debug_out = out;
	}

	// Save polynomials for next frame (search around polynomial)
	prev_left_poly = left_poly;
	prev_right_poly = right_poly;
	has_prev_polys = true;
	frame_count++;  // Increment frame counter for next frame

	return true;
}

}  // namespace perception
