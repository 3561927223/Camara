#include "ds_planner.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ds
{
	static inline double clamp(double x, double lo, double hi)
	{
		return std::max(lo, std::min(x, hi));
	}

	// Integrate constant jerk for time dt from initial state (a0, v0, s0)
	static inline void integrateJerk(double j, double dt, double a0, double v0, double s0,
			double &a1, double &v1, double &s1)
	{
		a1 = a0 + j * dt;
		v1 = v0 + a0 * dt + 0.5 * j * dt * dt;
		s1 = s0 + v0 * dt + 0.5 * a0 * dt * dt + (1.0 / 6.0) * j * dt * dt * dt;
	}

	// Distance required to change speed from v_start -> v_end with jerk/acc limits
	// accelerating=true means v_end >= v_start, false means v_end <= v_start
	static double distanceForSpeedChange(double v_start, double v_end, bool accelerating,
			double a_max, double j_max)
	{
		double dv = accelerating ? (v_end - v_start) : (v_start - v_end);
		if (dv < 0) return std::numeric_limits<double>::quiet_NaN();
		double t_j = a_max / j_max;
		// Triangular accel: cannot reach a_max
		double t_tri = std::sqrt(std::max(0.0, dv / j_max));
		if (t_tri <= t_j)
		{
			double a = 0.0, v = v_start, x = 0.0;
			double a1, v1, x1; integrateJerk(accelerating ? +j_max : -j_max, t_tri, a, v, x, a1, v1, x1);
			double a2, v2, x2; integrateJerk(accelerating ? -j_max : +j_max, t_tri, a1, v1, x1, a2, v2, x2);
			(void)a2; (void)v2;
			return x2 - x;
		}
		// Trapezoidal accel: reach a_max with plateau in acceleration
		double t2_local = (dv - a_max * t_j) / a_max;
		if (t2_local < 0) t2_local = 0;
		double a = 0.0, v = v_start, x = 0.0;
		double a1, v1, x1; integrateJerk(accelerating ? +j_max : -j_max, t_j, a, v, x, a1, v1, x1);
		double a2, v2, x2; integrateJerk(0.0,                    t2_local, a1, v1, x1, a2, v2, x2);
		double a3, v3, x3; integrateJerk(accelerating ? -j_max : +j_max, t_j, a2, v2, x2, a3, v3, x3);
		(void)a3; (void)v3;
		return x3 - x;
	}

	// Compute maximum reachable end speed given start speed and distance (forward pass)
	static double reachableEndSpeedForward(double v_start, double distance,
			double v_max, double a_max, double j_max)
	{
		v_start = clamp(v_start, 0.0, v_max);
		if (distance <= 0) return v_start;
		double lo = v_start;
		double hi = v_max;
		for (int i = 0; i < 70; ++i)
		{
			double mid = 0.5 * (lo + hi);
			double s_need = distanceForSpeedChange(v_start, mid, true, a_max, j_max);
			if (std::isnan(s_need) || s_need > distance) hi = mid; else lo = mid;
		}
		return lo;
	}

	// Compute maximum allowed start speed to end at v_end over distance (backward pass)
	static double reachableStartSpeedBackward(double v_end, double distance,
			double v_max, double a_max, double j_max)
	{
		v_end = clamp(v_end, 0.0, v_max);
		if (distance <= 0) return v_end;
		double lo = v_end;
		double hi = v_max;
		for (int i = 0; i < 70; ++i)
		{
			double mid = 0.5 * (lo + hi);
			double s_need = distanceForSpeedChange(mid, v_end, false, a_max, j_max);
			if (std::isnan(s_need) || s_need > distance) hi = mid; else lo = mid;
		}
		return lo;
	}

	// Helper to append a constant-jerk segment and advance state
	static void appendSegment(std::vector<ProfileSegment> &segments,
			double t0, double dt, double j, double &a, double &v, double &s)
	{
		ProfileSegment seg{};
		seg.t0 = t0;
		seg.dt = dt;
		seg.j = j;
		seg.a0 = a;
		seg.v0 = v;
		seg.s0 = s;
		double a1, v1, s1;
		integrateJerk(j, dt, a, v, s, a1, v1, s1);
		a = a1; v = v1; s = s1;
		segments.push_back(seg);
	}

	// Core planner: builds a 7-phase profile (some phases may have zero duration)
	Profile planDoubleS(const Boundary &b, const MotionLimits &lim)
	{
		if (lim.v_max <= 0 || lim.a_max <= 0 || lim.j_max <= 0)
			throw std::invalid_argument("Limits must be positive");

		const double direction = (b.s1 >= b.s0) ? 1.0 : -1.0;
		Boundary bd = b;
		// Normalize to positive direction
		if (direction < 0)
		{
			bd.s0 = -b.s0; bd.s1 = -b.s1; bd.v0 = -b.v0; bd.v1 = -b.v1;
		}

		const double s_total = bd.s1 - bd.s0;
		if (s_total < 1e-12)
		{
			// Trivial: no distance. Try to connect velocities via S-curve around same position
			if (std::abs(bd.v0 - bd.v1) < 1e-9)
			{
				Profile p{}; p.total_time = 0.0; return p;
			}
			throw std::runtime_error("Zero distance but different velocities");
		}

		const double v_max = lim.v_max;
		const double a_max = lim.a_max;
		const double j_max = lim.j_max;

		// Phase durations: t1 j=+j, t2 j=0 a=+a_max, t3 j=-j to end accel
		// Cruise t4 at v=vc, then mirror for decel: t5 j=-j, t6 j=0 a=-a_max, t7 j=+j
		double t1, t2, t3, t4, t5, t6, t7;
		t1 = t2 = t3 = t4 = t5 = t6 = t7 = 0.0;

		// Compute feasible accel ramp to go from v0 to some cruise velocity vc, and from vc to v1
		// First compute max reachable delta v with full S-ramp (accel part):
		// If a_max reached: t1 = a_max / j_max, then a plateau t2 >= 0, then t3 = t1
		const double t_j = a_max / j_max; // time to reach a_max jerk-limited
		const double dv_ramp_full = (a_max * t_j) + (0.5 * a_max * t_j); // dv over t1+t2=0+t3? Needs derivation
		// The above quick formula can be inaccurate; derive accurately:
		// Accel phase with t1, t2, t3 where t1=t3=tj, t2>=0, starting accel a0 unknown. We'll compute using closed-forms below.

		// Strategy: search for profile type: with cruise or without cruise.
		// We'll attempt to compute a candidate with cruise at v_max; if distance insufficient, use no-cruise and adjust peak velocity below v_max.

		auto computeDistanceForAccPhase = [&](double v_start, double v_end, bool accelerating) {
			// accelerating=true for accel phase (increase velocity), false for decel (decrease)
			// Compute minimal-jerk S-curve (no a-plateau) if delta v too small to reach a_max.
			double dv = (accelerating ? (v_end - v_start) : (v_start - v_end));
			if (dv < 0) return std::numeric_limits<double>::quiet_NaN();
			// Case 1: triangular acceleration (no a plateau). Each jerk phase duration t = sqrt(dv / j)
			double t_tri = std::sqrt(std::max(0.0, dv / j_max));
			if (t_tri <= t_j)
			{
				// triangular: t1=t3=t_tri, t2=0; distance of this accel chunk
				double s = 0.0;
				double a = 0.0, v = v_start, x = 0.0;
				// phase +j
				double a1, v1, x1; integrateJerk(+j_max, t_tri, a, v, x, a1, v1, x1);
				// phase -j
				double a2, v2, x2; integrateJerk(-j_max, t_tri, a1, v1, x1, a2, v2, x2);
				(void)a2; (void)v2;
				s = x2 - x;
				return s;
			}
			// Case 2: trapezoidal accel (reach a_max): t1=t3=t_j, t2 chosen so that dv matches
			double t2_local = (dv - a_max * t_j) / a_max; // dv = area under accel curve = a_max*t2 + 0.5*a_max*t_j + 0.5*a_max*t_j
			if (t2_local < 0) t2_local = 0; // numerical guard
			double a = 0.0, v = v_start, x = 0.0;
			double a1, v1, x1; integrateJerk(+j_max, t_j, a, v, x, a1, v1, x1);
			double a2, v2, x2; integrateJerk(0.0,    t2_local, a1, v1, x1, a2, v2, x2);
			double a3, v3, x3; integrateJerk(-j_max, t_j, a2, v2, x2, a3, v3, x3);
			double s = x3 - x;
			return s;
		};

		auto simulatePhase = [&](double v_start, double dv, bool accel, std::vector<ProfileSegment> &segs, double &t_cursor, double &a, double &v, double &x) {
			if (dv < 1e-12) return;
			// triangular vs trapezoidal
			double t_tri = std::sqrt(std::max(0.0, dv / j_max));
			if (t_tri <= t_j)
			{
				appendSegment(segs, t_cursor, t_tri, accel ? +j_max : -j_max, a, v, x);
				t_cursor += t_tri;
				appendSegment(segs, t_cursor, t_tri, accel ? -j_max : +j_max, a, v, x);
				t_cursor += t_tri;
			}
			else
			{
				double t2_local = (dv - a_max * t_j) / a_max;
				if (t2_local < 0) t2_local = 0;
				appendSegment(segs, t_cursor, t_j, accel ? +j_max : -j_max, a, v, x); t_cursor += t_j;
				appendSegment(segs, t_cursor, t2_local, 0.0, a, v, x); t_cursor += t2_local;
				appendSegment(segs, t_cursor, t_j, accel ? -j_max : +j_max, a, v, x); t_cursor += t_j;
			}
		};

		// Attempt with cruise at v_max.
		double s_acc_to_vmax = computeDistanceForAccPhase(bd.v0, v_max, true);
		double s_dec_from_vmax = computeDistanceForAccPhase(v_max, bd.v1, false);
		if (!std::isnan(s_acc_to_vmax) && !std::isnan(s_dec_from_vmax) && s_acc_to_vmax + s_dec_from_vmax <= s_total)
		{
			// With cruise
			double s_cruise = s_total - (s_acc_to_vmax + s_dec_from_vmax);
			std::vector<ProfileSegment> segments;
			double t = 0.0, a = 0.0, v = bd.v0, x = bd.s0;
			// Accel to v_max
			simulatePhase(bd.v0, std::max(0.0, v_max - bd.v0), true, segments, t, a, v, x);
			// Cruise
			if (s_cruise > 1e-12)
			{
				double dt = s_cruise / std::max(1e-12, v_max);
				appendSegment(segments, t, dt, 0.0, a, v, x);
				t += dt;
			}
			// Decel to v1
			simulatePhase(v_max, std::max(0.0, v_max - bd.v1), false, segments, t, a, v, x);

			Profile p; p.segments = std::move(segments); p.total_time = t;
			// Denormalize direction in stored states
			for (auto &seg : p.segments)
			{
				seg.j *= direction;
				seg.a0 *= direction;
				seg.v0 *= direction;
				seg.s0 *= direction;
			}
			return p;
		}

		// No-cruise case: compute peak velocity v_peak < v_max that fits exactly the distance.
		// Use binary search on v_peak.
		double v_lo = std::max({0.0, bd.v0, bd.v1});
		double v_hi = v_max;
		for (int iter = 0; iter < 80; ++iter)
		{
			double v_mid = 0.5 * (v_lo + v_hi);
			double s_acc = computeDistanceForAccPhase(bd.v0, v_mid, true);
			double s_dec = computeDistanceForAccPhase(v_mid, bd.v1, false);
			double s_sum = s_acc + s_dec;
			if (std::isnan(s_acc) || std::isnan(s_dec))
			{
				v_hi = v_mid; continue;
			}
			if (s_sum > s_total) v_hi = v_mid; else v_lo = v_mid;
		}
		double v_peak = v_lo;
		// Build the segments
		std::vector<ProfileSegment> segments;
		double t = 0.0, a = 0.0, v = bd.v0, x = bd.s0;
		simulatePhase(bd.v0, std::max(0.0, v_peak - bd.v0), true, segments, t, a, v, x);
		simulatePhase(v_peak, std::max(0.0, v_peak - bd.v1), false, segments, t, a, v, x);

		Profile p; p.segments = std::move(segments); p.total_time = t;
		for (auto &seg : p.segments)
		{
			seg.j *= direction;
			seg.a0 *= direction;
			seg.v0 *= direction;
			seg.s0 *= direction;
		}
		return p;
	}

	State sample(const Profile &profile, double t)
	{
		if (profile.segments.empty()) return {0, 0, 0};
		if (t <= 0) {
			const auto &s0 = profile.segments.front();
			return {s0.s0, s0.v0, s0.a0};
		}
		double a = 0.0, v = 0.0, x = 0.0;
		bool init = false;
		double t_cursor = 0.0;
		for (const auto &seg : profile.segments)
		{
			if (!init)
			{
				a = seg.a0; v = seg.v0; x = seg.s0; init = true; t_cursor = seg.t0;
			}
			if (t < seg.t0) break;
			double dt = std::min(seg.dt, std::max(0.0, t - seg.t0));
			double a1, v1, x1; integrateJerk(seg.j, dt, a, v, x, a1, v1, x1);
			if (t < seg.t0 + seg.dt) return {x1, v1, a1};
			a = a1; v = v1; x = x1; t_cursor = seg.t0 + seg.dt;
		}
		return {x, v, a};
	}

	Profile planPath(const std::vector<double> &waypoints,
			double v_start, double v_end,
			const MotionLimits &limits)
	{
		if (limits.v_max <= 0 || limits.a_max <= 0 || limits.j_max <= 0)
			throw std::invalid_argument("Limits must be positive");
		if (waypoints.size() < 2)
			throw std::invalid_argument("At least two waypoints required");

		// Check monotonicity and determine direction
		double dir = (waypoints.back() >= waypoints.front()) ? 1.0 : -1.0;
		for (size_t i = 1; i < waypoints.size(); ++i)
		{
			double d = waypoints[i] - waypoints[i - 1];
			if (dir > 0 && d < -1e-12) throw std::invalid_argument("Waypoints must be monotonic");
			if (dir < 0 && d >  1e-12) throw std::invalid_argument("Waypoints must be monotonic");
		}

		// Use speeds as magnitudes in planning, sign applied when building profiles
		double v0 = std::abs(v_start);
		double vN = std::abs(v_end);
		v0 = clamp(v0, 0.0, limits.v_max);
		vN = clamp(vN, 0.0, limits.v_max);

		size_t N = waypoints.size();
		std::vector<double> vf(N, 0.0), vb(N, 0.0), v(N, 0.0);
		vf[0] = v0;
		// Forward pass
		for (size_t i = 0; i + 1 < N; ++i)
		{
			double dist = std::abs(waypoints[i + 1] - waypoints[i]);
			double vend_max = reachableEndSpeedForward(vf[i], dist, limits.v_max, limits.a_max, limits.j_max);
			vf[i + 1] = std::min(limits.v_max, vend_max);
		}
		// Backward pass
		vb[N - 1] = vN;
		for (size_t k = N - 1; k-- > 0; )
		{
			double dist = std::abs(waypoints[k + 1] - waypoints[k]);
			double vstart_max = reachableStartSpeedBackward(vb[k + 1], dist, limits.v_max, limits.a_max, limits.j_max);
			vb[k] = std::min(vf[k], vstart_max);
		}
		v = vb;

		// Build concatenated profile
		Profile total;
		double t_offset = 0.0;
		for (size_t i = 0; i + 1 < N; ++i)
		{
			Boundary b{};
			b.s0 = waypoints[i];
			b.s1 = waypoints[i + 1];
			b.v0 = (dir > 0 ? +v[i] : -v[i]);
			b.v1 = (dir > 0 ? +v[i + 1] : -v[i + 1]);
			Profile sub = planDoubleS(b, limits);
			for (auto seg : sub.segments)
			{
				seg.t0 += t_offset;
				total.segments.push_back(seg);
			}
			t_offset += sub.total_time;
		}
		total.total_time = t_offset;
		return total;
	}
}

