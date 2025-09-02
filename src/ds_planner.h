#pragma once

#include <vector>
#include <cstdint>

namespace ds
{
	struct MotionLimits
	{
		double v_max;   // maximum speed (m/s)
		double a_max;   // maximum acceleration magnitude (m/s^2)
		double j_max;   // maximum jerk magnitude (m/s^3)
	};

	struct Boundary
	{
		double s0;      // start position
		double s1;      // target position
		double v0;      // start velocity
		double v1;      // end velocity
	};

	struct ProfileSegment
	{
		double t0;      // segment start time
		double dt;      // segment duration
		double j;       // constant jerk in this segment
		double a0;      // acceleration at segment start
		double v0;      // velocity at segment start
		double s0;      // position at segment start
	};

	struct Profile
	{
		std::vector<ProfileSegment> segments; // up to 7 for symmetric double-S, more for edge cases
		double total_time{};
	};

	// Compute a jerk-limited double-S profile for 1D motion along a Cartesian axis.
	// Supports non-zero start/end velocities, and enforces v_max, a_max, j_max.
	// Throws std::runtime_error on infeasible requests.
	Profile planDoubleS(const Boundary &bounds, const MotionLimits &limits);

	// Sample the profile at time t in [0, total_time]. Returns (s, v, a).
	struct State { double s; double v; double a; };
	State sample(const Profile &profile, double t);

}

