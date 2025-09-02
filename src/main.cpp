#include "ds_planner.h"

#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>
#include <cstdlib>

static void usage(const char* prog)
{
	std::cerr << "Usage: " << prog << " s0 s1 v0 v1 v_max a_max j_max dt" << std::endl;
	std::cerr << "Outputs CSV: t,s,v,a" << std::endl;
}

int main(int argc, char** argv)
{
	if (argc < 9)
	{
		usage(argv[0]);
		return 1;
	}

	ds::Boundary b{};
	b.s0 = std::atof(argv[1]);
	b.s1 = std::atof(argv[2]);
	b.v0 = std::atof(argv[3]);
	b.v1 = std::atof(argv[4]);

	ds::MotionLimits lim{};
	lim.v_max = std::atof(argv[5]);
	lim.a_max = std::atof(argv[6]);
	lim.j_max = std::atof(argv[7]);

	double dt = std::atof(argv[8]);
	if (dt <= 0) dt = 0.001;

	ds::Profile p = ds::planDoubleS(b, lim);

	// Determine start state to print initial s0,v0,a0 correctly
	double a0 = 0.0, v0 = 0.0, s0 = 0.0;
	if (!p.segments.empty())
	{
		a0 = p.segments.front().a0;
		v0 = p.segments.front().v0;
		s0 = p.segments.front().s0;
	}

	std::cout << std::fixed << std::setprecision(6);
	std::cout << "t,s,v,a" << '\n';
	for (double t = 0.0; t <= p.total_time + 1e-9; t += dt)
	{
		ds::State st = ds::sample(p, t);
		std::cout << t << "," << st.s << "," << st.v << "," << st.a << '\n';
	}
	// Ensure last sample exactly at total_time
	ds::State st = ds::sample(p, p.total_time);
	if (p.total_time > 0)
		std::cout << p.total_time << "," << st.s << "," << st.v << "," << st.a << '\n';

	return 0;
}

