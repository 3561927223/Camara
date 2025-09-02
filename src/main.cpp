#include "ds_planner.h"

#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>
#include <cstdlib>

static void usage(const char* prog)
{
	std::cerr << "Usage:" << std::endl;
	std::cerr << "  " << prog << " s0 s1 v0 v1 v_max a_max j_max dt" << std::endl;
	std::cerr << "  " << prog << " --path w0,w1,... v_start v_end v_max a_max j_max dt" << std::endl;
	std::cerr << "Outputs CSV: t,s,v,a" << std::endl;
}

int main(int argc, char** argv)
{
	if (argc < 2)
	{
		usage(argv[0]);
		return 1;
	}

	bool path_mode = (std::string(argv[1]) == "--path");

	ds::MotionLimits lim{};
	double dt = 0.001;
	ds::Profile p;

	if (!path_mode)
	{
		if (argc < 9) { usage(argv[0]); return 1; }
		ds::Boundary b{};
		b.s0 = std::atof(argv[1]);
		b.s1 = std::atof(argv[2]);
		b.v0 = std::atof(argv[3]);
		b.v1 = std::atof(argv[4]);
		lim.v_max = std::atof(argv[5]);
		lim.a_max = std::atof(argv[6]);
		lim.j_max = std::atof(argv[7]);
		dt = std::atof(argv[8]); if (dt <= 0) dt = 0.001;
		p = ds::planDoubleS(b, lim);
	}
	else
	{
		if (argc < 9) { usage(argv[0]); return 1; }
		// argv[2] is waypoints CSV
		std::vector<double> wpts; wpts.reserve(16);
		{
			std::string csv = argv[2];
			std::stringstream ss(csv);
			std::string tok;
			while (std::getline(ss, tok, ','))
			{
				if (tok.empty()) continue;
				wpts.push_back(std::atof(tok.c_str()));
			}
		}
		double v_start = std::atof(argv[3]);
		double v_end   = std::atof(argv[4]);
		lim.v_max = std::atof(argv[5]);
		lim.a_max = std::atof(argv[6]);
		lim.j_max = std::atof(argv[7]);
		dt = std::atof(argv[8]); if (dt <= 0) dt = 0.001;
		p = ds::planPath(wpts, v_start, v_end, lim);
	}

	std::cout << std::fixed << std::setprecision(6);
	std::cout << "t,s,v,a" << '\n';
	for (double t = 0.0; t <= p.total_time + 1e-9; t += dt)
	{
		ds::State st = ds::sample(p, t);
		std::cout << t << "," << st.s << "," << st.v << "," << st.a << '\n';
	}
	ds::State st = ds::sample(p, p.total_time);
	if (p.total_time > 0)
		std::cout << p.total_time << "," << st.s << "," << st.v << "," << st.a << '\n';

	return 0;
}

