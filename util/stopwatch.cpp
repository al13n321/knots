#include "stopwatch.h"

#ifdef _WIN32
#include <windows.h>
#endif

#ifdef __APPLE__
#include <mach/mach_time.h>
#endif

#include <iostream>
using namespace std;

static long long GetTime() {
#ifdef _WIN32
    LARGE_INTEGER t;
    QueryPerformanceCounter(&t);
    return t.QuadPart;
#endif

#ifdef __APPLE__
    return static_cast<long long>(mach_absolute_time());
#endif
}

static inline double GetClocksPerSecond() {
	static double saved = -1;

	if (saved >= 0)
		return saved;

#ifdef _WIN32
	LARGE_INTEGER t;
	QueryPerformanceFrequency(&t);
	saved = static_cast<double>(t.QuadPart);
#endif
    
#ifdef __APPLE__
    mach_timebase_info_data_t timebase;
    kern_return_t error = mach_timebase_info(&timebase);
    if(error == 0)
        saved = 1e9 * (double) timebase.numer / (double) timebase.denom;
    else
        saved = 0;
#endif

	return saved;
}
    
Stopwatch::Stopwatch() {
	Restart();
}

Stopwatch::~Stopwatch() {}

double Stopwatch::TimeSinceRestart() const {
	long long t = GetTime();
	return (t - start_time) / GetClocksPerSecond();
}

double Stopwatch::Restart() {
	long long t = GetTime();
	double res = (t - start_time) / GetClocksPerSecond();
	start_time = t;
	return res;
}

void Stopwatch::ReportAndRestart(const char *task_name) {
	ReportIfGreaterAndRestart(task_name, -1);
}

void Stopwatch::ReportIfGreaterAndRestart(const char *task_name, double min_time) {
	long long t = GetTime();
	double r = (t - start_time) / GetClocksPerSecond();
	if (r > min_time)
		cerr << task_name << " took " << r << " seconds" << endl;
	start_time = t;
}
