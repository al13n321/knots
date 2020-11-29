#pragma once

class Stopwatch {
public:
	Stopwatch();
	~Stopwatch();

	double TimeSinceRestart() const;
	double Restart();

	void ReportAndRestart(const char *task_name);
	void ReportIfGreaterAndRestart(const char *task_name, double min_time);
private:
	long long start_time;
};
