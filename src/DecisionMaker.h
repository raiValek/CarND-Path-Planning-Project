#pragma once

#include <vector>
#include "State.h"

class DecisionMaker {
public:
	DecisionMaker(double safeDistance, double safeGapFront, double safeGapBack, double patience);

	carState Decide(carState currentState, int lane, double s, double d, double v, int prevPath, const std::vector<std::vector<double>> &sensorFusion);

	inline double GetSafeSpeed() const {return _safeSpeed * 2.24;}

private:
	// parameters
	const double _safeApproachDistance;
	const double _safeGapHigh;
	const double _safeGapLow;
	const int _followCountMax;

	double _safeSpeed;
	int _followCount;

	carState ST_KeepLane(carState state, int lane, double s, double d, double v, int prevPath, const std::vector<std::vector<double>> &sensorFusion);

	carState ST_ChangeLane(carState state, int lane, double s, double d, double v, int prevPath, const std::vector<std::vector<double>> &sensorFusion) const;

	carState ST_AdjustSpeed();
};