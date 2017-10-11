#include "DecisionMaker.h"
#include <limits>
#include <map>
#include <math.h>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include <iostream>

DecisionMaker::DecisionMaker(double safeApproachDistance, double safeGapHigh, double safeGapLow, double patience):
	_safeApproachDistance(safeApproachDistance), _safeGapHigh(safeGapHigh), _safeGapLow(safeGapLow), _safeSpeed(0.0),
	_followCountMax(static_cast<int>(patience / 0.02)) {}

carState DecisionMaker::Decide(carState currentState, int lane, double s, double d, double v, int prevPath, const std::vector<std::vector<double>> &sensorFusion) {

	// Reminder: sensor fusion structure
	// [0] Car ID
	// [1] x
	// [2] y
	// [3] vx
	// [4] vy
	// [5] s
	// [6] d

	switch (currentState) {
		case carState::ST_KEEPLANE:
			return ST_KeepLane(currentState, lane, s, d, v, prevPath, sensorFusion);
		case carState::ST_ADJUSTSPEED:
			return ST_KeepLane(currentState, lane, s, d, v, prevPath, sensorFusion);
		case carState::ST_CHANGELANELEFT:
			return ST_ChangeLane(currentState, lane, s, d, v, prevPath, sensorFusion);
		case carState::ST_CHANGELANERIGHT:
			return ST_ChangeLane(currentState, lane, s, d, v, prevPath, sensorFusion);
		case carState::ST_CHANGETWOLANESLEFT:
			return ST_ChangeLane(currentState, lane, s, d, v, prevPath, sensorFusion);
		case carState::ST_CHANGETWOLANESRIGHT:
			return ST_ChangeLane(currentState, lane, s, d, v, prevPath, sensorFusion);
	}
	
	if (currentState == carState::ST_KEEPLANE || currentState == carState::ST_ADJUSTSPEED) {

	}
}

carState DecisionMaker::ST_KeepLane(carState state, int lane, double s, double d, double v, int prevPath, const std::vector<std::vector<double>> &sensorFusion) {
	// check if the road is clear
	
	if (state == carState::ST_KEEPLANE) {
		_followCount = 0;
	}
	

	bool roadClear = true;

	int closestCarID = -1;
	int closestCarDistance = std::numeric_limits<int>::max();
	double safeSpeed = 0.0;

	std::map<int, std::vector<std::vector<double>>> laneTraffic;

	for (const auto &car : sensorFusion) {

		float carD = car[6];

		if (carD >= 0 && carD < 4) {
			laneTraffic[0].push_back(car);
		} else if (carD >= 4 && carD < 8) {
			laneTraffic[1].push_back(car);
		} else if (carD >= 8 && carD < 12) {
			laneTraffic[2].push_back(car);
		}

		//check if car is in current lane
    if (carD < (2+4*lane+2) && carD > (2+4*lane-2)) {
      int id = car[0];
      double vx = car[3];
      double vy = car[4];
      double carSpeed = sqrt(vx*vx+vy*vy);
      double carS = car[5];

      // predict where the car would be at the end of the current path
      //carS += (double)prevPath * .02 * carSpeed;

      double distance = carS-s;

      // check if the car is in front of us and if it's too close
      if (carS > s && distance < _safeApproachDistance){

      	if (distance < closestCarDistance) {
      		closestCarDistance = distance;
      		closestCarID = id;
      		safeSpeed = carSpeed;
      	}

      	roadClear = false;
      }
    }
	}

	// stay in lane if road is clear
	if (roadClear) {
		return carState::ST_KEEPLANE;
	}

	// decide what to do if road is blocked

	// check left
	if (lane > 0) {
		bool leftLaneClear = true;

		for (const auto &car : laneTraffic[lane-1]) {
			double carS = car[5];

			//         | cs______s_________cs|
			//--------#######-----------#######--------
			//----------------EEEEEEE-----#######------
			//-------------------########-------------

			if ((carS >= s && (carS-s) < _safeGapHigh) ||
			    (carS < s && (s-carS) < _safeGapLow)) {
				leftLaneClear = false;
				break;
			}
		}

		if (leftLaneClear) {
			return carState::ST_CHANGELANELEFT;
		}
	}

	// check right
	if (lane < 2) {
		bool rightLaneClear = true;

		for (const auto &car : laneTraffic[lane+1]) {
			double carS = car[5];

			//-------------------########-------------
			//----------------EEEEEEE-----#######------
			//--------#######-----------#######--------
			//         | cs______s_________cs|

			if ((carS >= s && (carS-s) < _safeGapHigh) ||
			    (carS < s && (s-carS) < _safeGapLow)) {
				rightLaneClear = false;
				break;
			}
		}

		if (rightLaneClear) {
			return carState::ST_CHANGELANERIGHT;
		}
	}

	// check two lane takeover
	if ((lane == 0 || lane == 2) && (_followCount > _followCountMax)) {

		std::cout << "Trying two lane takeover" << std::endl;

		int checkLane;
		if (lane == 0) {
			checkLane = 2;
		} else {
			checkLane = 0;
		}

		// check the middle lane first
		bool middleLaneClear = true;

		for (const auto &car : laneTraffic[1]) {
			double carS = car[5];

			if ((carS >= s && (carS-s) < _safeGapLow) ||
			    (carS < s && (s-carS) < _safeGapLow)) {
				middleLaneClear = false;
				break;
			}
		}

		if (middleLaneClear) {
			bool laneClear = true;

			for (const auto &car : laneTraffic[checkLane]) {
				double carS = car[5];

				if ((carS >= s && (carS-s) < _safeGapHigh) ||
				    (carS < s && (s-carS) < _safeGapLow)) {
					laneClear = false;
					break;
				}
			}

			if (laneClear) {
				if (checkLane == 0) {
					return carState::ST_CHANGETWOLANESLEFT;
				}
				else {
					return carState::ST_CHANGETWOLANESRIGHT;
				}
			}
		}
	}

	// Still no decision? Well, then just stay in lane and adjust speed.
	_safeSpeed = safeSpeed;

	return ST_AdjustSpeed();
}

carState DecisionMaker::ST_ChangeLane(carState state, int lane, double s, double d, double v, int prevPath, const std::vector<std::vector<double>> &sensorFusion) const {
	// check if lane change is completed

	if (d < (2+4*lane+1) && d > (2+4*lane-1)) {
		return carState::ST_KEEPLANE;
	} else {
		return state;
	}
}

carState DecisionMaker::ST_AdjustSpeed() {
	
	_followCount++;

	return carState::ST_ADJUSTSPEED;
}