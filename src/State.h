#pragma once

#include <vector>
#include <string>

	enum carState {
		ST_KEEPLANE,
		ST_CHANGELANELEFT,
		ST_CHANGELANERIGHT,
		ST_CHANGETWOLANESLEFT,
		ST_CHANGETWOLANESRIGHT,
		ST_ADJUSTSPEED
	};

	static std::vector<std::string> carStateNames = {
		"ST_KEEPLANE",
		"ST_CHANGELANELEFT",
		"ST_CHANGELANERIGHT",
		"ST_CHANGETWOLANESLEFT",
		"ST_CHANGETWOLANESRIGHT",
		"ST_ADJUSTSPEED"
	};