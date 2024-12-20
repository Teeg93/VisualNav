#pragma once

#include "Math/MathFwd.h"

#define FT_TO_M 0.3048
#define DEG_TO_RAD 0.0174532925
#define RAD_TO_DEG 57.29577951


double haversine(double lat1, double lon1, double lat2, double lon2);
double bearing(double lat1, double lon1, double lat2, double lon2);
FVector2D get_xy_offset_from_origin(double origin_lat, double origin_lon, double target_lat, double target_lon);