#pragma once

#include "Math/MathFwd.h"

#define FT_TO_M 0.3048
#define DEG_TO_RAD 0.0174532925
#define RAD_TO_DEG 57.29577951

// Earth semimajor and semiminor axes
#define EQUATORIAL_RADIUS 6378.137
#define POLAR_RADIUS 6356.752


double haversine(double lat1, double lon1, double lat2, double lon2);
double bearing(double lat1, double lon1, double lat2, double lon2);

FVector3d lat_lon_to_unit_vec(double lat, double lon);
FVector3d vector_offset_from_origin(double origin_lat, double origin_lon, double ground_height, double lat, double lon, double alt_msl);
FVector3d rotate_ecef_to_ned(FVector3d ecef, double lat, double lon);

double radius_at_latitude(double lat);


FVector2D get_xy_offset_from_origin(double origin_lat, double origin_lon, double target_lat, double target_lon);