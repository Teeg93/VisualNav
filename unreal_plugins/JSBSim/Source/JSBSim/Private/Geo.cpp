#include "Geo.h"

#include <math.h>
#include "Math/MathFwd.h"

double haversine(double lat1, double lon1, double lat2, double lon2)
{
    lat1 = lat1 * DEG_TO_RAD;
    lon1 = lon1 * DEG_TO_RAD;
    lat2 = lat2 * DEG_TO_RAD;
    lon2 = lon2 * DEG_TO_RAD;

    double dlon = lon2 - lon1;
    double dlat = lat2 - lat1;
    double a = pow(sin(dlat/2),2) + cos(lat1) * cos(lat2) * pow(sin(dlon/2), 2);
    double c = 2 * asin(sqrt(a));
    double r = 6371;
    return c * r;
}

double bearing(double lat1, double lon1, double lat2, double lon2)
{
    lat1 = lat1 * DEG_TO_RAD;
    lon1 = lon1 * DEG_TO_RAD;
    lat2 = lat2 * DEG_TO_RAD;
    lon2 = lon2 * DEG_TO_RAD;

    double dlon = lon2 - lon1;

    double y = sin(dlon) * cos(lat2);
    double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon);
    double b = atan2(y, x);

    double bearing = b * RAD_TO_DEG;

    return bearing;
}

FVector2D get_xy_offset_from_origin(double origin_lat, double origin_lon, double target_lat, double target_lon)
{
    double target_range = haversine(origin_lat, origin_lon, target_lat, target_lon) * 1000;

    // Enable this for Mercator projections 
    //target_range = target_range / cos(origin_lat * DEG_TO_RAD);

    double target_bearing = bearing(origin_lat, origin_lon, target_lat, target_lon) * DEG_TO_RAD;

    double offset_x = target_range * sin(target_bearing);
    double offset_y = target_range * cos(target_bearing);

    FVector2D ret = {offset_x, offset_y};

    return ret;

}