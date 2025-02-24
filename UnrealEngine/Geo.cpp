#include "Geo.h"

#include <math.h>
#include "Math/MathFwd.h"
#include "EngineMinimal.h"
#include <iostream>

double haversine(double lat1, double lon1, double lat2, double lon2)
{
    lat1 = lat1 * DEG_TO_RAD;
    lon1 = lon1 * DEG_TO_RAD;
    lat2 = lat2 * DEG_TO_RAD;
    lon2 = lon2 * DEG_TO_RAD;

    double dlon = lon2 - lon1;
    double dlat = lat2 - lat1;
    double a = pow(sin(dlat/2.0), 2.0) + cos(lat1) * cos(lat2) * pow(sin(dlon/2.0), 2.0);
    double c = 2.0 * asin(sqrtl(a));
    double r = 6371.0;
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
    double target_range = haversine(origin_lat, origin_lon, target_lat, target_lon) * 1000.0;

    // Enable this for Mercator projections 
    //target_range = target_range / cos(origin_lat * DEG_TO_RAD);

    double target_bearing = bearing(origin_lat, origin_lon, target_lat, target_lon) * DEG_TO_RAD;

    //GEngine->AddOnScreenDebugMessage(-1, 10.0f, FColor::Blue, FString::Printf(TEXT("Range: %.2f, Bearing %.5f"), target_range, target_bearing));

    double offset_x = target_range * sin(target_bearing);
    double offset_y = target_range * cos(target_bearing);

    FVector2D ret = {offset_x, offset_y};

    return ret;

}

double radius_at_latitude(double lat){
    double lat_rads = lat * DEG_TO_RAD;

    double a = pow(EQUATORIAL_RADIUS, 2.0) * cos(lat_rads);
    double b = pow(POLAR_RADIUS, 2.0) * sin(lat_rads);
    double c = EQUATORIAL_RADIUS * cos(lat_rads);
    double d = POLAR_RADIUS * sin(lat_rads);

    std::cout << "a: " << a << std::endl;
    std::cout << "b: " << b << std::endl;
    std::cout << "c: " << c << std::endl;
    std::cout << "d: " << d << std::endl;

    double result = 1000 * sqrt( (a*a + b*b) / (c*c + d*d) );
    std::cout << "r: " << result << std::endl;

    return result;
}

FVector3d lat_lon_to_unit_vec(double lat, double lon){
    double lat_rad = lat * DEG_TO_RAD;
    double lon_rad = lon * DEG_TO_RAD;
    double x = cos(lat_rad) * cos(lon_rad);
    double y = cos(lat_rad) * sin(lon_rad);
    double z = sin(lat);

    FVector3d ret = {x, y, z};
    return ret;
}

FVector2D unit_vec_to_lat_lon(FVector3d vec){

    double mag = sqrt(vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2]);

    double lon = atan2(vec[1], vec[0]) * RAD_TO_DEG;
    double lat = asin(vec[2] / mag) * RAD_TO_DEG;

    FVector2D ret = {lat, lon};
    return ret;
}

FVector3d rotate_ecef_to_ned(FVector3d ecef, double lat, double lon){
    double lat_rad = lat * DEG_TO_RAD;
    double lon_rad = lon * DEG_TO_RAD;

    double x = -sin(lat_rad) * cos(lon_rad) * ecef[0] - sin(lat_rad) * sin(lon_rad) * ecef[1] + cos(lat_rad) * ecef[2];
    double y = -sin(lon_rad) * ecef[0] + cos(lon_rad) * ecef[1];
    double z = -cos(lat_rad)  * cos(lon_rad) * ecef[0] - cos(lat_rad) * sin(lon_rad) * ecef[1] - sin(lat_rad) * ecef[2];

    FVector3d ret = {x, y, z};
    return ret;

}

FVector3d vector_offset_from_origin(double origin_lat, double origin_lon, double origin_alt, double lat, double lon, double alt){

    FVector3d v1 = lat_lon_to_unit_vec(origin_lat, origin_lon);
    v1 = rotate_ecef_to_ned(v1, origin_lat, origin_lon);
    double v1_rad = radius_at_latitude(origin_lat) + origin_alt;
    std::cout << "Origin lat: " << origin_lat << std::endl;
    std::cout << "v1 radius: " << v1_rad << std::endl;
    v1 = v1 * v1_rad;

    FVector3d v2 = lat_lon_to_unit_vec(lat, lon);
    v2 = rotate_ecef_to_ned(v2, origin_lat, origin_lon);
    double v2_rad = radius_at_latitude(lat) + alt;
    std::cout << "Lat: " << lat << std::endl;
    std::cout << "v2 radius: " << v2_rad << std::endl;
    v2 = v2 * v2_rad;

    FVector3d offset = v2 - v1;

    std::cout << "v1: " << v1[0] << ", " << v1[2] << ", " << v1[2] << std::endl;
    std::cout << "v2: " << v2[0] << ", " << v2[2] << ", " << v2[2] << std::endl;

    return offset;
}
