#include "GeoConversion.h"

/* 
 * Functions for conversion between geographical longitude and altitude
 * coordinates and 3D Cartesian coordinates.
 * The coordinates may be given as degrees and minutes (GEO),
 * or as degrees in decimal form (GEOM). 
 */

#define PI 3.141592
#define RRR 6378.388

#undef M_PI
#define M_PI 3.14159265358979323846264
#define M_RRR 6378388.0

void GEO2XYZ(double Lon, double Lat, double *X, double *Y, double *Z)
{
    double lon = (int) Lon + 5.0 * (Lon - (int) Lon) / 3.0;
    double lat = (int) Lat + 5.0 * (Lat - (int) Lat) / 3.0;
    double XRad = PI * lon / 180.0;
    double YRad = PI * lat / 180.0;
    double cosXRad = cos(XRad);

    *X = RRR * cosXRad * cos(YRad);
    *Y = RRR * cosXRad * sin(YRad);
    *Z = RRR * sin(XRad);
}

void XYZ2GEO(double X, double Y, double Z, double *Lon, double *Lat)
{
    double lon = asin(Z / sqrt(X * X + Y * Y + Z * Z)) * 180 / PI;
    double lat = atan2(Y, X) * 180 / PI;
    *Lon = (int) lon + 3.0 * (lon - (int) lon) / 5.0;
    *Lat = (int) lat + 3.0 * (lat - (int) lat) / 5.0;
}

void GEOM2XYZ(double Lon, double Lat, double *X, double *Y, double *Z)
{
    double XRad = M_PI * Lon / 180.0;
    double YRad = M_PI * Lat / 180.0;
    double cosXRad = cos(XRad);

    *X = M_RRR * cosXRad * cos(YRad);
    *Y = M_RRR * cosXRad * sin(YRad);
    *Z = M_RRR * sin(XRad);
}

void XYZ2GEOM(double X, double Y, double Z, double *Lon, double *Lat)
{
    *Lon = asin(Z / sqrt(X * X + Y * Y + Z * Z)) * 180 / M_PI;
    *Lat = atan2(Y, X) * 180 / M_PI;
}
