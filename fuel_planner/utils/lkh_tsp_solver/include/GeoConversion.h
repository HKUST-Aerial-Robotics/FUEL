#ifndef _GEOCONVERSION_H
#define _GEOCONVERSION_H

/*
 * This header specifies the interface for conversion of
 * GEO and GEOM coordinates.
 */

#include <math.h>

void GEO2XYZ(double Lon, double Lat, double* X, double* Y, double* Z);
void XYZ2GEO(double X, double Y, double Z, double* Lon, double* Lat);

void GEOM2XYZ(double Lon, double Lat, double* X, double* Y, double* Z);
void XYZ2GEOM(double X, double Y, double Z, double* Lon, double* Lat);

#endif
