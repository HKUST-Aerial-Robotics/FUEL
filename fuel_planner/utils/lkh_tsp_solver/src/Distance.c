#include "LKH.h"

/*
 * Functions for computing distances (see TSPLIB).
 *
 * The appropriate function is referenced by the function pointer Distance.
 */

int Distance_1(Node * Na, Node * Nb)
{
    return 1;
}

int Distance_LARGE(Node * Na, Node * Nb)
{
    return 10000000;
}

int Distance_ATSP(Node * Na, Node * Nb)
{
    int n = DimensionSaved;
    if ((Na->Id <= n) == (Nb->Id <= n))
        return M;
    if (abs(Na->Id - Nb->Id) == n)
        return 0;
    return Na->Id <= n ? Na->C[Nb->Id - n] : Nb->C[Na->Id - n];
}

int Distance_ATT(Node * Na, Node * Nb)
{
    double xd = Na->X - Nb->X, yd = Na->Y - Nb->Y;
    return (int) ceil(sqrt((xd * xd + yd * yd) / 10.0));
}

int Distance_CEIL_2D(Node * Na, Node * Nb)
{
    double xd = Na->X - Nb->X, yd = Na->Y - Nb->Y;
    return (int) ceil(sqrt(xd * xd + yd * yd));
}

int Distance_CEIL_3D(Node * Na, Node * Nb)
{
    double xd = Na->X - Nb->X, yd = Na->Y - Nb->Y, zd = Na->Z - Nb->Z;
    return (int) ceil(sqrt(xd * xd + yd * yd + zd * zd));
}

int Distance_EUC_2D(Node * Na, Node * Nb)
{
    double xd = Na->X - Nb->X, yd = Na->Y - Nb->Y;
    return (int) (sqrt(xd * xd + yd * yd) + 0.5);
}

int Distance_EUC_3D(Node * Na, Node * Nb)
{
    double xd = Na->X - Nb->X, yd = Na->Y - Nb->Y, zd = Na->Z - Nb->Z;
    return (int) (sqrt(xd * xd + yd * yd + zd * zd) + 0.5);
}

int Distance_EXPLICIT(Node * Na, Node * Nb)
{
    return Na->Id < Nb->Id ? Nb->C[Na->Id] : Na->C[Nb->Id];
}

int Distance_FLOOR_2D(Node * Na, Node * Nb)
{
    double xd = Na->X - Nb->X, yd = Na->Y - Nb->Y;
    return (int) floor(sqrt(xd * xd + yd * yd));
}

int Distance_FLOOR_3D(Node * Na, Node * Nb)
{
    double xd = Na->X - Nb->X, yd = Na->Y - Nb->Y, zd = Na->Z - Nb->Z;
    return (int) floor(sqrt(xd * xd + yd * yd + zd * zd));
}

#define PI 3.141592
#define RRR 6378.388

int Distance_GEO(Node * Na, Node * Nb)
{
    int deg;
    double NaLatitude, NaLongitude, NbLatitude, NbLongitude, min,
        q1, q2, q3;
    deg = (int) Na->X;
    min = Na->X - deg;
    NaLatitude = PI * (deg + 5.0 * min / 3.0) / 180.0;
    deg = (int) Na->Y;
    min = Na->Y - deg;
    NaLongitude = PI * (deg + 5.0 * min / 3.0) / 180.0;
    deg = (int) Nb->X;
    min = Nb->X - deg;
    NbLatitude = PI * (deg + 5.0 * min / 3.0) / 180.0;
    deg = (int) Nb->Y;
    min = Nb->Y - deg;
    NbLongitude = PI * (deg + 5.0 * min / 3.0) / 180.0;
    q1 = cos(NaLongitude - NbLongitude);
    q2 = cos(NaLatitude - NbLatitude);
    q3 = cos(NaLatitude + NbLatitude);
    return (int) (RRR * acos(0.5 * ((1.0 + q1) * q2 - (1.0 - q1) * q3)) +
                  1.0);
}

#undef M_PI
#define M_PI 3.14159265358979323846264
#define M_RRR 6378388.0

int Distance_GEOM(Node * Na, Node * Nb)
{
    double lati = M_PI * (Na->X / 180.0);
    double latj = M_PI * (Nb->X / 180.0);
    double longi = M_PI * (Na->Y / 180.0);
    double longj = M_PI * (Nb->Y / 180.0);
    double q1 = cos(latj) * sin(longi - longj);
    double q3 = sin((longi - longj) / 2.0);
    double q4 = cos((longi - longj) / 2.0);
    double q2 = sin(lati + latj) * q3 * q3 - sin(lati - latj) * q4 * q4;
    double q5 = cos(lati - latj) * q4 * q4 - cos(lati + latj) * q3 * q3;
    return (int) (M_RRR * atan2(sqrt(q1 * q1 + q2 * q2), q5) + 1.0);
}

int Distance_MAN_2D(Node * Na, Node * Nb)
{
    return (int) (fabs(Na->X - Nb->X) + fabs(Na->Y - Nb->Y) + 0.5);
}

int Distance_MAN_3D(Node * Na, Node * Nb)
{
    return (int) (fabs(Na->X - Nb->X) +
                  fabs(Na->Y - Nb->Y) + fabs(Na->Z - Nb->Z) + 0.5);
}

int Distance_MAX_2D(Node * Na, Node * Nb)
{
    int dx = (int) (fabs(Na->X - Nb->X) + 0.5),
        dy = (int) (fabs(Na->Y - Nb->Y) + 0.5);
    return dx > dy ? dx : dy;
}

int Distance_MAX_3D(Node * Na, Node * Nb)
{
    int dx = (int) (fabs(Na->X - Nb->X) + 0.5),
        dy = (int) (fabs(Na->Y - Nb->Y) + 0.5),
        dz = (int) (fabs(Na->Z - Nb->Z) + 0.5);
    if (dy > dx)
        dx = dy;
    return dx > dz ? dx : dz;
}

/* Function for computing the distance in kilometers between two points on
   the Earth's surface, based on the high accuracy method by H. Andoyer,
   as described in
 
        "Astronomical Algorithms (2nd Ed.)", pg. 85, Jean Meeus (2000).
*/

static double Meeus(double lat1, double lon1, double lat2, double lon2)
{
    const double a = 6378.137;  /* equator earth radius */
    const double fl = 1 / 298.257;      /* earth flattening */
    double f, g, l, sg, sl, sf, s, c, w, r, d, h1, h2;

    if (lat1 == lat2 && lon1 == lon2)
        return 0;
    f = (lat1 + lat2) / 2;
    g = (lat1 - lat2) / 2;
    l = (lon1 - lon2) / 2;
    sg = sin(g);
    sl = sin(l);
    sf = sin(f);
    sg = sg * sg;
    sl = sl * sl;
    sf = sf * sf;
    s = sg * (1 - sl) + (1 - sf) * sl;
    c = (1 - sg) * (1 - sl) + sf * sl;
    w = atan(sqrt(s / c));
    r = sqrt(s * c) / w;
    d = 2 * w * a;
    h1 = (3 * r - 1) / 2 / c;
    h2 = (3 * r + 1) / 2 / s;
    return d * (1 + fl * (h1 * sf * (1 - sg) - h2 * (1 - sf) * sg));
}

int Distance_GEO_MEEUS(Node * Na, Node * Nb)
{
    double lat1 =
        M_PI * ((int) Na->X + 5 * (Na->X - (int) Na->X) / 3) / 180;
    double lon1 =
        M_PI * ((int) Na->Y + 5 * (Na->Y - (int) Na->Y) / 3) / 180;
    double lat2 =
        M_PI * ((int) Nb->X + 5 * (Nb->X - (int) Nb->X) / 3) / 180;
    double lon2 =
        M_PI * ((int) Nb->Y + 5 * (Nb->Y - (int) Nb->Y) / 3) / 180;
    return (int) (Meeus(lat1, lon1, lat2, lon2) + 0.5);
}

int Distance_GEOM_MEEUS(Node * Na, Node * Nb)
{
    double lat1 = M_PI * (Na->X / 180);
    double lon1 = M_PI * (Na->Y / 180);
    double lat2 = M_PI * (Nb->X / 180);
    double lon2 = M_PI * (Nb->Y / 180);
    return (int) (1000 * Meeus(lat1, lon1, lat2, lon2) + 0.5);
}

#undef min
#undef max

static double min(double a, double b)
{
    return a < b ? a : b;
}

static double max(double a, double b)
{
    return a > b ? a : b;
}

int Distance_TOR_2D(Node * Na, Node * Nb)
{
    double dx = fabs(Na->X - Nb->X);
    double dy = fabs(Na->Y - Nb->Y);
    dx = min(dx, GridSize - dx);
    dy = min(dy, GridSize - dy);
    return (int) (sqrt(dx * dx + dy * dy) + 0.5);
}

int Distance_TOR_3D(Node * Na, Node * Nb)
{
    double dx = fabs(Na->X - Nb->X);
    double dy = fabs(Na->Y - Nb->Y);
    double dz = fabs(Na->Z - Nb->Z);
    dx = min(dx, GridSize - dx);
    dy = min(dy, GridSize - dy);
    dz = min(dz, GridSize - dz);
    return (int) (sqrt(dx * dx + dy * dy + dz * dz) + 0.5);
}

int Distance_XRAY1(Node * Na, Node * Nb)
{
    double distp =
        min(fabs(Na->X - Nb->X), fabs(fabs(Na->X - Nb->X) - 360));
    double distc = fabs(Na->Y - Nb->Y);
    double distt = fabs(Na->Z - Nb->Z);
    double cost = max(distp, max(distc, distt));
    return (int) (100 * cost + 0.5);
}

int Distance_XRAY2(Node * Na, Node * Nb)
{
    double distp =
        min(fabs(Na->X - Nb->X), fabs(fabs(Na->X - Nb->X) - 360));
    double distc = fabs(Na->Y - Nb->Y);
    double distt = fabs(Na->Z - Nb->Z);
    double cost = max(distp / 1.25, max(distc / 1.5, distt / 1.15));
    return (int) (100 * cost + 0.5);
}
