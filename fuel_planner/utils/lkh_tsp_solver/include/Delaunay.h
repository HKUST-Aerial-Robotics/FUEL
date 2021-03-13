#ifndef _DELAUNAY_H
#define _DELAUNAY_H

/*
 * This header specifies the interface for the use of Delaunay graphs.
 */

#define Org(e) ((e)->org)
#define Dest(e) ((e)->dest)
#define Onext(e) ((e)->onext)
#define Oprev(e) ((e)->oprev)
#define Dnext(e) ((e)->dnext)
#define Dprev(e) ((e)->dprev)

#define Other_point(e, p) ((e)->org == p ? (e)->dest : (e)->org)
#define Next(e, p) ((e)->org == p ? (e)->onext : (e)->dnext)
#define Prev(e, p) ((e)->org == p ? (e)->oprev : (e)->dprev)

#define Vector(p1, p2, u, v) (u = p2->x - p1->x, v = p2->y - p1->y)
#define Cross_product_2v(u1, v1, u2, v2) (u1 * v2 - v1 * u2)
#define Cross_product_3p(p1, p2, p3)                                                                    \
  ((p2->x - p1->x) * (p3->y - p1->y) - (p2->y - p1->y) * (p3->x - p1->x))
#define Dot_product_2v(u1, v1, u2, v2) (u1 * u2 + v1 * v2)

typedef struct point point;
typedef struct edge edge;
typedef double real;

struct point {
  real x, y;
  int id;
  edge* entry_pt;
};

struct edge {
  point *org, *dest;
  edge *onext, *oprev, *dnext, *dprev;
};

typedef enum { right, left } side;

extern point* p_array;

void delaunay(int n);
void free_memory();

#endif
