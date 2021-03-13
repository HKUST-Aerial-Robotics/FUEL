#ifndef _GAINTYPE_H
#define _GAINTYPE_H

#define HAVE_LONG_LONG
/* Undefine if you don't have the long long type */
/* #undef HAVE_LONG_LONG */

#include <float.h>
#include <limits.h>

#ifdef HAVE_LONG_LONG
typedef long long GainType;
#ifndef LLONG_MAX
#define LLONG_MAX 9223372036854775807LL
#endif
#ifndef LLONG_MIN
#define LLONG_MIN (-LLONG_MAX - 1LL)
#endif
#define PLUS_INFINITY LLONG_MAX
#define MINUS_INFINITY LLONG_MIN
#define GainFormat "%lld"
#define GainInputFormat "%lld"
#else
typedef double GainType;
#define PLUS_INFINITY DBL_MAX
#define MINUS_INFINITY -DBL_MAX
#define GainFormat "%0.0lf"
#define GainInputFormat "%0.0lf"
#endif

#endif
