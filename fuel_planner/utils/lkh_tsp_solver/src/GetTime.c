#define HAVE_GETRUSAGE
/* Undefine if you don't have the getrusage function */
#undef HAVE_GETRUSAGE

/*
 * The GetTime function is used to measure execution time.
 *
 * The function is called before and after the code to be 
 * measured. The difference between the second and the
 * first call gives the number of seconds spent in executing
 * the code.
 *
 * If the system call getrusage() is supported, the difference 
 * gives the user time used; otherwise, the accounted real time.
 */

#ifdef HAVE_GETRUSAGE
#include <sys/time.h>
#include <sys/resource.h>

double GetTime()
{
    struct rusage ru;
    getrusage(RUSAGE_SELF, &ru);
    return ru.ru_utime.tv_sec + ru.ru_utime.tv_usec / 1000000.0;
}

#else

#include <time.h>

double GetTime()
{
    return (double) clock() / CLOCKS_PER_SEC;
}

#endif
