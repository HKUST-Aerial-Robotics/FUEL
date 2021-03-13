#include <stdio.h>
#include <ctype.h>

/*
 * The fscanint function reads the next int integer from the stream f,
 * and assigns the value through the second argmument, v, which must be
 * a pointer. It returns 0 if end of file or an error occurs; otherwise 
 * it returns 1.
 *
 * It is faster than fscanf.
 */

int fscanint(FILE * f, int *v)
{
    int val;
    int c, sign = 1;

    while (isspace(c = getc(f)));
    if (c == '-' || c == '+') {
        if (c == '-')
            sign = -1;
        if ((c = getc(f)) == EOF) {
            ungetc(c, f);
            return 0;
        }
    }
    if (!isdigit(c)) {
        ungetc(c, f);
        return 0;
    }
    val = c - '0';
    while (isdigit(c = getc(f)))
        val = 10 * val + (c - '0');
    *v = sign * val;
    return 1;
}
