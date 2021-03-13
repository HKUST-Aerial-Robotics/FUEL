#include "Segment.h"
#include "LKH.h"

/*
 * The Make4OptMove function makes a 4-opt move by calling the macro Swap3.
 */

void
Make4OptMove(Node * t1, Node * t2, Node * t3, Node * t4,
             Node * t5, Node * t6, Node * t7, Node * t8, int Case)
{
    if (SUC(t1) != t2)
        Reversed ^= 1;
    switch (Case) {
    case 1:
    case 2:
        Swap3(t1, t2, t3, t6, t5, t4, t7, t8, t1);
        return;
    case 3:
    case 4:
        Swap3(t1, t2, t3, t8, t7, t6, t5, t8, t1);
        return;
    case 5:
        if (!BETWEEN(t2, t7, t3))
            Swap3(t5, t6, t7, t2, t1, t4, t1, t4, t5);
        else if (BETWEEN(t2, t7, t6))
            Swap3(t5, t6, t7, t5, t8, t3, t3, t8, t1);
        else
            Swap3(t1, t2, t7, t7, t2, t3, t4, t7, t6);
        return;
    case 6:
        Swap3(t3, t4, t5, t6, t3, t2, t1, t6, t7);
        return;
    case 7:
        Swap3(t6, t5, t8, t2, t1, t4, t8, t5, t4);
        return;
    case 11:
        Swap3(t1, t2, t7, t3, t4, t5, t3, t6, t7);
        return;
    case 12:
        Swap3(t3, t4, t5, t7, t8, t1, t3, t6, t7);
        return;
    case 15:
        Swap3(t3, t4, t5, t3, t6, t7, t8, t3, t2);
        return;
    default:
        eprintf("Make4OptMove: Internal error");
    }
}
