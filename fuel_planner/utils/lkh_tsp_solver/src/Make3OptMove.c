#include "Segment.h"
#include "LKH.h"

/*
 * The Make3OptMove function makes a 3-opt move by calling the macro Swap2 
 * or Swap3.
 */

void
Make3OptMove(Node * t1, Node * t2, Node * t3, Node * t4,
             Node * t5, Node * t6, int Case)
{
    switch (Case) {
    case 1:
    case 2:
        Swap2(t1, t2, t3, t6, t5, t4);
        return;
    case 5:
        Swap3(t1, t2, t4, t6, t5, t4, t6, t2, t3);
        return;
    case 6:
        Swap2(t3, t4, t5, t1, t2, t3);
        return;
    default:
        eprintf("Make3OptMove: Internal error");
    }
}
