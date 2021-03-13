#include "Segment.h"
#include "LKH.h"

/*
 * The Make2OptMove function makes a 2-opt move by calling the macro Swap1 
 * (i.e., by calling either Flip of Flip_SL). Edges (t1,t2) and (t3,t4) 
 * are exchanged with edges (t2,t3) and (t4,t1). Node t4 is one of t3's 
 * two neighbors on the tour; which one is uniquely determined by the 
 * orientation of (t1,t2).
 */

void Make2OptMove(Node * t1, Node * t2, Node * t3, Node * t4)
{
    Swap1(t1, t2, t3);
}
