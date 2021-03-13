#include "Segment.h"
#include "LKH.h"

/*
 * The NormalizeNodeList function is used to swap the Suc and Pred fields 
 * of nodes in such a way that the list of nodes constitutes a cyclic 
 * two-way list. 
 *
 * A call of the function corrupts the segment list representation.   
 */

void NormalizeNodeList()
{
    Node *t1, *t2;

    t1 = FirstNode;
    do {
        t2 = SUC(t1);
        t1->Pred = PRED(t1);
        t1->Suc = t2;
        t1->Parent = 0;
    }
    while ((t1 = t2) != FirstNode);
}
