#include "LKH.h"

/*
 * The NormalizeSegmentList function is used to swap the Suc and Pred fields 
 * of segments in such a way that the list of segments constitutes a cyclic 
 * two-way list. 
 *
 * A call of the function corrupts the tree representation of the tour.   
 *
 * The function is called from LinKernighan.   
 */

void NormalizeSegmentList()
{
    Segment *s1, *s2;

    s1 = FirstSegment;
    do {
        if (!s1->Parent->Reversed)
            s2 = s1->Suc;
        else {
            s2 = s1->Pred;
            s1->Pred = s1->Suc;
            s1->Suc = s2;
        }
    }
    while ((s1 = s2) != FirstSegment);
}
