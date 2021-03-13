#include "LKH.h"

/*
 * The Between_SL function is used to determine whether a node is 
 * between two other nodes with respect to the current orientation. 
 * The function is only used if the two-level tree representation 
 * is used for a tour; if the doubly linked list representation is 
 * used, the function Between is used instead.
 * 	
 * Between_SL(a,b,c) returns 1 if node b is between node a and c.
 * Otherwise, 0 is returned.
 * 	
 * The function is called from the functions BestMove, Gain23,
 * BridgeGain, Make4OptMove, Make5OptMove and FindPermutation.
 */

int Between_SL(const Node * ta, const Node * tb, const Node * tc)
{
    const Segment *Pa, *Pb, *Pc;

    if (tb == ta || tb == tc)
        return 1;
    if (ta == tc)
        return 0;
    Pa = ta->Parent;
    Pb = tb->Parent;
    Pc = tc->Parent;
    if (Pa == Pc) {
        if (Pb == Pa)
            return (Reversed == Pa->Reversed) ==
                (ta->Rank < tc->Rank ?
                 tb->Rank > ta->Rank && tb->Rank < tc->Rank :
                 tb->Rank > ta->Rank || tb->Rank < tc->Rank);
        return (Reversed == Pa->Reversed) == (ta->Rank > tc->Rank);
    }
    if (Pb == Pc)
        return (Reversed == Pb->Reversed) == (tb->Rank < tc->Rank);
    if (Pa == Pb)
        return (Reversed == Pa->Reversed) == (ta->Rank < tb->Rank);
    return Reversed !=
        (Pa->Rank < Pc->Rank ?
         Pb->Rank > Pa->Rank && Pb->Rank < Pc->Rank :
         Pb->Rank > Pa->Rank || Pb->Rank < Pc->Rank);
}
