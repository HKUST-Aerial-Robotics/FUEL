#include "LKH.h"

/*
   The Between_SSL function is used to determine whether a segment is 
   between two other segments with respect to the current orientation. 
   The function is only used if the three-level tree representation 
   is used for a tour.
   	
   Between_SSL(a,b,c) returns 1 if segment b is between segment a and c.
   Otherwise, 0 is returned.
   	
   The function is called from the functions BestMove, Gain23,
   BridgeGain, Make4OptMove and Make5OptMove.
*/

int Between_SSL(const Node * ta, const Node * tb, const Node * tc)
{
    const Segment *Pa, *Pb, *Pc;
    const SSegment *PPa, *PPb, *PPc;

    if (tb == ta || tb == tc)
        return 1;
    if (ta == tc)
        return 0;
    Pa = ta->Parent;
    Pb = tb->Parent;
    Pc = tc->Parent;
    PPa = Pa->Parent;
    PPb = Pb->Parent;
    PPc = Pc->Parent;
    if (Pa == Pc) {
        if (Pb == Pa)
            return (Reversed == (Pa->Reversed != PPa->Reversed)) ==
                (ta->Rank < tc->Rank ?
                 tb->Rank > ta->Rank && tb->Rank < tc->Rank :
                 tb->Rank > ta->Rank || tb->Rank < tc->Rank);
        return (Reversed == (Pa->Reversed != PPa->Reversed)) == (ta->Rank >
                                                                 tc->Rank);
    }
    if (Pb == Pc)
        return (Reversed == (Pb->Reversed != PPb->Reversed)) == (tb->Rank <
                                                                 tc->Rank);
    if (Pa == Pb)
        return (Reversed == (Pa->Reversed != PPa->Reversed)) == (ta->Rank <
                                                                 tb->Rank);
    if (PPa == PPc) {
        if (PPb == PPa)
            return (Reversed == PPa->Reversed) ==
                (Pa->Rank < Pc->Rank ?
                 Pb->Rank > Pa->Rank && Pb->Rank < Pc->Rank :
                 Pb->Rank > Pa->Rank || Pb->Rank < Pc->Rank);
        return (Reversed == PPa->Reversed) == (Pa->Rank > Pc->Rank);
    }
    if (PPb == PPc)
        return (Reversed == PPb->Reversed) == (Pb->Rank < Pc->Rank);
    if (PPa == PPb)
        return (Reversed == PPa->Reversed) == (Pa->Rank < Pb->Rank);
    return Reversed !=
        (PPa->Rank < PPc->Rank ?
         PPb->Rank > PPa->Rank && PPb->Rank < PPc->Rank :
         PPb->Rank > PPa->Rank || PPb->Rank < PPc->Rank);
}
