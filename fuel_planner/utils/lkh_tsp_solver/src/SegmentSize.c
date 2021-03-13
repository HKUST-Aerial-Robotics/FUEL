#include "Segment.h"
#include "LKH.h"

/*
 * The SegmentSize function returns the number of nodes in the 
 * tour segment between two given nodes in the current direction. 
 * Note, however, that if the two-level or three-level tree is used,
 * the number of nodes is only approximate (for efficiency reasons).
 * 
 * Time complexity: O(1).
 */

#ifdef ONE_LEVEL_TREE

int SegmentSize(Node * ta, Node * tb)
{
    int n = !Reversed ? tb->Rank - ta->Rank : ta->Rank - tb->Rank;
    return (n < 0 ? n + Dimension : n) + 1;
}

#elif defined TWO_LEVEL_TREE

int SegmentSize(Node * ta, Node * tb)
{
    Segment *Pa, *Pb;
    int nLeft, nMid, nRight;

    Pa = ta->Parent;
    Pb = tb->Parent;
    if (Pa == Pb) {
        int n = Reversed == Pa->Reversed ? tb->Rank - ta->Rank :
            ta->Rank - tb->Rank;
        return (n < 0 ? n + Dimension : n) + 1;
    }
    nLeft =
        Reversed ==
        Pa->Reversed ? Pa->Last->Rank - ta->Rank : ta->Rank -
        Pa->First->Rank;
    if (nLeft < 0)
        nLeft += Pa->Size;
    nMid = !Reversed ? Pb->Rank - Pa->Rank : Pa->Rank - Pb->Rank;
    if (nMid < 0)
        nMid += Groups;
    nMid = nMid == 2 ? (!Reversed ? Pa->Suc : Pa->Pred)->Size
        : (nMid - 1) * GroupSize;
    nRight =
        Reversed ==
        Pb->Reversed ? tb->Rank -
        Pb->First->Rank : Pb->Last->Rank - tb->Rank;
    if (nRight < 0)
        nRight += Pb->Size;
    return nLeft + nMid + nRight + 2;
}

#elif defined  THREE_LEVEL_TREE

int SegmentSize(Node * ta, Node * tb)
{
    Segment *Pa, *Pb;
    SSegment *PPa, *PPb;
    int n, nLeft, nMid, nRight;

    Pa = ta->Parent;
    Pb = tb->Parent;
    PPa = Pa->Parent;
    PPb = Pb->Parent;
    if (Pa == Pb) {
        n = Reversed == (Pa->Reversed != PPa->Reversed) ?
            tb->Rank - ta->Rank : ta->Rank - tb->Rank;
        if (n < 0)
            n += Dimension;
    } else if (PPa == PPb) {
        nLeft =
            Reversed == (Pa->Reversed != PPa->Reversed) ?
            Pa->Last->Rank - ta->Rank : ta->Rank - Pa->First->Rank;
        if (nLeft < 0)
            nLeft += Pa->Size;
        nMid =
            Reversed == PPa->Reversed ?
            Pb->Rank - Pa->Rank : Pa->Rank - Pb->Rank;
        if (nMid < 0)
            nMid += Groups;
        nMid = nMid == 2 ?
            (Reversed == PPa->Reversed ? Pa->Suc : Pa->Pred)->Size
            : (nMid - 1) * GroupSize;
        nRight =
            (Reversed != PPa->Reversed) ==
            (Pb->Reversed != PPb->Reversed) ? tb->Rank -
            Pb->First->Rank : Pb->Last->Rank - tb->Rank;
        if (nRight < 0)
            nRight += Pb->Size;
        n = nLeft + nMid + nRight + 1;
    } else {
        nLeft =
            Reversed == PPa->Reversed ?
            PPa->Last->Rank - Pa->Rank : Pa->Rank - PPa->First->Rank;
        if (nLeft < 0)
            nLeft += PPa->Size;
        if (nLeft > 0)
            nLeft *= GroupSize;
        else {
            nLeft =
                Reversed == (Pa->Reversed != PPa->Reversed) ?
                Pa->Last->Rank - ta->Rank : ta->Rank - Pa->First->Rank;
            if (nLeft < 0)
                nLeft += Pa->Size;
        }
        nMid = !Reversed ? PPb->Rank - PPa->Rank : PPa->Rank - PPb->Rank;
        if (nMid < 0)
            nMid += SGroups;
        nMid = nMid == 2 ?
            (!Reversed ? PPa->Suc : PPa->Pred)->Size
            : (nMid - 1) * SGroupSize;
        nMid *= GroupSize;
        nRight =
            Reversed == PPb->Reversed ? Pb->Rank -
            PPb->First->Rank : PPb->Last->Rank - Pb->Rank;
        if (nRight < 0)
            nRight += PPb->Size;
        if (nRight > 0)
            nRight *= GroupSize;
        else {
            nRight =
                (Reversed != PPa->Reversed) ==
                (Pb->Reversed != PPb->Reversed) ? tb->Rank -
                Pb->First->Rank : Pb->Last->Rank - tb->Rank;
            if (nRight < 0)
                nRight += Pb->Size;
        }
        n = nLeft + nMid + nRight + 1;
    }
    return n + 1;
}

#endif
