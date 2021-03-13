#include "LKH.h"

/*
 * The IsPossibleCandidate function is used to test if an edge, (From,To),
 * may be a solution edge together with all fixed or common edges.
 *
 * If the edge is possible, the function returns 1; otherwise 0.
 */

int IsPossibleCandidate(Node * From, Node * To)
{
    Node *Na, *Nb, *Nc, *N;

    if (Forbidden(From, To))
        return 0;
    if (InInitialTour(From, To) ||
        From->SubproblemSuc == To || To->SubproblemSuc == From ||
        FixedOrCommon(From, To))
        return 1;
    if (From->FixedTo2 || To->FixedTo2)
        return 0;
    if (!IsCandidate(From, To) &&
        (FixedOrCommonCandidates(From) == 2 ||
         FixedOrCommonCandidates(To) == 2))
        return 0;
    if (MergeTourFiles < 2)
        return 1;
    if (!From->Head) {
        Na = FirstNode;
        do
            Na->Head = Na->Tail = Na;
        while ((Na = Na->Suc) != FirstNode);
        Na = FirstNode;
        while ((Nb = Na->MergeSuc[0]) != FirstNode
               && FixedOrCommon(Na, Nb))
            Na = Nb;
        if (Nb != FirstNode) {
            N = Nb;
            do {
                Nc = Nb;
                do {
                    Na = Nb;
                    Na->Head = Nc;
                    Nb = Na->MergeSuc[0];
                } while (FixedOrCommon(Na, Nb));
                do
                    Nc->Tail = Na;
                while (Nc != N && (Nc = Nc->MergeSuc[0]) != Nb);
            } while (Nc != N);
        } else {
            do
                Nb->Head = Nb->Tail = FirstNode;
            while ((Nb = Nb->Suc) != FirstNode);
        }
    }
    if (From->Head == To->Head ||
        (From->Head != From && From->Tail != From) ||
        (To->Head != To && To->Tail != To))
        return 0;
    return 1;
}
