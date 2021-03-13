#include "LKH.h"

/*
 * The AddTourCandidates function extends the candidate set with tour
 * edges given in the tour files.
 *   
 * The function is called from GenerateCandidateSet and OrderCandidateSet.  
*/

void AddTourCandidates()
{
    Node *Na, *Nb;
    int i, d, Subproblem = FirstNode->Subproblem;

    /* Add fixed edges */
    Na = FirstNode;
    do {
        if (Na->FixedTo1)
            AddCandidate(Na, Na->FixedTo1, D(Na, Na->FixedTo1), 0);
        if (Na->FixedTo2)
            AddCandidate(Na, Na->FixedTo2, D(Na, Na->FixedTo2), 0);
    }
    while ((Na = Na->Suc) != FirstNode);

    /* Add MERGE_TOUR_FILE edges */
    for (i = 0; i < MergeTourFiles; i++) {
        Na = FirstNode;
        do {
            Nb = Na->MergeSuc[i];
            if (!Nb)
                break;
            if (Na->Subproblem == Subproblem &&
                Nb->Subproblem == Subproblem) {
                d = D(Na, Nb);
                AddCandidate(Na, Nb, d, 1);
                AddCandidate(Nb, Na, d, 1);
            }
        }
        while ((Na = Nb) != FirstNode);
    }

    /* Add INITIAL_TOUR_FILE edges */
    Na = FirstNode;
    do {
        Nb = Na->InitialSuc;
        if (!Nb)
            break;
        if (Na->Subproblem == Subproblem && Nb->Subproblem == Subproblem) {
            d = D(Na, Nb);
            AddCandidate(Na, Nb, d, 1);
            AddCandidate(Nb, Na, d, 1);
        }
    }
    while ((Na = Nb) != FirstNode);

    /* Add INPUT_TOUR_FILE edges */
    Na = FirstNode;
    do {
        Nb = Na->InputSuc;
        if (!Nb)
            break;
        if (Na->Subproblem == Subproblem && Nb->Subproblem == Subproblem) {
            d = D(Na, Nb);
            AddCandidate(Na, Nb, d, 1);
            AddCandidate(Nb, Na, d, 1);
        }
    }
    while ((Na = Nb) != FirstNode);

    /* Add SUBPROBLEM_TOUR_FILE edges */
    Na = FirstNode;
    do {
        Nb = Na->SubproblemSuc;
        if (!Nb)
            break;
        if (Na->Subproblem == Subproblem && Nb->Subproblem == Subproblem) {
            d = D(Na, Nb);
            AddCandidate(Na, Nb, d, 1);
            AddCandidate(Nb, Na, d, 1);
        }
    } while ((Na = Nb) != FirstNode);
}
