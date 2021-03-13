#include "LKH.h"

/*
 * The TrimCandidateSet function takes care that each node has 
 * associated at most MaxCandidates candidate edges.                         
 */

void TrimCandidateSet(int MaxCandidates)
{
    Node *From;
    Candidate *NFrom;
    int Count;

    From = FirstNode;
    do {
        Count = 0;
        for (NFrom = From->CandidateSet; NFrom && NFrom->To; NFrom++)
            Count++;
        if (Count > MaxCandidates) {
            From->CandidateSet =
               (Candidate *) realloc(From->CandidateSet,
                                     (MaxCandidates + 1) * sizeof(Candidate));
            From->CandidateSet[MaxCandidates].To = 0;
        }
    } while ((From = From->Suc) != FirstNode);
}
