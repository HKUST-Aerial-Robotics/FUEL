#include "LKH.h"

/*
 * The AddCandidate function adds a given edge (From, To) to the set
 * of candidate edges associated with the node From. The cost and 
 * alpha-value of the edge are passed as parameters to the function.
 *
 * The function has no effect if the edge is already in the candidate
 * set.
 *
 * If the edge was added, the function returns 1; otherwise 0.
 *    
 * The function is called from the functions CreateDelaunaySet and
 * OrderCandidateSet.
 */

int AddCandidate(Node * From, Node * To, int Cost, int Alpha)
{
    int Count;
    Candidate *NFrom;

    if (From->Subproblem != FirstNode->Subproblem)
        return 0;
    if (From->CandidateSet == 0)
        From->CandidateSet = (Candidate *) calloc(3, sizeof(Candidate));
    if (From == To || To->Subproblem != FirstNode->Subproblem ||
        !IsPossibleCandidate(From, To))
        return 0;
    Count = 0;
    for (NFrom = From->CandidateSet; NFrom->To && NFrom->To != To; NFrom++)
        Count++;
    if (NFrom->To) {
        if (NFrom->Alpha == INT_MAX)
            NFrom->Alpha = Alpha;
        return 0;
    }
    NFrom->Cost = Cost;
    NFrom->Alpha = Alpha;
    NFrom->To = To;
    From->CandidateSet =
        (Candidate *) realloc(From->CandidateSet,
                              (Count + 2) * sizeof(Candidate));
    From->CandidateSet[Count + 1].To = 0;
    return 1;
}
