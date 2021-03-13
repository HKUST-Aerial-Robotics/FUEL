#include "LKH.h"

/*
 * When a trial has produced a better tour, the set of candidate edges is
 * adjusted as follows:
 *
 *   (1) The set is extended with tour edges not present in the
 *       current set;
 *   (2) Precedence is given to those edges that are common to the two
 *       currently best tours.
 *
 * The AdjustCandidateSet function adjusts for each node its table of 
 * candidate edges. A new candidate edge is added by extending the table 
 * and inserting the edge as its last ordinary element (disregarding the 
 * dummy edge). The Alpha field of the new candidate edge is set to 
 * INT_MAX. Edges that belong to the best tour as well as the next best 
 * tour are moved to the start of the table.                         
 */

void AdjustCandidateSet()
{
    Candidate *NFrom, *NN, Temp;
    Node *From = FirstNode, *To;

    /* Extend and reorder candidate sets */
    do {
        if (!From->CandidateSet)
            From->CandidateSet = (Candidate *) calloc(3, sizeof(Candidate));
        /* Extend */
        for (To = From->Pred; To; To = To == From->Pred ? From->Suc : 0) {
            int Count = 0;
            if ((ProblemType == HCP || ProblemType == HPP) &&
                !IsBackboneCandidate(From, To))
                continue;
            for (NFrom = From->CandidateSet; NFrom->To && NFrom->To != To;
                 NFrom++)
                Count++;
            if (!NFrom->To) {
                /* Add new candidate edge */
                NFrom->Cost = C(From, To);
                NFrom->To = To;
                NFrom->Alpha = INT_MAX;
                From->CandidateSet =
                   (Candidate *) realloc(From->CandidateSet,
                                         (Count + 2) * sizeof(Candidate));
                From->CandidateSet[Count + 1].To = 0;
            }
        }
        /* Reorder */
        for (NFrom = From->CandidateSet + 1; (To = NFrom->To); NFrom++)
            if (InBestTour(From, To) &&
                (InNextBestTour(From, To) || InInitialTour(From, To))) {
                /* Move the edge to the start of the candidate table */
                Temp = *NFrom;
                for (NN = NFrom - 1; NN >= From->CandidateSet; NN--)
                    *(NN + 1) = *NN;
                *(NN + 1) = Temp;
            }
    }
    while ((From = From->Suc) != FirstNode);
}
