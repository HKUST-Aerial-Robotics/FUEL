#include "LKH.h"

/* 
 * The SymmetrizeCandidateSet function complements the candidate set such 
 * that every candidate edge is associated with both its two end nodes. 
*/

void SymmetrizeCandidateSet()
{
    Node *From, *To;
    Candidate *NFrom;

    From = FirstNode;
    do {
        for (NFrom = From->CandidateSet; NFrom && (To = NFrom->To);
             NFrom++)
            AddCandidate(To, From, NFrom->Cost, NFrom->Alpha);
    }
    while ((From = From->Suc) != FirstNode);
    ResetCandidateSet();
}
