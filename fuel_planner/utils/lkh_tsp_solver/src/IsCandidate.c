#include "LKH.h"

/* 
 * The IsCandidate function is used to test if an edge, (ta,tb), 
 * belongs to the set of candidate edges.
 *
 * If the edge is a candidate edge the function returns 1; otherwise 0.
 */

int IsCandidate(const Node * ta, const Node * tb)
{
    Candidate *Nta;

    for (Nta = ta->CandidateSet; Nta && Nta->To; Nta++)
        if (Nta->To == tb)
            return 1;
    return 0;
}
