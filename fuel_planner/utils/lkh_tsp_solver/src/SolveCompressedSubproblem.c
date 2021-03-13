#include "LKH.h"

/* 
 * The SolveCompressedSubproblem compresses and solves a given subproblem. 
 * The subproblem is identified by the parameter CurrentSuproblem and 
 * contains all nodes with Subproblem equal to CurrentSubproblem.   
 *  
 * The parameter Subproblems specifies the number of subproblems.
 * The parameter GlobalBestCost references the current best cost of the 
 * whole problem.
 */

#define MaxLevel 10

void
SolveCompressedSubproblem(int CurrentSubproblem, int Subproblems,
                          GainType * GlobalBestCost)
{
    int Level, Number, RestrictedSearchSaved = RestrictedSearch;
    GainType OldGlobalBestCost = *GlobalBestCost;
    Node *N;

    if ((Number = CurrentSubproblem % Subproblems) == 0)
        Number = Subproblems;
    RestrictedSearch = 0;
    for (Level = 1;
         Level <= MaxLevel && *GlobalBestCost == OldGlobalBestCost;
         Level++) {
        if (TraceLevel >= 1)
            printff("\nCompress subproblem %d (Level %d):", Number, Level);
        if (!SolveSubproblem
            (CurrentSubproblem, Subproblems, GlobalBestCost))
            break;
    }
    RestrictedSearch = RestrictedSearchSaved;
    N = FirstNode;
    do
        N->Subproblem = abs(N->Subproblem);
    while ((N = N->Suc) != FirstNode);
}
