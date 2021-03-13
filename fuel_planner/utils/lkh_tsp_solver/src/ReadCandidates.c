#include "LKH.h"

/*
 * The ReadCandidates function attempts to read candidate edges from 
 * files. The candidate edges are added to the current candidate sets.
 *
 * The first line of the file contains the number of nodes.
 *
 * Each of the follwong lines contains a node number, the number of the
 * dad of the node in the minimum spanning tree (0, if the node has no dad), 
 * the number of candidate edges emanating from the node, followed by the 
 * candidate edges. For each candidate edge its end node number and 
 * alpha-value are given.
 *
 * The parameter MaxCandidates specifies the maximum number of candidate edges 
 * allowed for each node.
 *
 * If reading succeeds, the function returns 1; otherwise 0. 
 *
 * The function is called from the CreateCandidateSet function. 
 */

int ReadCandidates(int MaxCandidates)
{
    FILE *CandidateFile = 0;
    Node *From, *To;
    int Dimension, i, f, Id, Alpha, Count;

    if (CandidateFiles == 0 ||
        (CandidateFiles == 1 &&
         !(CandidateFile = fopen(CandidateFileName[0], "r"))))
        return 0;
    Dimension = ProblemType != ATSP ? DimensionSaved : 2 * DimensionSaved;
    for (f = 0; f < CandidateFiles; f++) {
        if (CandidateFiles >= 2 &&
            !(CandidateFile = fopen(CandidateFileName[f], "r")))
            eprintf("Cannot open CANDIDATE_FILE: \"%s\"",
                    CandidateFileName[f]);
        if (TraceLevel >= 1)
            printff("Reading CANDIDATE_FILE: \"%s\" ... ",
                    CandidateFileName[f]);
        fscanint(CandidateFile, &i);
        if (i != Dimension)
            eprintf("CANDIDATE_FILE \"%s\" does not match problem",
                    CandidateFileName[f]);
        while (fscanint(CandidateFile, &Id) == 1 && Id != -1) {
            assert(Id >= 1 && Id <= Dimension);
            From = &NodeSet[Id];
            fscanint(CandidateFile, &Id);
            assert(Id >= 0 && Id <= Dimension);
            if (Id > 0)
                From->Dad = &NodeSet[Id];
            assert(From != From->Dad);
            fscanint(CandidateFile, &Count);
            assert(Count >= 0 && Count < Dimension);
            if (!From->CandidateSet)
                From->CandidateSet =
                   (Candidate *) calloc(Count + 1, sizeof(Candidate));
            for (i = 0; i < Count; i++) {
                fscanint(CandidateFile, &Id);
                assert(Id >= 1 && Id <= Dimension);
                To = &NodeSet[Id];
                fscanint(CandidateFile, &Alpha);
                AddCandidate(From, To, D(From, To), Alpha);
            }
        }
        fclose(CandidateFile);
        if (TraceLevel >= 1)
            ;
    }
    ResetCandidateSet();
    if (MaxCandidates > 0)
        TrimCandidateSet(MaxCandidates);
    return 1;
}
