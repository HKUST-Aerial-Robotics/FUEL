#include "LKH.h"

/*
 * The WriteCandidates function writes the candidate edges to file
 * CandidateFileName[0].
 *
 * The first line of the file contains the number of nodes.
 *
 * Each of the follwong lines contains a node number, the number of the
 * dad of the node in the minimum spanning tree (0, if the node has no dad), 
 * the number of candidate edges emanating from the node, followed by the 
 * candidate edges. For each candidate edge its end node number and 
 * alpha-value are given.
 *
 * The function is called from the CreateCandidateSet function. 
 */

void WriteCandidates()
{
    FILE *CandidateFile;
    int i, Count;
    Candidate *NN;
    Node *N;

    if (CandidateFiles == 0 ||
        !(CandidateFile = fopen(CandidateFileName[0], "w")))
        return;
    if (TraceLevel >= 1)
        printff("Writing CANDIDATE_FILE: \"%s\" ... ",
                CandidateFileName[0]);
    fprintf(CandidateFile, "%d\n", Dimension);
    for (i = 1; i <= Dimension; i++) {
        N = &NodeSet[i];
        fprintf(CandidateFile, "%d %d", N->Id, N->Dad ? N->Dad->Id : 0);
        Count = 0;
        for (NN = N->CandidateSet; NN && NN->To; NN++)
            Count++;
        fprintf(CandidateFile, " %d ", Count);
        for (NN = N->CandidateSet; NN && NN->To; NN++)
            fprintf(CandidateFile, "%d %d ", NN->To->Id, NN->Alpha);
        fprintf(CandidateFile, "\n");
    }
    fprintf(CandidateFile, "-1\nEOF\n");
    fclose(CandidateFile);
    if (TraceLevel >= 1)
        ;
}
