#include "LKH.h"

/*
 * The SolveSFCSubproblems function attempts to improve a given tour 
 * by means of partitioning based on a space-filling curve, either a 
 * Sierpinski curve or a Moore curve. If an improvement is found, 
 * the new tour is written to TourFile.
 * 
 * The tour is divided into segments of equal size. Each segment 
 * constitutes a subproblem to be solved by the LinKernighan function. 
 * 
 * The original tour is given by the SubproblemSuc references of the nodes. 
 * The size of each segment is SubproblemSize.
 *
 * The function runs in two rounds. In the first round the first segment 
 * starts at FirstNode. In the second round the middle of this segment is 
 * used as a starting point for the first segment. 
 */

void SolveSFCSubproblems()
{
    Node *FirstNodeSaved, *N;
    int CurrentSubproblem, Subproblems, Round, i;
    GainType GlobalBestCost, OldGlobalBestCost;
    Node **Suc;
    double EntryTime = GetTime();

    SFCTour(SierpinskiPartitioning ? SIERPINSKI : MOORE);
    Suc = (Node **) malloc((1 + Dimension) * sizeof(Node *));
    N = FirstNode;
    do
        Suc[N->Id] = N->Suc;
    while ((N = N->Suc) != FirstNode);
    AllocateStructures();
    Subproblems = (int) ceil((double) Dimension / SubproblemSize);
    ReadPenalties();
    FirstNode = &NodeSet[Random() % Dimension + 1];

    /* Compute upper bound for the original problem */
    GlobalBestCost = 0;
    N = FirstNodeSaved = FirstNode;
    do {
        if (!Fixed(N, N->SubproblemSuc))
            GlobalBestCost += Distance(N, N->SubproblemSuc);
        N->Subproblem = 0;
    }
    while ((N = N->SubproblemSuc) != FirstNode);
    for (Round = 1; Round <= 2; Round++) {
        if (Round == 2 && Subproblems == 1)
            break;
        if (TraceLevel >= 1) {
            if (Round == 2 || TraceLevel >= 2)
                printff("\n");
            printff
                ("*** %s partitioning *** [Round %d of %d, Cost = "
                 GainFormat "]\n",
                 SierpinskiPartitioning ? "Sierpinski" : "Moore",
                 Round, Subproblems > 1 ? 2 : 1, GlobalBestCost);
        }
        FirstNode = FirstNodeSaved;
        if (Round == 2)
            for (i = SubproblemSize / 2; i > 0; i--)
                FirstNode = Suc[FirstNode->Id];
        for (CurrentSubproblem = 1;
             CurrentSubproblem <= Subproblems; CurrentSubproblem++) {
            for (i = 0, N = FirstNode; i < SubproblemSize;
                 i++, N = Suc[N->Id]) {
                N->Subproblem =
                    (Round - 1) * Subproblems + CurrentSubproblem;
                N->FixedTo1Saved = N->FixedTo2Saved = 0;
                N->SubBestPred = N->SubBestSuc = 0;
            }
            OldGlobalBestCost = GlobalBestCost;
            SolveSubproblem((Round - 1) * Subproblems + CurrentSubproblem,
                            Subproblems, &GlobalBestCost);
            if (SubproblemsCompressed
                && GlobalBestCost == OldGlobalBestCost)
                SolveCompressedSubproblem((Round - 1) * Subproblems +
                                          CurrentSubproblem, Subproblems,
                                          &GlobalBestCost);
            FirstNode = N;
        }
    }
    free(Suc);
    printff("\nCost = " GainFormat, GlobalBestCost);
    if (Optimum != MINUS_INFINITY && Optimum != 0)
        printff(", Gap = %0.4f%%",
                100.0 * (GlobalBestCost - Optimum) / Optimum);
    printff(", Time = %0.2f sec. %s\n", fabs(GetTime() - EntryTime),
            GlobalBestCost < Optimum ? "<" : GlobalBestCost ==
            Optimum ? "=" : "");
    if (SubproblemBorders && Subproblems > 1)
        SolveSubproblemBorderProblems(Subproblems, &GlobalBestCost);
}
