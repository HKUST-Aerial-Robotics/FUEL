#include "LKH.h"
#include "GeoConversion.h"

/*
 * The SolveKarpSubproblems function attempts to improve a given tour 
 * by means of Karp's partitioning scheme. 
 *
 * The overall region containing the nodes is subdivided into rectangles
 * with SubproblemSize nodes in each rectangle. Each rectangle together 
 * with the given tour induces a subproblem consisting of all nodes inside 
 * the rectangle, and with edges fixed between nodes that are connected 
 * by tour segments whose interior points are outside the rectangle.  
 *  
 * If an improvement is found, the new tour is written to TourFile. 
 * The original tour is given by the SubproblemSuc references of the nodes.
 */

static void KarpPartition(int start, int end);
static void CalculateSubproblems(int start, int end);

static Node **KDTree;
static GainType GlobalBestCost, OldGlobalBestCost;
static int CurrentSubproblem, Subproblems;

void SolveKarpSubproblems()
{
    Node *N;
    double EntryTime = GetTime();

    AllocateStructures();
    ReadPenalties();

    /* Compute upper bound for the original problem */
    GlobalBestCost = 0;
    N = FirstNode;
    do {
        if (!Fixed(N, N->SubproblemSuc))
            GlobalBestCost += Distance(N, N->SubproblemSuc);
        N->Subproblem = 0;
    }
    while ((N = N->SubproblemSuc) != FirstNode);
    if (TraceLevel >= 1) {
        if (TraceLevel >= 2)
            printff("\n");
        printff("*** Karp partitioning *** [Cost = " GainFormat "]\n",
                GlobalBestCost);
    }
    if (WeightType == GEO || WeightType == GEOM ||
        WeightType == GEO_MEEUS || WeightType == GEOM_MEEUS) {
        N = FirstNode;
        do {
            N->Xc = N->X;
            N->Yc = N->Y;
            N->Zc = N->Z;
            if (WeightType == GEO || WeightType == GEO_MEEUS)
                GEO2XYZ(N->Xc, N->Yc, &N->X, &N->Y, &N->Z);
            else
                GEOM2XYZ(N->Xc, N->Yc, &N->X, &N->Y, &N->Z);
        } while ((N = N->SubproblemSuc) != FirstNode);
        CoordType = THREED_COORDS;
    }
    KDTree = BuildKDTree(SubproblemSize);
    if (WeightType == GEO || WeightType == GEOM ||
        WeightType == GEO_MEEUS || WeightType == GEOM_MEEUS) {
        N = FirstNode;
        do {
            N->X = N->Xc;
            N->Y = N->Yc;
            N->Z = N->Zc;
        } while ((N = N->SubproblemSuc) != FirstNode);
        CoordType = TWOD_COORDS;
    }

    Subproblems = 0;
    CalculateSubproblems(0, Dimension - 1);
    CurrentSubproblem = 0;
    KarpPartition(0, Dimension - 1);
    free(KDTree);
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

/*
 * The KarpPartition function subidivides the overall region into 
 * rectangles and attempts to solve the induced subproblems. 
 */

static void KarpPartition(int start, int end)
{
    if (end - start + 1 <= SubproblemSize) {
        int i;
        CurrentSubproblem++;
        for (i = start; i <= end; i++)
            KDTree[i]->Subproblem = CurrentSubproblem;
        OldGlobalBestCost = GlobalBestCost;
        SolveSubproblem(CurrentSubproblem, Subproblems, &GlobalBestCost);
        if (SubproblemsCompressed && GlobalBestCost == OldGlobalBestCost)
            SolveCompressedSubproblem(CurrentSubproblem, Subproblems,
                                      &GlobalBestCost);
    } else {
        int mid = (start + end) / 2;
        KarpPartition(start, mid);
        KarpPartition(mid + 1, end);
    }
}

/*
 * The CalculateSubproblems function is used to calculate the number of 
 * subproblems (Subproblems) created by the KarpPartition function.
 */

static void CalculateSubproblems(int start, int end)
{
    if (end - start + 1 <= SubproblemSize)
        Subproblems++;
    else {
        int mid = (start + end) / 2;
        CalculateSubproblems(start, mid);
        CalculateSubproblems(mid + 1, end);
    }
}
