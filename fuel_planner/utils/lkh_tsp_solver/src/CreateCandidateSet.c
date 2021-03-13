#include "LKH.h"

/*
 * The CreateCandidateSet function determines for each node its set of incident
 * candidate edges.
 *
 * The Ascent function is called to determine a lower bound on the optimal tour 
 * using subgradient optimization. But only if the penalties (the Pi-values) is
 * not available on file. In the latter case, the penalties is read from the 
 * file, and the lower bound is computed from a minimum 1-tree.      
 *
 * The function GenerateCandidates is called to compute the Alpha-values and to 
 * associate to each node a set of incident candidate edges.  
 *
 * The CreateCandidateSet function itself is called from LKHmain.
 */

void CreateCandidateSet()
{
    GainType Cost, MaxAlpha, A;
    Node *Na;
    int CandidatesRead = 0, i;
    double EntryTime = GetTime();

    Norm = 9999;
    if (C == C_EXPLICIT) {
        Na = FirstNode;
        do {
            for (i = 1; i < Na->Id; i++)
                Na->C[i] *= Precision;
        }
        while ((Na = Na->Suc) != FirstNode);
    }
    if (Distance == Distance_1 ||
        (MaxTrials == 0 &&
         (FirstNode->InitialSuc || InitialTourAlgorithm == SIERPINSKI ||
          InitialTourAlgorithm == MOORE))) {
        CandidatesRead = ReadCandidates(MaxCandidates) ||
            ReadEdges(MaxCandidates);
        AddTourCandidates();
        if (ProblemType == HCP || ProblemType == HPP)
            Ascent();
        goto End_CreateCandidateSet;
    }
    if (TraceLevel >= 2)
        printff("Creating candidates ...\n");
    if (MaxCandidates > 0 &&
        (CandidateSetType == QUADRANT || CandidateSetType == NN)) {
        ReadPenalties();
        if (!(CandidatesRead = ReadCandidates(MaxCandidates) ||
              ReadEdges(MaxCandidates)) && MaxCandidates > 0) {
            if (CandidateSetType == QUADRANT)
                CreateQuadrantCandidateSet(MaxCandidates);
            else if (CandidateSetType == NN) {
                if ((CoordType == TWOD_COORDS
                     && Distance != Distance_TOR_2D)
                    || (CoordType == THREED_COORDS
                        && Distance != Distance_TOR_3D))
                    CreateNearestNeighborCandidateSet(MaxCandidates);
                else
                    CreateNNCandidateSet(MaxCandidates);
            }
        }
        AddTourCandidates();
        if (CandidateSetSymmetric)
            SymmetrizeCandidateSet();
        goto End_CreateCandidateSet;
    }
    if (!ReadPenalties()) {
        /* No PiFile specified or available */
        Na = FirstNode;
        do
            Na->Pi = 0;
        while ((Na = Na->Suc) != FirstNode);
        CandidatesRead = ReadCandidates(MaxCandidates) ||
            ReadEdges(MaxCandidates);
        Cost = Ascent();
        if (Subgradient && SubproblemSize == 0) {
            WritePenalties();
            PiFile = 0;
        }
    } else if ((CandidatesRead = ReadCandidates(MaxCandidates) ||
                ReadEdges(MaxCandidates)) || MaxCandidates == 0) {
        AddTourCandidates();
        if (CandidateSetSymmetric)
            SymmetrizeCandidateSet();
        goto End_CreateCandidateSet;
    } else {
        if (CandidateSetType != DELAUNAY &&
            CandidateSetType != POPMUSIC &&
            MaxCandidates > 0) {
            if (TraceLevel >= 2)
                printff("Computing lower bound ... ");
            Cost = Minimum1TreeCost(0);
            if (TraceLevel >= 2)
                ;
        } else {
            if (CandidateSetType == DELAUNAY)
                CreateDelaunayCandidateSet();
            else
                Create_POPMUSIC_CandidateSet(AscentCandidates);
            Na = FirstNode;
            do {
                Na->BestPi = Na->Pi;
                Na->Pi = 0;
            }
            while ((Na = Na->Suc) != FirstNode);
            if (TraceLevel >= 2)
                printff("Computing lower bound ... ");
            Cost = Minimum1TreeCost(1);
            if (TraceLevel >= 2)
                ;
            Na = FirstNode;
            do {
                Na->Pi = Na->BestPi;
                Cost -= 2 * Na->Pi;
            }
            while ((Na = Na->Suc) != FirstNode);
        }
    }
    LowerBound = (double) Cost / Precision;
    if (TraceLevel >= 1) {
        // printff("Lower bound = %0.1f", LowerBound);
        if (Optimum != MINUS_INFINITY && Optimum != 0)
            printff(", Gap = %0.2f%%",
                    100.0 * (Optimum - LowerBound) / Optimum);
        if (!PiFile);
            // printff(", Ascent time = %0.2f sec.",
            //         fabs(GetTime() - EntryTime));
        printff("\n");
    }
    MaxAlpha = (GainType) fabs(Excess * Cost);
    if ((A = Optimum * Precision - Cost) > 0 && A < MaxAlpha)
        MaxAlpha = A;
    if (CandidateSetType == DELAUNAY ||
        CandidateSetType == POPMUSIC ||
        MaxCandidates == 0)
        OrderCandidateSet(MaxCandidates, MaxAlpha, CandidateSetSymmetric);
    else
        GenerateCandidates(MaxCandidates, MaxAlpha, CandidateSetSymmetric);

  End_CreateCandidateSet:
    if (ExtraCandidates > 0) {
        AddExtraCandidates(ExtraCandidates,
                           ExtraCandidateSetType,
                           ExtraCandidateSetSymmetric);
        AddTourCandidates();
    }
    ResetCandidateSet();
    if (MaxTrials > 0 ||
        (InitialTourAlgorithm != SIERPINSKI &&
         InitialTourAlgorithm != MOORE)) {
        Na = FirstNode;
        do {
            if (!Na->CandidateSet || !Na->CandidateSet[0].To) {
                if (MaxCandidates == 0)
                    eprintf
                        ("MAX_CANDIDATES = 0: Node %d has no candidates",
                         Na->Id);
                else
                    eprintf("Node %d has no candidates", Na->Id);
            }
        }
        while ((Na = Na->Suc) != FirstNode);
        if (!CandidatesRead && SubproblemSize == 0)
            WriteCandidates();
    }
    if (C == C_EXPLICIT) {
        Na = FirstNode;
        do
            for (i = 1; i < Na->Id; i++)
                Na->C[i] += Na->Pi + NodeSet[i].Pi;
        while ((Na = Na->Suc) != FirstNode);
    }
    if (TraceLevel >= 1) {
        CandidateReport();
        // printff("Preprocessing time = %0.5f sec.\n",
        //         fabs(GetTime() - EntryTime));
    }
}
