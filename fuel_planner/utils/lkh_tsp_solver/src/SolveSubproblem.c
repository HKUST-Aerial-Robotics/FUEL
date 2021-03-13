#include "LKH.h"
#include "Genetic.h"

/* 
 * The SolveSubproblem solves a given subproblem. The subproblem is 
 * identified by the parameter CurrentSuproblem and contains all 
 * nodes with Subproblem equal to CurrentSubproblem.   
 *  
 * The parameter Subproblems specifies the number of subproblems.
 * The parameter GlobalBestCost references the current best cost of the 
 * whole problem.
 *
 * If the subproblem is too small (Dimension <= 3), the function returns 0;
 * otherwise 1. 
 */

int
SolveSubproblem(int CurrentSubproblem, int Subproblems,
                GainType * GlobalBestCost)
{
    Node *FirstNodeSaved = FirstNode, *N, *Next, *Last = 0;
    GainType OptimumSaved = Optimum, Cost, Improvement, GlobalCost;
    double LastTime, Time, ExcessSaved = Excess;
    int NewDimension = 0, OldDimension = 0, Number, i, InitialTourEdges =
        0, AscentCandidatesSaved = AscentCandidates, InitialPeriodSaved =
        InitialPeriod, MaxTrialsSaved = MaxTrials;

    BestCost = PLUS_INFINITY;
    FirstNode = 0;
    N = FirstNodeSaved;
    do {
        if (N->Subproblem == CurrentSubproblem) {
            if (SubproblemsCompressed &&
                (((N->SubproblemPred == N->SubBestPred ||
                   FixedOrCommon(N, N->SubproblemPred) ||
                   (N->SubBestPred &&
                    (N->FixedTo1Saved == N->SubBestPred ||
                     N->FixedTo2Saved == N->SubBestPred))) &&
                  (N->SubproblemSuc == N->SubBestSuc ||
                   FixedOrCommon(N, N->SubproblemSuc) ||
                   (N->SubBestSuc &&
                    (N->FixedTo1Saved == N->SubBestSuc ||
                     N->FixedTo2Saved == N->SubBestSuc)))) ||
                 ((N->SubproblemPred == N->SubBestSuc ||
                   FixedOrCommon(N, N->SubproblemPred) ||
                   (N->SubBestSuc &&
                    (N->FixedTo1Saved == N->SubBestSuc ||
                     N->FixedTo2Saved == N->SubBestSuc))) &&
                  (N->SubproblemSuc == N->SubBestPred ||
                   FixedOrCommon(N, N->SubproblemSuc) ||
                   (N->SubBestPred &&
                    (N->FixedTo1Saved == N->SubBestPred ||
                     N->FixedTo2Saved == N->SubBestPred))))))
                N->Subproblem = -CurrentSubproblem;
            else {
                if (!FirstNode)
                    FirstNode = N;
                NewDimension++;
            }
            N->Head = N->Tail = 0;
            if (N->SubBestSuc)
                OldDimension++;
        }
        N->SubBestPred = N->SubBestSuc = 0;
        N->FixedTo1Saved = N->FixedTo1;
        N->FixedTo2Saved = N->FixedTo2;
    } while ((N = N->SubproblemSuc) != FirstNodeSaved);
    if ((Number = CurrentSubproblem % Subproblems) == 0)
        Number = Subproblems;
    if (NewDimension <= 3 || NewDimension == OldDimension) {
        if (TraceLevel >= 1) {
            printff("\nSubproblem %d of %d: Dimension = %d ",
                    Number, Subproblems, NewDimension);
            if (NewDimension <= 3)
                printff("(too small)\n");
            else
                printff("(did not change)\n");
        }
        FirstNode = FirstNodeSaved;
        return 0;
    }
    if (AscentCandidates > NewDimension - 1)
        AscentCandidates = NewDimension - 1;
    if (InitialPeriod < 0) {
        InitialPeriod = NewDimension / 2;
        if (InitialPeriod < 100)
            InitialPeriod = 100;
    }
    if (Excess < 0)
        Excess = 1.0 / NewDimension;
    if (MaxTrials == -1)
        MaxTrials = NewDimension;
    N = FirstNode;
    do {
        Next = N->SubproblemSuc;
        if (N->Subproblem == CurrentSubproblem) {
            N->Pred = N->Suc = N;
            if (N != FirstNode)
                Follow(N, Last);
            Last = N;
        } else if (Next->Subproblem == CurrentSubproblem
                   && !Fixed(Last, Next)) {
            if (!Last->FixedTo1
                || Last->FixedTo1->Subproblem != CurrentSubproblem)
                Last->FixedTo1 = Next;
            else
                Last->FixedTo2 = Next;
            if (!Next->FixedTo1
                || Next->FixedTo1->Subproblem != CurrentSubproblem)
                Next->FixedTo1 = Last;
            else
                Next->FixedTo2 = Last;
            if (C == C_EXPLICIT) {
                if (Last->Id > Next->Id) {
                    Last->SavedCost = Last->C[Next->Id];
                    Last->C[Next->Id] = 0;
                } else {
                    Next->SavedCost = Next->C[Last->Id];
                    Next->C[Last->Id] = 0;
                }
            }
        }
    }
    while ((N = Next) != FirstNode);

    Dimension = NewDimension;
    AllocateSegments();
    InitializeStatistics();
    if (CacheSig)
        for (i = 0; i <= CacheMask; i++)
            CacheSig[i] = 0;
    OptimumSaved = Optimum;
    Optimum = 0;
    N = FirstNode;
    do {
        if (N->SubproblemSuc == N->InitialSuc ||
            N->SubproblemPred == N->InitialSuc)
            InitialTourEdges++;
        if (!Fixed(N, N->Suc))
            Optimum += Distance(N, N->Suc);
        if (N->FixedTo1 && N->Subproblem != N->FixedTo1->Subproblem)
            eprintf("Illegal fixed edge (%d,%d)", N->Id, N->FixedTo1->Id);
        if (N->FixedTo2 && N->Subproblem != N->FixedTo2->Subproblem)
            eprintf("Illegal fixed edge (%d,%d)", N->Id, N->FixedTo2->Id);
        N->BestSuc = N->Suc;
    }
    while ((N = N->Suc) != FirstNode);
    if (TraceLevel >= 1)
        printff
            ("\nSubproblem %d of %d: Dimension = %d, Upper bound = "
             GainFormat "\n", Number, Subproblems, Dimension, Optimum);
    FreeCandidateSets();
    CreateCandidateSet();

    for (Run = 1; Run <= Runs; Run++) {
        LastTime = GetTime();
        Cost = Norm != 0 ? FindTour() : Optimum;
        /* Merge with subproblem tour */
        Last = 0;
        N = FirstNode;
        do {
            if (N->Subproblem == CurrentSubproblem) {
                if (Last)
                    Last->Next = N;
                Last = N;
            }
        }
        while ((N = N->SubproblemSuc) != FirstNode);
        Last->Next = FirstNode;
        Cost = MergeWithTour();
        if (MaxPopulationSize > 1) {
            /* Genetic algorithm */
            for (i = 0; i < PopulationSize; i++)
                Cost = MergeTourWithIndividual(i);
            if (!HasFitness(Cost)) {
                if (PopulationSize < MaxPopulationSize) {
                    AddToPopulation(Cost);
                    if (TraceLevel >= 1)
                        PrintPopulation();
                } else if (Cost < Fitness[PopulationSize - 1]) {
                    ReplaceIndividualWithTour(PopulationSize - 1, Cost);
                    if (TraceLevel >= 1)
                        PrintPopulation();
                }
            }
        }
        if (Cost < BestCost) {
            N = FirstNode;
            do {
                N->SubBestPred = N->Pred;
                N->SubBestSuc = N->Suc;
            } while ((N = N->Suc) != FirstNode);
            BestCost = Cost;
        }
        if (Cost < Optimum || (Cost != Optimum && OutputTourFileName)) {
            Improvement = Optimum - Cost;
            if (Improvement > 0) {
                BestCost = GlobalCost = *GlobalBestCost -= Improvement;
                Optimum = Cost;
            } else
                GlobalCost = *GlobalBestCost - Improvement;
            N = FirstNode;
            do
                N->Mark = 0;
            while ((N = N->SubproblemSuc) != FirstNode);
            do {
                N->Mark = N;
                if (!N->SubproblemSuc->Mark &&
                    (N->Subproblem != CurrentSubproblem ||
                     N->SubproblemSuc->Subproblem != CurrentSubproblem))
                    N->BestSuc = N->SubproblemSuc;
                else if (!N->SubproblemPred->Mark &&
                         (N->Subproblem != CurrentSubproblem ||
                          N->SubproblemPred->Subproblem !=
                          CurrentSubproblem))
                    N->BestSuc = N->SubproblemPred;
                else if (!N->Suc->Mark)
                    N->BestSuc = N->Suc;
                else if (!N->Pred->Mark)
                    N->BestSuc = N->Pred;
                else
                    N->BestSuc = FirstNode;
            }
            while ((N = N->BestSuc) != FirstNode);
            Dimension = ProblemType != ATSP ? DimensionSaved :
                2 * DimensionSaved;
            i = 0;
            do {
                if (ProblemType != ATSP)
                    BetterTour[++i] = N->Id;
                else if (N->Id <= Dimension / 2) {
                    i++;
                    if (N->BestSuc->Id != N->Id + Dimension / 2)
                        BetterTour[i] = N->Id;
                    else
                        BetterTour[Dimension / 2 - i + 1] = N->Id;
                }
            }
            while ((N = N->BestSuc) != FirstNode);
            BetterTour[0] =
                BetterTour[ProblemType !=
                           ATSP ? Dimension : Dimension / 2];
            WriteTour(OutputTourFileName, BetterTour, GlobalCost);
            if (Improvement > 0) {
                do
                    if (N->Subproblem != CurrentSubproblem)
                        break;
                while ((N = N->SubproblemPred) != FirstNode);
                if (N->SubproblemSuc == N->BestSuc) {
                    N = FirstNode;
                    do {
                        N->BestSuc->SubproblemPred = N;
                        N = N->SubproblemSuc = N->BestSuc;
                    }
                    while (N != FirstNode);
                } else {
                    N = FirstNode;
                    do
                        (N->SubproblemPred = N->BestSuc)->SubproblemSuc =
                            N;
                    while ((N = N->BestSuc) != FirstNode);
                }
                RecordBestTour();
                WriteTour(TourFileName, BestTour, GlobalCost);
            }
            Dimension = NewDimension;
            if (TraceLevel >= 1) {
                printff("*** %d: Cost = " GainFormat, Number, GlobalCost);
                if (OptimumSaved != MINUS_INFINITY && OptimumSaved != 0)
                    printff(", Gap = %0.4f%%",
                            100.0 * (GlobalCost -
                                     OptimumSaved) / OptimumSaved);
                printff(", Time = %0.2f sec. %s\n",
                        fabs(GetTime() - LastTime),
                        GlobalCost < OptimumSaved ? "<" : GlobalCost ==
                        OptimumSaved ? "=" : "");
            }
        }

        Time = fabs(GetTime() - LastTime);
        UpdateStatistics(Cost, Time);
        if (TraceLevel >= 1 && Cost != PLUS_INFINITY)
            printff("Run %d: Cost = " GainFormat ", Time = %0.2f sec.\n\n",
                    Run, Cost, Time);
        if (PopulationSize >= 2 &&
            (PopulationSize == MaxPopulationSize
             || Run >= 2 * MaxPopulationSize) && Run < Runs) {
            Node *N;
            int Parent1, Parent2;
            Parent1 = LinearSelection(PopulationSize, 1.25);
            do
                Parent2 = LinearSelection(PopulationSize, 1.25);
            while (Parent1 == Parent2);
            ApplyCrossover(Parent1, Parent2);
            N = FirstNode;
            do {
                int d = C(N, N->Suc);
                AddCandidate(N, N->Suc, d, INT_MAX);
                AddCandidate(N->Suc, N, d, INT_MAX);
                N = N->InitialSuc = N->Suc;
            }
            while (N != FirstNode);
        }
        SRandom(++Seed);
        if (Norm == 0)
            break;
    }

    if (TraceLevel >= 1)
        PrintStatistics();

    if (C == C_EXPLICIT) {
        N = FirstNode;
        do {
            for (i = 1; i < N->Id; i++) {
                N->C[i] -= N->Pi + NodeSet[i].Pi;
                N->C[i] /= Precision;
            }
            if (N->FixedTo1 && N->FixedTo1 != N->FixedTo1Saved) {
                if (N->Id > N->FixedTo1->Id)
                    N->C[N->FixedTo1->Id] = N->SavedCost;
                else
                    N->FixedTo1->C[N->Id] = N->FixedTo1->SavedCost;
            }
            if (N->FixedTo2 && N->FixedTo2 != N->FixedTo2Saved) {
                if (N->Id > N->FixedTo2->Id)
                    N->C[N->FixedTo2->Id] = N->SavedCost;
                else
                    N->FixedTo2->C[N->Id] = N->FixedTo2->SavedCost;
            }
        }
        while ((N = N->Suc) != FirstNode);
    }

    FreeSegments();
    FreeCandidateSets();
    FreePopulation();
    if (InitialTourEdges == Dimension) {
        do
            N->InitialSuc = N->SubproblemSuc;
        while ((N = N->SubproblemSuc) != FirstNode);
    } else {
        do
            N->InitialSuc = 0;
        while ((N = N->SubproblemSuc) != FirstNode);
    }
    Dimension = ProblemType != ATSP ? DimensionSaved : 2 * DimensionSaved;
    N = FirstNode = FirstNodeSaved;
    do {
        N->Suc = N->BestSuc = N->SubproblemSuc;
        N->Suc->Pred = N;
        Next = N->FixedTo1;
        N->FixedTo1 = N->FixedTo1Saved;
        N->FixedTo1Saved = Next;
        Next = N->FixedTo2;
        N->FixedTo2 = N->FixedTo2Saved;
        N->FixedTo2Saved = Next;
    }
    while ((N = N->Suc) != FirstNode);
    Optimum = OptimumSaved;
    Excess = ExcessSaved;
    AscentCandidates = AscentCandidatesSaved;
    InitialPeriod = InitialPeriodSaved;
    MaxTrials = MaxTrialsSaved;
    return 1;
}
