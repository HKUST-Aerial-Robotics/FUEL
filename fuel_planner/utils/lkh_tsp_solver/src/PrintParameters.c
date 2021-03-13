#include "LKH.h"
#include "Genetic.h"

/*
 * The PrintParameters function prints the problem parameters to 
 * standard output. 
*/

void PrintParameters()
{
    int i;

    printff("ASCENT_CANDIDATES = %d\n", AscentCandidates);
    printff("BACKBONE_TRIALS = %d\n", BackboneTrials);
    printff("BACKTRACKING = %s\n", Backtracking ? "YES" : "NO");
    if (CandidateFiles == 0)
        printff("# CANDIDATE_FILE =\n");
    else
        for (i = 0; i < CandidateFiles; i++)
            printff("CANDIDATE_FILE = %s\n", CandidateFileName[i]);
    printff("CANDIDATE_SET_TYPE = %s%s\n",
            CandidateSetType == ALPHA ? "ALPHA" :
            CandidateSetType == DELAUNAY ? "DELAUNAY" :
            CandidateSetType == NN ? "NEAREST-NEIGHBOR" :
            CandidateSetType == POPMUSIC ? "POPMUSIC" :
            CandidateSetType == QUADRANT ? "QUADRANT" : "",
            DelaunayPure ? " PURE" : "");
    if (EdgeFiles == 0)
        printff("# EDGE_FILE =\n");
    else
        for (i = 0; i < EdgeFiles; i++)
            printff("EDGE_FILE = %s\n", EdgeFileName[i]);
    if (Excess >= 0)
        printff("EXCESS = %g\n", Excess);
    else
        printff("# EXCESS =\n");
    printff("EXTRA_CANDIDATES = %d %s\n",
            ExtraCandidates,
            ExtraCandidateSetSymmetric ? "SYMMETRIC" : "");
    printff("EXTRA_CANDIDATE_SET_TYPE = %s\n",
            ExtraCandidateSetType == NN ? "NEAREST-NEIGHBOR" :
            ExtraCandidateSetType == QUADRANT ? "QUADRANT" : "");
    printff("GAIN23 = %s\n", Gain23Used ? "YES" : "NO");
    printff("GAIN_CRITERION = %s\n", GainCriterionUsed ? "YES" : "NO");
    if (InitialPeriod >= 0)
        printff("INITIAL_PERIOD = %d\n", InitialPeriod);
    else
        printff("# INITIAL_PERIOD =\n");
    printff("INITIAL_STEP_SIZE = %d\n", InitialStepSize);
    printff("INITIAL_TOUR_ALGORITHM = %s\n",
            InitialTourAlgorithm == BORUVKA ? "BORUVKA" :
            InitialTourAlgorithm == GREEDY ? "GREEDY" :
            InitialTourAlgorithm == MOORE ? "MOORE" :
            InitialTourAlgorithm == NEAREST_NEIGHBOR ? "NEAREST-NEIGHBOR" :
            InitialTourAlgorithm == QUICK_BORUVKA ? "QUICK-BORUVKA" :
            InitialTourAlgorithm == SIERPINSKI ? "SIERPINSKI" : "WALK");
    printff("%sINITIAL_TOUR_FILE = %s\n",
            InitialTourFileName ? "" : "# ",
            InitialTourFileName ? InitialTourFileName : "");
    printff("INITIAL_TOUR_FRACTION = %0.3f\n", InitialTourFraction);
    printff("%sINPUT_TOUR_FILE = %s\n",
            InputTourFileName ? "" : "# ",
            InputTourFileName ? InputTourFileName : "");
    printff("KICK_TYPE = %d\n", KickType);
    printff("KICKS = %d\n", Kicks);
    if (MaxBreadth == INT_MAX)
        printff("# MAX_BREADTH =\n");
    else
        printff("MAX_BREADTH = %d\n", MaxBreadth);
    printff("MAX_CANDIDATES = %d %s\n",
            MaxCandidates, CandidateSetSymmetric ? "SYMMETRIC" : "");
    if (MaxSwaps >= 0)
        printff("MAX_SWAPS = %d\n", MaxSwaps);
    else
        printff("# MAX_SWAPS =\n");
    if (MaxTrials >= 0)
        printff("MAX_TRIALS = %d\n", MaxTrials);
    else
        printff("# MAX_TRIALS =\n");
    if (MergeTourFiles == 0)
        printff("# MERGE_TOUR_FILE =\n");
    else
        for (i = 0; i < MergeTourFiles; i++)
            printff("MERGE_TOUR_FILE = %s\n", MergeTourFileName[i]);
    printff("MOVE_TYPE = %d\n", MoveType);
    printff("%sNONSEQUENTIAL_MOVE_TYPE = %d\n",
            PatchingA > 1 ? "" : "# ", NonsequentialMoveType);
    if (Optimum == MINUS_INFINITY)
        printff("# OPTIMUM =\n");
    else
        printff("OPTIMUM = " GainFormat "\n", Optimum);
    printff("%sOUTPUT_TOUR_FILE = %s\n",
            OutputTourFileName ? "" : "# ",
            OutputTourFileName ? OutputTourFileName : "");
    printff("PATCHING_A = %d %s\n", PatchingA,
            PatchingARestricted ? "RESTRICTED" :
            PatchingAExtended ? "EXTENDED" : "");
    printff("PATCHING_C = %d %s\n", PatchingC,
            PatchingCRestricted ? "RESTRICTED" :
            PatchingCExtended ? "EXTENDED" : "");
    printff("%sPI_FILE = %s\n",
            PiFileName ? "" : "# ", PiFileName ? PiFileName : "");
    printff("POPMUSIC_INITIAL_TOUR = %s\n",
            POPMUSIC_InitialTour ? "YES" : "NO");
    printff("POPMUSIC_MAX_NEIGHBORS = %d\n", POPMUSIC_MaxNeighbors);
    printff("POPMUSIC_SAMPLE_SIZE = %d\n", POPMUSIC_SampleSize);
    printff("POPMUSIC_SOLUTIONS = %d\n", POPMUSIC_Solutions);
    printff("POPMUSIC_TRIALS = %d\n", POPMUSIC_Trials);
    if (MaxPopulationSize == 0)
        printff("# ");
    printff("POPULATION_SIZE = %d\n", MaxPopulationSize);
    printff("PRECISION = %d\n", Precision);
    printff("%sPROBLEM_FILE = %s\n",
            ProblemFileName ? "" : "# ",
            ProblemFileName ? ProblemFileName : "");
    printff("RECOMBINATION = %s\n", Recombination == IPT ? "IPT" :
            Recombination == GPX2 ? "GPX2" : "UNKNOWN");
    printff("RESTRICTED_SEARCH = %s\n", RestrictedSearch ? "YES" : "NO");
    printff("RUNS = %d\n", Runs);
    printff("SEED = %u\n", Seed);
    printff("STOP_AT_OPTIMUM = %s\n", StopAtOptimum ? "YES" : "NO");
    printff("SUBGRADIENT = %s\n", Subgradient ? "YES" : "NO");
    if (SubproblemSize == 0)
        printff("# SUBPROBLEM_SIZE =\n");
    else
        printff("SUBPROBLEM_SIZE = %d%s%s%s\n", SubproblemSize,
                DelaunayPartitioning ? " DELAUNAY" :
                KarpPartitioning ? " KARP" :
                KCenterPartitioning ? " K-CENTER" :
                KMeansPartitioning ? " K-MEANS" :
                MoorePartitioning ? " MOORE" :
                RohePartitioning ? " ROHE" :
                SierpinskiPartitioning ? " SIERPINSKI" : "",
                SubproblemBorders ? " BORDERS" : "",
                SubproblemsCompressed ? " COMPRESSED" : "");
    printff("%sSUBPROBLEM_TOUR_FILE = %s\n",
            SubproblemTourFileName ? "" : "# ",
            SubproblemTourFileName ? SubproblemTourFileName : "");
    printff("SUBSEQUENT_MOVE_TYPE = %d\n",
            SubsequentMoveType == 0 ? MoveType : SubsequentMoveType);
    printff("SUBSEQUENT_PATCHING = %s\n",
            SubsequentPatching ? "YES" : "NO");
    if (TimeLimit == DBL_MAX)
        printff("# TIME_LIMIT =\n");
    else
        printff("TIME_LIMIT = %0.1f\n", TimeLimit);
    printff("%sTOUR_FILE = %s\n",
            TourFileName ? "" : "# ", TourFileName ? TourFileName : "");
    printff("TRACE_LEVEL = %d\n\n", TraceLevel);
}
