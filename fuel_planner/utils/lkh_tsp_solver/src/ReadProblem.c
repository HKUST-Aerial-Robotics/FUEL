#include "LKH.h"
#include "Heap.h"

/*
 * The ReadProblem function reads the problem data in TSPLIB format from the
 * file specified in the parameter file (PROBLEM_FILE).
 *
 * The following description of the file format is extracted from the TSPLIB
 * documentation.
 *
 * The file consists of a specification part and a data part. The specification
 * part contains information on the file format and on its contents. The data
 * part contains explicit data.
 *
 * (1) The specification part
 *
 * All entries in this section are of the form <keyword> : <value>, where
 * <keyword> denotes an alphanumerical keyword and <value> denotes
 * alphanumerical or numerical data. The terms <string>, <integer> and <real>
 * denote character string, integer or real data, respectively. The order of
 * specification of the keywords in the data file is arbitrary (in principle),
 * but must be consistent, i.e., whenever a keyword is specified, all
 * necessary information for the correct interpretation of the keyword has to
 * be known.
 *
 * Below is given a list of all available keywords.
 *
 * NAME : <string>e
 * Identifies the data file.
 *
 * TYPE : <string>
 * Specifies the type of data. Possible types are
 * TSP          Data for a symmetric traveling salesman problem
 * ATSP         Data for an asymmetric traveling salesman problem
 * HCP          Hamiltonian cycle problem data
 * HPP          Hamiltonian path problem data (not available in TSPLIB)
 *
 * COMMENT : <string>
 * Additional comments (usually the name of the contributor or the creator of
 * the problem instance is given here).
 *
 * DIMENSION : < integer>
 * The number of nodes.
 *
 * EDGE_WEIGHT_TYPE : <string>
 * Specifies how the edge weights (or distances) are given. The values are:
 * ATT          Special distance function for problem att48 and att532
 * CEIL_2D      Weights are Euclidean distances in 2-D rounded up
 * CEIL_3D      Weights are Euclidean distances in 3-D rounded up
 * EUC_2D       Weights are Euclidean distances in 2-D
 * EUC_3D       Weights are Euclidean distances in 3-D
 * EXPLICIT     Weights are listed explicitly in the corresponding section
 * FLOOR_2D     Weights are Euclidean distances in 2-D rounded down
 * FLOOR_3D     Weights are Euclidean distances in 3-D rounded down
 * GEO          Weights are geographical distances in kilometers (TSPLIB).
 *              Coordinates are given in the form DDD.MM where DDD are the
 *              degrees and MM the minutes
 * GEOM         Weights are geographical distances in meters (used for the
 *              world TSP). Coordinates are given in decimal form
 * GEO_MEEUS    Weights are geographical distances in kilometers, computed
 *              according to Meeus' formula.  Coordinates are given in the
 *              form DDD.MM where DDD are the degrees and MM the minutes
 * GEOM_MEEUS   Weights are geographical distances, computed according to
 *              Meeus' formula. Coordinates are given in decimal form
 * MAN_2D       Weights are Manhattan distances in 2-D
 * MAN_3D       Weights are Manhattan distances in 3-D
 * MAX_2D       Weights are maximum distances in 2-D
 * MAX_3D       Weights are maximum distances in 3-D
 * TOR_2D       Wirghes are toroidal distances in 2-D
 * TOR_3D       Wirghes are toroidal distances in 3-D
 * XRAY1        Distance function for crystallography problems (Version 1)
 * XRAY2        Distance function for crystallography problems (Version 2)
 * SPECIAL      There is a special distance function implemented in
 *              the Distance_SPECIAL function.
 *
 * EDGE-WEIGHT_FORMAT : <string>
 * Describes the format of the edge weights if they are given explicitly.
 * The values are
 * FUNCTION         Weights are given by a function (see above)
 * FULL_MATRIX      Weights are given by a full matrix
 * UPPER_ROW        Upper triangular matrix
 *                      (row-wise without diagonal entries)
 * LOWER_ROW        Lower triangular matrix
 *                      (row-wise without diagonal entries)
 * UPPER_DIAG_ROW   Upper triangular matrix
 *                      (row-wise including diagonal entries)
 * LOWER_DIAG_ROW   Lower triangular matrix
 *                      (row-wise including diagonal entries)
 * UPPER_COL        Upper triangular matrix
 *                      (column-wise without diagonal entries)
 * LOWER_COL        Lower triangular matrix
 *                      (column-wise without diagonal entries)
 * UPPER_DIAG_COL   Upper triangular matrix
 *                      (column-wise including diagonal entries)
 * LOWER_DIAG_COL   Lower triangular matrix
 *                      (column-wise including diagonal entries)
 *
 * EDGE_DATA_FORMAT : <string>
 * Describes the format in which the edges of a graph are given, if the
 * graph is not complete. The values are
 * EDGE_LIST    The graph is given by an edge list
 * ADJ_LIST     The graph is given by an adjacency list
 *
 * NODE_COORD_TYPE : <string>
 * Specifies whether the coordinates are associated with each node
 * (which, for example may be used for either graphical display or
 * distance computations.
 * The values are
 * TWOD_COORDS      Nodes are specified by coordinates in 2-D
 * THREED_COORDS    Nodes are specified by coordinates in 3-D
 * NO_COORDS        The nodes do not have associated coordinates
 * The default value is NO_COORDS. In the current implementation, however,
 * the value has no significance.
 *
 * DISPLAY_DATA_TYPE : <string>
 * Specifies how a graphical display of the nodes can be obtained.
 * The values are
 * COORD_DISPLAY    Display is generated from the node coordinates
 * TWOD_DISPLAY     Explicit coordinates in 2-D are given
 * NO_DISPLAY       No graphical display is possible
 *
 * The default value is COORD_DISPLAY if node coordinates are specifies and
 * NO_DISPLAY otherwise. In the current implementation, however, the value
 * has no significance.
 *
 * GRID_SIZE : <real>
 * The grid size for toroidal instances.
 * Default: 1000000.0
 *
 * EOF
 * Terminates input data. The entry is optional.
 *
 * (2) The data part
 *
 * Depending on the choice of specifications some additional data may be
 * required. These data are given corresponding data sections following the
 * specification part. Each data section begins with the corresponding
 * keyword. The length of the sectionis either explicitly known form the
 * format specification, or the section is terminated by an appropriate
 * end-of-section identifier.
 *
 * NODE_COORD_SECTION :
 * Node coordinates are given in this section. Each line is of the form
 *
 *      <integer> <real> <real>
 *
 * if NODE_COORD_TYPE is TWOD_COORDS, or
 *
 *      <integer> <real> <real> <real>
 *
 * if NODE_COORD_TYPE is THREED_COORDS. The integers give the number of the
 * respective nodes. The real numbers are the associated coordinates.
 *
 * EDGE_DATA_SECTION :
 * Edges of the graph are specified in either of the two formats allowed in
 * the EDGE_DATA_FORMAT entry. If the type is EDGE_LIST, then the edges are
 * given as a sequence of lines of one of the forms
 *
 *      <integer> <integer>
 *      <integer> <integer> <integer>
 *
 * each entry giving the terminal nodes of some edge, and if three integers are
 * given, the last one specifies its weight. The list is terminated y a -1.
 * If the type is ADJ_LIST, the section consists of adjacency lists for nodes.
 * The adjacency list of a node x is specified as
 *
 *      <integer> <integer> ... <integer> -1
 *
 * where the first integer gives the number of node x and the following
 * integers (terminated by -1) the numbers of the nodes adjacent to x.
 * The list of adjacency lists are terminated by an additional -1.
 *
 * FIXED_EDGES_SECTION :
 * In this section, edges are listed that are required to appear in each
 * solution to the problem. The edges to be fixed are given in the form
 * (per line)
 *
 *      <integer> <integer>
 *
 * meaning that the edge (arc) from the first node to the second node has
 * to be contained in a solution. This section is terminated by a -1.
 *
 * DISPLAY_DATA_SECTION :
 * If DISPLAY_DATA_TYPE is TWOD_DISPLAY, the 2-dimensional coordinates from
 * which a display can be generated are given in the form (per line)
 *
 *      <integer> <real> <real>
 *
 * The integers specify the respective nodes and the real numbers give the
 * associated coordinates. The contents of this section, however, has no
 * significance in the current implementation.
 *
 * TOUR_SECTION :
 * A tour is specified in this section. The tour is given by a list of
 * integers giving the sequence in which the nodes are visited in the tour.
 * The tour is terminated by a -1. Note: In contrast to the TSPLIB format,
 * only one tour can be given in this section. The tour is used to limit
 * the search (the last edge to be excluded in a non-gainful move must not
 * belong to the tour). In addition, the Alpha field of its edges is set to
 * -1.
 *
 * EDGE_WEIGHT_SECTION :
 * The edge weights are given in the format specifies by the EDGE_WEIGHT_FORMAT
 * entry. At present, all explicit data are integral and is given in one of the
 * (self-explanatory) matrix formats, with explicitly known lengths.
 */

static const char Delimiters[] = " :=\n\t\r\f\v\xef\xbb\xbf";
static void CheckSpecificationPart(void);
static char *Copy(char *S);
static void CreateNodes(void);
static int FixEdge(Node * Na, Node * Nb);
static void Read_DIMENSION(void);
static void Read_DISPLAY_DATA_SECTION(void);
static void Read_DISPLAY_DATA_TYPE(void);
static void Read_EDGE_DATA_FORMAT(void);
static void Read_EDGE_DATA_SECTION(void);
static void Read_EDGE_WEIGHT_FORMAT(void);
static void Read_EDGE_WEIGHT_SECTION(void);
static void Read_EDGE_WEIGHT_TYPE(void);
static void Read_FIXED_EDGES_SECTION(void);
static void Read_GRID_SIZE(void);
static void Read_NAME(void);
static void Read_NODE_COORD_SECTION(void);
static void Read_NODE_COORD_TYPE(void);
static void Read_TOUR_SECTION(FILE ** File);
static void Read_TYPE(void);
static int TwoDWeightType(void);
static int ThreeDWeightType(void);

void ReadProblem()
{
    int i, K;
    char *Line, *Keyword;

    if (!(ProblemFile = fopen(ProblemFileName, "r")))
        eprintf("Cannot open PROBLEM_FILE: \"%s\"", ProblemFileName);
    // if (TraceLevel >= 1)
    //     printff("Reading PROBLEM_FILE: \"%s\" ... ", ProblemFileName);
    FreeStructures();
    FirstNode = 0;
    WeightType = WeightFormat = ProblemType = -1;
    CoordType = NO_COORDS;
    Name = Copy("Unnamed");
    Type = EdgeWeightType = EdgeWeightFormat = 0;
    EdgeDataFormat = NodeCoordType = DisplayDataType = 0;
    Distance = 0;
    GridSize = 1000000.0;
    C = 0;
    c = 0;
    while ((Line = ReadLine(ProblemFile))) {
        if (!(Keyword = strtok(Line, Delimiters)))
            continue;
        for (i = 0; i < (int) strlen(Keyword); i++)
            Keyword[i] = (char) toupper(Keyword[i]);
        if (!strcmp(Keyword, "COMMENT"));
        else if (!strcmp(Keyword, "DEMAND_SECTION"))
            eprintf("Not implemented: %s", Keyword);
        else if (!strcmp(Keyword, "DEPOT_SECTION"))
            eprintf("Not implemented: %s", Keyword);
        else if (!strcmp(Keyword, "DIMENSION"))
            Read_DIMENSION();
        else if (!strcmp(Keyword, "DISPLAY_DATA_SECTION"))
            Read_DISPLAY_DATA_SECTION();
        else if (!strcmp(Keyword, "DISPLAY_DATA_TYPE"))
            Read_DISPLAY_DATA_TYPE();
        else if (!strcmp(Keyword, "EDGE_DATA_FORMAT"))
            Read_EDGE_DATA_FORMAT();
        else if (!strcmp(Keyword, "EDGE_DATA_SECTION"))
            Read_EDGE_DATA_SECTION();
        else if (!strcmp(Keyword, "EDGE_WEIGHT_FORMAT"))
            Read_EDGE_WEIGHT_FORMAT();
        else if (!strcmp(Keyword, "EDGE_WEIGHT_SECTION"))
            Read_EDGE_WEIGHT_SECTION();
        else if (!strcmp(Keyword, "EDGE_WEIGHT_TYPE"))
            Read_EDGE_WEIGHT_TYPE();
        else if (!strcmp(Keyword, "EOF"))
            break;
        else if (!strcmp(Keyword, "FIXED_EDGES_SECTION"))
            Read_FIXED_EDGES_SECTION();
        else if (!strcmp(Keyword, "GRID_SIZE"))
            Read_GRID_SIZE();
        else if (!strcmp(Keyword, "NAME"))
            Read_NAME();
        else if (!strcmp(Keyword, "NODE_COORD_SECTION"))
            Read_NODE_COORD_SECTION();
        else if (!strcmp(Keyword, "NODE_COORD_TYPE"))
            Read_NODE_COORD_TYPE();
        else if (!strcmp(Keyword, "TOUR_SECTION"))
            Read_TOUR_SECTION(&ProblemFile);
        else if (!strcmp(Keyword, "TYPE"))
            Read_TYPE();
        else
            eprintf("Unknown keyword: %s", Keyword);
    }
    Swaps = 0;

    /* Adjust parameters */
    if (Seed == 0)
        Seed = (unsigned) time(0);
    if (Precision == 0)
        Precision = 100;
    if (InitialStepSize == 0)
        InitialStepSize = 1;
    if (MaxSwaps < 0)
        MaxSwaps = Dimension;
    if (KickType > Dimension / 2)
        KickType = Dimension / 2;
    if (Runs == 0)
        Runs = 10;
    if (MaxCandidates > Dimension - 1)
        MaxCandidates = Dimension - 1;
    if (ExtraCandidates > Dimension - 1)
        ExtraCandidates = Dimension - 1;
    if (SubproblemSize >= Dimension)
        SubproblemSize = Dimension;
    else if (SubproblemSize == 0) {
        if (AscentCandidates > Dimension - 1)
            AscentCandidates = Dimension - 1;
        if (InitialPeriod < 0) {
            InitialPeriod = Dimension / 2;
            if (InitialPeriod < 100)
                InitialPeriod = 100;
        }
        if (Excess < 0)
            Excess = 1.0 / Dimension;
        if (MaxTrials == -1)
            MaxTrials = Dimension;
        MakeHeap(Dimension);
    }
    if (POPMUSIC_MaxNeighbors > Dimension - 1)
        POPMUSIC_MaxNeighbors = Dimension - 1;
    if (POPMUSIC_SampleSize > Dimension)
        POPMUSIC_SampleSize = Dimension;
    if (CostMatrix == 0 && Dimension <= MaxMatrixDimension &&
        Distance != 0 && Distance != Distance_1 && Distance != Distance_LARGE &&
        Distance != Distance_ATSP && Distance != Distance_SPECIAL) {
        Node *Ni, *Nj;
        CostMatrix = (int *) calloc((size_t) Dimension * (Dimension - 1) / 2,
                                    sizeof(int));
        Ni = FirstNode->Suc;
        do {
            Ni->C =
                &CostMatrix[(size_t) (Ni->Id - 1) * (Ni->Id - 2) / 2] - 1;
            if (ProblemType != HPP || Ni->Id < Dimension)
                for (Nj = FirstNode; Nj != Ni; Nj = Nj->Suc)
                    Ni->C[Nj->Id] = Fixed(Ni, Nj) ? 0 : Distance(Ni, Nj);
            else
                for (Nj = FirstNode; Nj != Ni; Nj = Nj->Suc)
                    Ni->C[Nj->Id] = 0;
        }
        while ((Ni = Ni->Suc) != FirstNode);
        WeightType = EXPLICIT;
        c = 0;
    }
    if (Precision > 1 && (WeightType == EXPLICIT || ProblemType == ATSP)) {
        int j, n = ProblemType == ATSP ? Dimension / 2 : Dimension;
        for (i = 2; i <= n; i++) {
            Node *N = &NodeSet[i];
            for (j = 1; j < i; j++)
                if (N->C[j] * Precision / Precision != N->C[j])
                    eprintf("PRECISION (= %d) is too large", Precision);
        }
    }
    C = WeightType == EXPLICIT ? C_EXPLICIT : C_FUNCTION;
    D = WeightType == EXPLICIT ? D_EXPLICIT : D_FUNCTION;
    if (SubsequentMoveType == 0)
        SubsequentMoveType = MoveType;
    K = MoveType >= SubsequentMoveType
        || !SubsequentPatching ? MoveType : SubsequentMoveType;
    if (PatchingC > K)
        PatchingC = K;
    if (PatchingA > 1 && PatchingA >= PatchingC)
        PatchingA = PatchingC > 2 ? PatchingC - 1 : 1;
    if (NonsequentialMoveType == -1 ||
        NonsequentialMoveType > K + PatchingC + PatchingA - 1)
        NonsequentialMoveType = K + PatchingC + PatchingA - 1;
    if (PatchingC >= 1) {
        BestMove = BestSubsequentMove = BestKOptMove;
        if (!SubsequentPatching && SubsequentMoveType <= 5) {
            MoveFunction BestOptMove[] =
                { 0, 0, Best2OptMove, Best3OptMove,
                Best4OptMove, Best5OptMove
            };
            BestSubsequentMove = BestOptMove[SubsequentMoveType];
        }
    } else {
        MoveFunction BestOptMove[] = { 0, 0, Best2OptMove, Best3OptMove,
            Best4OptMove, Best5OptMove
        };
        BestMove = MoveType <= 5 ? BestOptMove[MoveType] : BestKOptMove;
        BestSubsequentMove = SubsequentMoveType <= 5 ?
            BestOptMove[SubsequentMoveType] : BestKOptMove;
    }
    if (ProblemType == HCP || ProblemType == HPP)
        MaxCandidates = 0;
    if (TraceLevel >= 1) {
        ;
        // PrintParameters();
    } else
        printff("PROBLEM_FILE = %s\n",
                ProblemFileName ? ProblemFileName : "");
    fclose(ProblemFile);
    if (InitialTourFileName)
        ReadTour(InitialTourFileName, &InitialTourFile);
    if (InputTourFileName)
        ReadTour(InputTourFileName, &InputTourFile);
    if (SubproblemTourFileName && SubproblemSize > 0)
        ReadTour(SubproblemTourFileName, &SubproblemTourFile);
    if (MergeTourFiles >= 1) {
        free(MergeTourFile);
        MergeTourFile = (FILE **) malloc(MergeTourFiles * sizeof(FILE *));
        for (i = 0; i < MergeTourFiles; i++)
            ReadTour(MergeTourFileName[i], &MergeTourFile[i]);
    }
    free(LastLine);
    LastLine = 0;
}

static int TwoDWeightType()
{
    return WeightType == EUC_2D || WeightType == MAX_2D ||
        WeightType == MAN_2D || WeightType == CEIL_2D ||
        WeightType == FLOOR_2D ||
        WeightType == GEO || WeightType == GEOM ||
        WeightType == GEO_MEEUS || WeightType == GEOM_MEEUS ||
        WeightType == ATT || WeightType == TOR_2D ||
        (WeightType == SPECIAL && CoordType == TWOD_COORDS);
}

static int ThreeDWeightType()
{
    return WeightType == EUC_3D || WeightType == MAX_3D ||
        WeightType == MAN_3D || WeightType == CEIL_3D ||
        WeightType == FLOOR_3D || WeightType == TOR_3D ||
        WeightType == XRAY1 || WeightType == XRAY2 ||
        (WeightType == SPECIAL && CoordType == THREED_COORDS);
}

static void CheckSpecificationPart()
{
    if (ProblemType == -1)
        eprintf("TYPE is missing");
    if (Dimension < 3)
        eprintf("DIMENSION < 3 or not specified");
    if (WeightType == -1 && ProblemType != ATSP && ProblemType != HCP &&
        ProblemType != HPP && !EdgeWeightType)
        eprintf("EDGE_WEIGHT_TYPE is missing");
    if (WeightType == EXPLICIT && WeightFormat == -1 && !EdgeWeightFormat)
        eprintf("EDGE_WEIGHT_FORMAT is missing");
    if (WeightType == EXPLICIT && WeightFormat == FUNCTION)
        eprintf("Conflicting EDGE_WEIGHT_TYPE and EDGE_WEIGHT_FORMAT");
    if (WeightType != EXPLICIT
        && (WeightType != SPECIAL || CoordType != NO_COORDS)
        && WeightType != -1 && WeightFormat != -1
        && WeightFormat != FUNCTION)
        eprintf("Conflicting EDGE_WEIGHT_TYPE and EDGE_WEIGHT_FORMAT");
    if (ProblemType == ATSP && WeightType != EXPLICIT && WeightType != -1)
        eprintf("Conflicting TYPE and EDGE_WEIGHT_TYPE");
    if (CandidateSetType == DELAUNAY && !TwoDWeightType()
        && MaxCandidates > 0)
        eprintf
            ("Illegal EDGE_WEIGHT_TYPE for CANDIDATE_SET_TYPE = DELAUNAY");
    if (CandidateSetType == NN && !TwoDWeightType()
        && !ThreeDWeightType() && MaxCandidates > 0)
        eprintf
            ("Illegal EDGE_WEIGHT_TYPE for CANDIDATE_SET_TYPE = "
             "NEAREST-NEIGHBOR");
    if (CandidateSetType == QUADRANT && !TwoDWeightType()
        && !ThreeDWeightType() && MaxCandidates + ExtraCandidates > 0)
        eprintf
            ("Illegal EDGE_WEIGHT_TYPE for CANDIDATE_SET_TYPE = QUADRANT");
    if (ExtraCandidateSetType == NN && !TwoDWeightType()
        && !ThreeDWeightType() && ExtraCandidates > 0)
        eprintf
            ("Illegal EDGE_WEIGHT_TYPE for EXTRA_CANDIDATE_SET_TYPE = "
             "NEAREST-NEIGHBOR");
    if (ExtraCandidateSetType == QUADRANT && !TwoDWeightType()
        && !ThreeDWeightType()
        && ExtraCandidates > 0)
        eprintf
            ("Illegal EDGE_WEIGHT_TYPE for EXTRA_CANDIDATE_SET_TYPE = "
             "QUADRANT");
    if (InitialTourAlgorithm == QUICK_BORUVKA && !TwoDWeightType()
        && !ThreeDWeightType())
        eprintf
            ("Illegal EDGE_WEIGHT_TYPE for INITIAL_TOUR_ALGORITHM = "
             "QUICK-BORUVKA");
    if (InitialTourAlgorithm == SIERPINSKI && !TwoDWeightType())
        eprintf
            ("Illegal EDGE_WEIGHT_TYPE for INITIAL_TOUR_ALGORITHM = "
             "SIERPINSKI");
    if (DelaunayPartitioning && !TwoDWeightType())
        eprintf("Illegal EDGE_WEIGHT_TYPE for DELAUNAY specification");
    if (KarpPartitioning && !TwoDWeightType() && !ThreeDWeightType())
        eprintf("Illegal EDGE_WEIGHT_TYPE for KARP specification");
    if (KCenterPartitioning && !TwoDWeightType() && !ThreeDWeightType())
        eprintf("Illegal EDGE_WEIGHT_TYPE for K-CENTER specification");
    if (KMeansPartitioning && !TwoDWeightType() && !ThreeDWeightType())
        eprintf("Illegal EDGE_WEIGHT_TYPE for K-MEANS specification");
    if (MoorePartitioning && !TwoDWeightType())
        eprintf("Illegal EDGE_WEIGHT_TYPE for MOORE specification");
    if (RohePartitioning && !TwoDWeightType() && !ThreeDWeightType())
        eprintf("Illegal EDGE_WEIGHT_TYPE for ROHE specification");
    if (SierpinskiPartitioning && !TwoDWeightType())
        eprintf("Illegal EDGE_WEIGHT_TYPE for SIERPINSKI specification");
    if (SubproblemBorders && !TwoDWeightType() && !ThreeDWeightType())
        eprintf("Illegal EDGE_WEIGHT_TYPE for BORDERS specification");
}

static char *Copy(char *S)
{
    char *Buffer;

    if (!S || strlen(S) == 0)
        return 0;
    Buffer = (char *) malloc(strlen(S) + 1);
    strcpy(Buffer, S);
    return Buffer;
}

static void CreateNodes()
{
    Node *Prev = 0, *N = 0;
    int i;

    if (Dimension <= 0)
        eprintf("DIMENSION is not positive (or not specified)");
    if (ProblemType == ATSP)
        Dimension *= 2;
    else if (ProblemType == HPP) {
        Dimension++;
        if (Dimension > MaxMatrixDimension)
            eprintf("Dimension too large in HPP problem");
    }
    NodeSet = (Node *) calloc(Dimension + 1, sizeof(Node));
    for (i = 1; i <= Dimension; i++, Prev = N) {
        N = &NodeSet[i];
        if (i == 1)
            FirstNode = N;
        else
            Link(Prev, N);
        N->Id = i;
        if (MergeTourFiles >= 1)
            N->MergeSuc = (Node **) calloc(MergeTourFiles, sizeof(Node *));
    }
    Link(N, FirstNode);
}

static int FixEdge(Node * Na, Node * Nb)
{
    if (!Na->FixedTo1 || Na->FixedTo1 == Nb)
        Na->FixedTo1 = Nb;
    else if (!Na->FixedTo2 || Na->FixedTo2 == Nb)
        Na->FixedTo2 = Nb;
    else
        return 0;
    if (!Nb->FixedTo1 || Nb->FixedTo1 == Na)
        Nb->FixedTo1 = Na;
    else if (!Nb->FixedTo2 || Nb->FixedTo1 == Na)
        Nb->FixedTo2 = Na;
    else
        return 0;
    return 1;
}

static void Read_NAME()
{
    if (!(Name = Copy(strtok(0, Delimiters))))
        eprintf("NAME: string expected");
}

static void Read_DIMENSION()
{
    char *Token = strtok(0, Delimiters);

    if (!Token || !sscanf(Token, "%d", &Dimension))
        eprintf("DIMENSION: integer expected");
    DimensionSaved = Dimension;
}

static void Read_DISPLAY_DATA_SECTION()
{
    Node *N;
    int Id, i;

    CheckSpecificationPart();
    if (ProblemType == HPP)
        Dimension--;
    if (!DisplayDataType || strcmp(DisplayDataType, "TWOD_DISPLAY"))
        eprintf
            ("DISPLAY_DATA_SECTION conflicts with DISPLAY_DATA_TYPE: %s",
             DisplayDataType);
    if (!FirstNode)
        CreateNodes();
    N = FirstNode;
    do
        N->V = 0;
    while ((N = N->Suc) != FirstNode);
    for (i = 1; i <= Dimension; i++) {
        if (!fscanint(ProblemFile, &Id))
            eprintf("DIPLAY_DATA_SECTION: Missing nodes");
        if (Id <= 0 || Id > Dimension)
            eprintf("DIPLAY_DATA_SECTION: Node number out of range: %d",
                    Id);
        N = &NodeSet[Id];
        if (N->V == 1)
            eprintf("DIPLAY_DATA_SECTION: Node number occurs twice: %d",
                    N->Id);
        N->V = 1;
        if (!fscanf(ProblemFile, "%lf", &N->X))
            eprintf("DISPLAY_DATA_SECTION: Missing X-coordinate");
        if (!fscanf(ProblemFile, "%lf", &N->Y))
            eprintf("DISPLAY_DATA_SECTION: Missing Y-coordinate");
        if (CoordType == THREED_COORDS
            && !fscanf(ProblemFile, "%lf", &N->Z))
            eprintf("DISPLAY_DATA_SECTION: Missing Z-coordinate");
    }
    N = FirstNode;
    do
        if (!N->V)
            break;
    while ((N = N->Suc) != FirstNode);
    if (!N->V)
        eprintf("DIPLAY_DATA_SECTION: No coordinates given for node %d",
                N->Id);
    if (ProblemType == HPP)
        Dimension++;
}

static void Read_DISPLAY_DATA_TYPE()
{
    unsigned int i;

    if (!(DisplayDataType = Copy(strtok(0, Delimiters))))
        eprintf("DISPLAY_DATA_TYPE: string expected");
    for (i = 0; i < strlen(DisplayDataType); i++)
        DisplayDataType[i] = (char) toupper(DisplayDataType[i]);
    if (strcmp(DisplayDataType, "COORD_DISPLAY") &&
        strcmp(DisplayDataType, "TWOD_DISPLAY") &&
        strcmp(DisplayDataType, "NO_DISPLAY"))
        eprintf("Unknown DISPLAY_DATA_TYPE: %s", DisplayDataType);
}

static void Read_EDGE_DATA_FORMAT()
{
    unsigned int i;

    if (!(EdgeDataFormat = Copy(strtok(0, Delimiters))))
        eprintf("EDGE_DATA_FORMAT: string expected");
    for (i = 0; i < strlen(EdgeDataFormat); i++)
        EdgeDataFormat[i] = (char) toupper(EdgeDataFormat[i]);
    if (strcmp(EdgeDataFormat, "EDGE_LIST") &&
        strcmp(EdgeDataFormat, "ADJ_LIST"))
        eprintf("Unknown EDGE_DATA_FORMAT: %s", EdgeDataFormat);
    if (SubproblemTourFileName)
        eprintf("(EDGE_DATA_FORMAT)"
                " cannot be used together with SUBPROBLEM_TOUR_FILE");
}

static void Read_EDGE_DATA_SECTION()
{
    Node *Ni, *Nj;
    int i, j, W = 0, WithWeights = 0, FirstLine = 1;
    char *Line;

    CheckSpecificationPart();
    if (!EdgeDataFormat)
        eprintf("Missing EDGE_DATA_FORMAT specification");
    if (!FirstNode)
        CreateNodes();
    if (ProblemType == HPP)
        Dimension--;
    if (!strcmp(EdgeDataFormat, "EDGE_LIST")) {
        Line = ReadLine(ProblemFile);
        if (sscanf(Line, "%d %d %d\n", &i, &j, &W) == 3)
            WithWeights = 1;
        while (i != -1) {
            if (i <= 0 ||
                i > (ProblemType != ATSP ? Dimension : Dimension / 2))
                eprintf("EDGE_DATA_SECTION: Node number out of range: %d",
                        i);
            if (!FirstLine)
                fscanint(ProblemFile, &j);
            if (j <= 0
                || j > (ProblemType !=
                        ATSP ? Dimension : Dimension / 2))
                eprintf
                    ("EDGE_DATA_SECTION: Node number out of range: %d",
                     j);
            if (i == j)
                eprintf("EDGE_DATA_SECTION: Illegal edge: %d to %d",
                        i, j);
            if (ProblemType == ATSP)
                j += Dimension / 2;
            Ni = &NodeSet[i];
            Nj = &NodeSet[j];
            if (WithWeights) {
                if (!FirstLine)
                    fscanint(ProblemFile, &W);
                W *= Precision;
            }
            AddCandidate(Ni, Nj, W, 1);
            AddCandidate(Nj, Ni, W, 1);
            FirstLine = 0;
            if (!fscanint(ProblemFile, &i))
                i = -1;
        }
    } else if (!strcmp(EdgeDataFormat, "ADJ_LIST")) {
        if (!fscanint(ProblemFile, &i))
            i = -1;
        while (i != -1) {
            if (i <= 0 ||
                (ProblemType != ATSP ? Dimension : Dimension / 2))
                eprintf
                    ("EDGE_DATA_SECTION: Node number out of range: %d",
                     i);
            Ni = &NodeSet[i];
            fscanint(ProblemFile, &j);
            while (j != -1) {
                if (j <= 0 ||
                    (ProblemType != ATSP ? Dimension : Dimension / 2))
                    eprintf
                        ("EDGE_DATA_SECTION: Node number out of range: %d",
                         j);
                if (i == j)
                    eprintf("EDGE_DATA_SECTION: Illgal edge: %d to %d",
                            i, j);
                if (ProblemType == ATSP)
                    j += Dimension / 2;
                Nj = &NodeSet[j];
                AddCandidate(Ni, Nj, 0, 1);
                AddCandidate(Nj, Ni, 0, 1);
                fscanint(ProblemFile, &j);
            }
            fscanint(ProblemFile, &i);
        }
    } else
        eprintf("EDGE_DATA_SECTION: No EDGE_DATA_FORMAT specified");
    if (ProblemType == HPP)
        Dimension++;
    if (ProblemType == ATSP) {
        for (i = 1; i <= DimensionSaved; i++)
            FixEdge(&NodeSet[i], &NodeSet[i + DimensionSaved]);
    }
    WeightType = -1;
    MaxCandidates = ExtraCandidates = 0;
    Distance = WithWeights ? Distance_LARGE : Distance_1;
}

static void Read_EDGE_WEIGHT_FORMAT()
{
    unsigned int i;

    if (!(EdgeWeightFormat = Copy(strtok(0, Delimiters))))
        eprintf("EDGE_WEIGHT_FORMAT: string expected");
    for (i = 0; i < strlen(EdgeWeightFormat); i++)
        EdgeWeightFormat[i] = (char) toupper(EdgeWeightFormat[i]);
    if (!strcmp(EdgeWeightFormat, "FUNCTION"))
        WeightFormat = FUNCTION;
    else if (!strcmp(EdgeWeightFormat, "FULL_MATRIX"))
        WeightFormat = FULL_MATRIX;
    else if (!strcmp(EdgeWeightFormat, "UPPER_ROW"))
        WeightFormat = UPPER_ROW;
    else if (!strcmp(EdgeWeightFormat, "LOWER_ROW"))
        WeightFormat = LOWER_ROW;
    else if (!strcmp(EdgeWeightFormat, "UPPER_DIAG_ROW"))
        WeightFormat = UPPER_DIAG_ROW;
    else if (!strcmp(EdgeWeightFormat, "LOWER_DIAG_ROW"))
        WeightFormat = LOWER_DIAG_ROW;
    else if (!strcmp(EdgeWeightFormat, "UPPER_COL"))
        WeightFormat = UPPER_COL;
    else if (!strcmp(EdgeWeightFormat, "LOWER_COL"))
        WeightFormat = LOWER_COL;
    else if (!strcmp(EdgeWeightFormat, "UPPER_DIAG_COL"))
        WeightFormat = UPPER_DIAG_COL;
    else if (!strcmp(EdgeWeightFormat, "LOWER_DIAG_COL"))
        WeightFormat = LOWER_DIAG_COL;
    else
        eprintf("Unknown EDGE_WEIGHT_FORMAT: %s", EdgeWeightFormat);
}

static void Read_EDGE_WEIGHT_SECTION()
{
    Node *Ni, *Nj;
    int i, j, n, W;

    CheckSpecificationPart();
    if (!FirstNode)
        CreateNodes();
    if (ProblemType != ATSP) {
        CostMatrix = (int *) calloc((size_t) Dimension * (Dimension - 1) / 2,
                                    sizeof(int));
        Ni = FirstNode->Suc;
        do {
            Ni->C =
                &CostMatrix[(size_t) (Ni->Id - 1) * (Ni->Id - 2) / 2] - 1;
        }
        while ((Ni = Ni->Suc) != FirstNode);
    } else {
        n = Dimension / 2;
        CostMatrix = (int *) calloc((size_t) n * n, sizeof(int));
        for (Ni = FirstNode; Ni->Id <= n; Ni = Ni->Suc)
            Ni->C = &CostMatrix[(size_t) (Ni->Id - 1) * n] - 1;
    }
    if (ProblemType == HPP)
        Dimension--;
    switch (WeightFormat) {
    case FULL_MATRIX:
        if (ProblemType == ATSP) {
            n = Dimension / 2;
            for (i = 1; i <= n; i++) {
                Ni = &NodeSet[i];
                for (j = 1; j <= n; j++) {
                    if (!fscanint(ProblemFile, &W))
                        eprintf("EDGE_WEIGHT_SECTION: Missing weight");
                    if (j != i && W > INT_MAX / 2 / Precision)
                        eprintf("EDGE_WEIGHT_SECTION: "
                                "Weight %d > INT_MAX / 2 / PRECISION", W);
                    Ni->C[j] = W;
                    if (i != j && W > M)
                        M = W;
                }
                Nj = &NodeSet[i + n];
                FixEdge(Ni, Nj);
            }
            Distance = Distance_ATSP;
            WeightType = -1;
        } else
            // for (i = 1, Ni = FirstNode; i <= Dimension; i++, Ni = Ni->Suc) {
            //     for (j = 1; j <= Dimension; j++) {
            for (i = 2, Ni = FirstNode; i <= Dimension; i++, Ni = Ni->Suc) {
                for (j = 1; j < i; j++) {
                    if (!fscanint(ProblemFile, &W))
                        eprintf("EDGE_WEIGHT_SECTION: Missing weight");
                    if (j == i)
                        continue;
                    if (W > INT_MAX / 2 / Precision)
                        eprintf("EDGE_WEIGHT_SECTION: "
                                "Weight %d > INT_MAX / 2 / PRECISION", W);
                    Ni->C[j] = W;
                }
            }
        break;
    case UPPER_ROW:
        for (i = 1, Ni = FirstNode; i < Dimension; i++, Ni = Ni->Suc) {
            for (j = i + 1, Nj = Ni->Suc; j <= Dimension;
                 j++, Nj = Nj->Suc) {
                if (!fscanint(ProblemFile, &W))
                    eprintf("EDGE_WEIGHT_SECTION: Missing weight");
                if (W > INT_MAX / 2 / Precision)
                    eprintf("EDGE_WEIGHT_SECTION: "
                            "Weight %d > INT_MAX / 2 / PRECISION", W);
                Nj->C[i] = W;
            }
        }
        break;
    case LOWER_ROW:
        for (i = 2, Ni = FirstNode->Suc; i <= Dimension; i++, Ni = Ni->Suc) {
            for (j = 1; j < i; j++) {
                if (!fscanint(ProblemFile, &W))
                    eprintf("EDGE_WEIGHT_SECTION: Missing weight");
                if (W > INT_MAX / 2 / Precision)
                    eprintf("EDGE_WEIGHT_SECTION: "
                            "Weight %d > INT_MAX / 2 / PRECISION", W);
                Ni->C[j] = W;
            }
        }
        break;
    case UPPER_DIAG_ROW:
        for (i = 1, Ni = FirstNode; i <= Dimension; i++, Ni = Ni->Suc) {
            for (j = i, Nj = Ni; j <= Dimension; j++, Nj = Nj->Suc) {
                if (!fscanint(ProblemFile, &W))
                    eprintf("EDGE_WEIGHT_SECTION: Missing weight");
                if (j == i)
                    continue;
                if (W > INT_MAX / 2 / Precision)
                    eprintf("EDGE_WEIGHT_SECTION: "
                            "Weight %d > INT_MAX / 2 / PRECISION", W);
                Nj->C[i] = W;
            }
        }
        break;
    case LOWER_DIAG_ROW:
        for (i = 1, Ni = FirstNode; i <= Dimension; i++, Ni = Ni->Suc) {
            for (j = 1; j <= i; j++) {
                if (!fscanint(ProblemFile, &W))
                    eprintf("EDGE_WEIGHT_SECTION: Missing weight");
                if (j == i)
                    continue;
                if (W > INT_MAX / 2 / Precision)
                    eprintf("EDGE_WEIGHT_SECTION: "
                            "Weight %d > INT_MAX / 2 / PRECISION", W);
                Ni->C[j] = W;
            }
        }
        break;
    case UPPER_COL:
        for (j = 2, Nj = FirstNode->Suc; j <= Dimension; j++, Nj = Nj->Suc) {
            for (i = 1; i < j; i++) {
                if (!fscanint(ProblemFile, &W))
                    eprintf("EDGE_WEIGHT_SECTION: Missing weight");
                if (W > INT_MAX / 2 / Precision)
                    eprintf("EDGE_WEIGHT_SECTION: "
                            "Weight %d > INT_MAX / 2 / PRECISION", W);
                Nj->C[i] = W;
            }
        }
        break;
    case LOWER_COL:
        for (j = 1, Nj = FirstNode; j < Dimension; j++, Nj = Nj->Suc) {
            for (i = j + 1, Ni = Nj->Suc; i <= Dimension;
                 i++, Ni = Ni->Suc) {
                if (!fscanint(ProblemFile, &W))
                    eprintf("EDGE_WEIGHT_SECTION: Missing weight");
                if (W > INT_MAX / 2 / Precision)
                    eprintf("EDGE_WEIGHT_SECTION: "
                            "Weight %d > INT_MAX / 2 / PRECISION", W);
                Ni->C[j] = W;
            }
        }
        break;
    case UPPER_DIAG_COL:
        for (j = 1, Nj = FirstNode; j <= Dimension; j++, Nj = Nj->Suc) {
            for (i = 1; i <= j; i++) {
                if (!fscanint(ProblemFile, &W))
                    eprintf("EDGE_WEIGHT_SECTION: Missing weight");
                if (j == i)
                    continue;
                if (W > INT_MAX / 2 / Precision)
                    eprintf("EDGE_WEIGHT_SECTION: "
                            "Weight %d > INT_MAX / 2 / PRECISION", W);
                Nj->C[i] = W;
            }
        }
        break;
    case LOWER_DIAG_COL:
        for (j = 1, Nj = FirstNode; j <= Dimension; j++, Nj = Nj->Suc) {
            for (i = j, Ni = Nj; i <= Dimension; i++, Ni = Ni->Suc) {
                if (!fscanint(ProblemFile, &W))
                    eprintf("EDGE_WEIGHT_SECTION: Missing weight");
                if (j == i)
                    continue;
                if (W > INT_MAX / 2 / Precision)
                    eprintf("EDGE_WEIGHT_SECTION: "
                            "Weight %d > INT_MAX / 2 / PRECISION", W);
                Ni->C[j] = W;
            }
        }
        break;
    }
    if (ProblemType == HPP)
        Dimension++;
}

static void Read_EDGE_WEIGHT_TYPE()
{
    unsigned int i;

    if (!(EdgeWeightType = Copy(strtok(0, Delimiters))))
        eprintf("EDGE_WEIGHT_TYPE: string expected");
    for (i = 0; i < strlen(EdgeWeightType); i++)
        EdgeWeightType[i] = (char) toupper(EdgeWeightType[i]);
    if (!strcmp(EdgeWeightType, "ATT")) {
        WeightType = ATT;
        Distance = Distance_ATT;
        c = c_ATT;
        CoordType = TWOD_COORDS;
    } else if (!strcmp(EdgeWeightType, "CEIL_2D")) {
        WeightType = CEIL_2D;
        Distance = Distance_CEIL_2D;
        c = c_CEIL_2D;
        CoordType = TWOD_COORDS;
    } else if (!strcmp(EdgeWeightType, "CEIL_3D")) {
        WeightType = CEIL_3D;
        Distance = Distance_CEIL_3D;
        c = c_CEIL_3D;
        CoordType = THREED_COORDS;
    } else if (!strcmp(EdgeWeightType, "EUC_2D")) {
        WeightType = EUC_2D;
        Distance = Distance_EUC_2D;
        c = c_EUC_2D;
        CoordType = TWOD_COORDS;
    } else if (!strcmp(EdgeWeightType, "EUC_3D")) {
        WeightType = EUC_3D;
        Distance = Distance_EUC_3D;
        c = c_EUC_3D;
        CoordType = THREED_COORDS;
    } else if (!strcmp(EdgeWeightType, "EXPLICIT")) {
        WeightType = EXPLICIT;
        Distance = Distance_EXPLICIT;
    } else if (!strcmp(EdgeWeightType, "FLOOR_2D")) {
        WeightType = FLOOR_2D;
        Distance = Distance_FLOOR_2D;
        c = c_FLOOR_2D;
        CoordType = TWOD_COORDS;
    } else if (!strcmp(EdgeWeightType, "FLOOR_3D")) {
        WeightType = FLOOR_3D;
        Distance = Distance_FLOOR_3D;
        c = c_FLOOR_3D;
        CoordType = THREED_COORDS;
    } else if (!strcmp(EdgeWeightType, "MAN_2D")) {
        WeightType = MAN_2D;
        Distance = Distance_MAN_2D;
        CoordType = TWOD_COORDS;
    } else if (!strcmp(EdgeWeightType, "MAN_3D")) {
        WeightType = MAN_3D;
        Distance = Distance_MAN_3D;
        CoordType = THREED_COORDS;
    } else if (!strcmp(EdgeWeightType, "MAX_2D")) {
        WeightType = MAX_2D;
        Distance = Distance_MAX_2D;
        CoordType = TWOD_COORDS;
    } else if (!strcmp(EdgeWeightType, "MAX_3D")) {
        WeightType = MAX_3D;
        Distance = Distance_MAX_3D;
        CoordType = THREED_COORDS;
    } else if (!strcmp(EdgeWeightType, "GEO")) {
        WeightType = GEO;
        Distance = Distance_GEO;
        c = c_GEO;
        CoordType = TWOD_COORDS;
    } else if (!strcmp(EdgeWeightType, "GEOM")) {
        WeightType = GEOM;
        Distance = Distance_GEOM;
        c = c_GEOM;
        CoordType = TWOD_COORDS;
    } else if (!strcmp(EdgeWeightType, "GEO_MEEUS")) {
        WeightType = GEO_MEEUS;
        Distance = Distance_GEO_MEEUS;
        c = c_GEO_MEEUS;
        CoordType = TWOD_COORDS;
    } else if (!strcmp(EdgeWeightType, "GEOM_MEEUS")) {
        WeightType = GEOM_MEEUS;
        Distance = Distance_GEOM_MEEUS;
        c = c_GEOM_MEEUS;
        CoordType = TWOD_COORDS;
    } else if (!strcmp(EdgeWeightType, "TOR_2D")) {
        WeightType = TOR_2D;
        Distance = Distance_TOR_2D;
        CoordType = TWOD_COORDS;
    } else if (!strcmp(EdgeWeightType, "TOR_3D")) {
        WeightType = TOR_3D;
        Distance = Distance_TOR_3D;
        CoordType = THREED_COORDS;
    } else if (!strcmp(EdgeWeightType, "XRAY1")) {
        WeightType = XRAY1;
        Distance = Distance_XRAY1;
        CoordType = THREED_COORDS;
    } else if (!strcmp(EdgeWeightType, "XRAY2")) {
        WeightType = XRAY2;
        Distance = Distance_XRAY2;
        CoordType = THREED_COORDS;
    } else if (!strcmp(EdgeWeightType, "SPECIAL")) {
        WeightType = SPECIAL;
        Distance = Distance_SPECIAL;
    } else
        eprintf("Unknown EDGE_WEIGHT_TYPE: %s", EdgeWeightType);
}

static void Read_FIXED_EDGES_SECTION()
{
    Node *Ni, *Nj, *N, *NPrev = 0, *NNext;
    int i, j, Count = 0;

    CheckSpecificationPart();
    if (!FirstNode)
        CreateNodes();
    if (ProblemType == HPP)
        Dimension--;
    if (!fscanint(ProblemFile, &i))
        i = -1;
    while (i != -1) {
        if (i <= 0
            || i > (ProblemType != ATSP ? Dimension : Dimension / 2))
            eprintf("FIXED_EDGES_SECTION: Node number out of range: %d",
                    i);
        fscanint(ProblemFile, &j);
        if (j <= 0
            || j > (ProblemType != ATSP ? Dimension : Dimension / 2))
            eprintf("FIXED_EDGES_SECTION: Node number out of range: %d",
                    j);
        if (i == j)
            eprintf("FIXED_EDGES_SECTION: Illegal edge: %d to %d", i, j);
        Ni = &NodeSet[i];
        Nj = &NodeSet[ProblemType == ATSP ? j + Dimension / 2 : j];
        if (!FixEdge(Ni, Nj))
            eprintf("FIXED_EDGES_SECTION: Illegal fix: %d to %d", i, j);
        /* Cycle check */
        N = Ni;
        Count = 0;
        do {
            NNext = N->FixedTo1 != NPrev ? N->FixedTo1 : N->FixedTo2;
            NPrev = N;
            Count++;
        } while ((N = NNext) && N != Ni);
        if (N == Ni && Count != Dimension)
            eprintf("FIXED_EDGES_SECTION: Illegal fix: %d to %d", i, j);
        if (!fscanint(ProblemFile, &i))
            i = -1;
    }
    if (ProblemType == HPP)
        Dimension++;
}

static void Read_GRID_SIZE()
{
    char *Token = strtok(0, Delimiters);

    if (!Token || !sscanf(Token, "%lf", &GridSize))
        eprintf("GRID_SIZE: real expected");
    if (GridSize < 0)
        eprintf("GRID_SIZE: non-negative real expected");
}

static void Read_NODE_COORD_SECTION()
{
    Node *N;
    int Id, i;

    CheckSpecificationPart();
    if (CoordType != TWOD_COORDS && CoordType != THREED_COORDS)
        eprintf("NODE_COORD_SECTION conflicts with NODE_COORD_TYPE: %s",
                NodeCoordType);
    if (!FirstNode)
        CreateNodes();
    N = FirstNode;
    do
        N->V = 0;
    while ((N = N->Suc) != FirstNode);
    if (ProblemType == HPP)
        Dimension--;
    for (i = 1; i <= Dimension; i++) {
        if (!fscanint(ProblemFile, &Id))
            eprintf("NODE_COORD_SECTION: Missing nodes");
        if (Id <= 0 || Id > Dimension)
            eprintf("NODE_COORD_SECTION: Node number out of range: %d",
                    Id);
        N = &NodeSet[Id];
        if (N->V == 1)
            eprintf("NODE_COORD_SECTION: Node number occurs twice: %d",
                    N->Id);
        N->V = 1;
        if (!fscanf(ProblemFile, "%lf", &N->X))
            eprintf("NODE_COORD_SECTION : Missing X-coordinate");
        if (!fscanf(ProblemFile, "%lf", &N->Y))
            eprintf("NODE_COORD_SECTION: Missing Y-coordinate");
        if (CoordType == THREED_COORDS
            && !fscanf(ProblemFile, "%lf", &N->Z))
            eprintf("NODE_COORD_SECTION: Missing Z-coordinate");
        if (Name && !strcmp(Name, "d657")) {
            N->X = (float) N->X;
            N->Y = (float) N->Y;
        }
    }
    N = FirstNode;
    do
        if (!N->V && N->Id <= Dimension)
            break;
    while ((N = N->Suc) != FirstNode);
    if (!N->V)
        eprintf("NODE_COORD_SECTION: No coordinates given for node %d",
                N->Id);
    if (ProblemType == HPP)
        Dimension++;
}

static void Read_NODE_COORD_TYPE()
{
    unsigned int i;

    if (!(NodeCoordType = Copy(strtok(0, Delimiters))))
        eprintf("NODE_COORD_TYPE: string expected");
    for (i = 0; i < strlen(NodeCoordType); i++)
        NodeCoordType[i] = (char) toupper(NodeCoordType[i]);
    if (!strcmp(NodeCoordType, "TWOD_COORDS"))
        CoordType = TWOD_COORDS;
    else if (!strcmp(NodeCoordType, "THREED_COORDS"))
        CoordType = THREED_COORDS;
    else if (!strcmp(NodeCoordType, "NO_COORDS"))
        CoordType = NO_COORDS;
    else
        eprintf("Unknown NODE_COORD_TYPE: %s", NodeCoordType);
}

static void Read_TOUR_SECTION(FILE ** File)
{
    Node *First = 0, *Last = 0, *N, *Na;
    int i, k;

    if (TraceLevel >= 1) {
        printff("Reading ");
        if (File == &InitialTourFile)
            printff("INITIAL_TOUR_FILE: \"%s\" ... ", InitialTourFileName);
        else if (File == &InputTourFile)
            printff("INPUT_TOUR_FILE: \"%s\" ... ", InputTourFileName);
        else if (File == &SubproblemTourFile)
            printff("SUBPROBLEM_TOUR_FILE: \"%s\" ... ",
                    SubproblemTourFileName);
        else
            for (i = 0; i < MergeTourFiles; i++)
                if (File == &MergeTourFile[i])
                    printff("MERGE_TOUR_FILE: \"%s\" ... ",
                            MergeTourFileName[i]);
    }
    if (!FirstNode)
        CreateNodes();
    N = FirstNode;
    do
        N->V = 0;
    while ((N = N->Suc) != FirstNode);
    if (ProblemType == HPP)
        Dimension--;
    if (ProblemType == ATSP)
        Dimension /= 2;
    if (!fscanint(*File, &i))
        i = -1;
    for (k = 0; k <= Dimension && i != -1; k++) {
        if (i <= 0 || i > Dimension)
            eprintf("TOUR_SECTION: Node number out of range: %d", i);
        N = &NodeSet[i];
        if (N->V == 1 && k != Dimension)
            eprintf("TOUR_SECTION: Node number occurs twice: %d", N->Id);
        N->V = 1;
        if (k == 0)
            First = Last = N;
        else {
            if (ProblemType == ATSP) {
                Na = N + Dimension;
                Na->V = 1;
            } else
                Na = 0;
            if (File == &InitialTourFile) {
                if (!Na)
                    Last->InitialSuc = N;
                else {
                    Last->InitialSuc = Na;
                    Na->InitialSuc = N;
                }
            } else if (File == &InputTourFile) {
                if (!Na)
                    Last->InputSuc = N;
                else {
                    Last->InputSuc = Na;
                    Na->InputSuc = N;
                }
            } else if (File == &SubproblemTourFile) {
                if (!Na)
                    (Last->SubproblemSuc = N)->SubproblemPred = Last;
                else {
                    (Last->SubproblemSuc = Na)->SubproblemPred = Last;
                    (Na->SubproblemSuc = N)->SubproblemPred = Na;
                }
            } else {
                for (i = 0; i < MergeTourFiles; i++) {
                    if (File == &MergeTourFile[i]) {
                        if (!Na) {
                            Last->MergeSuc[i] = N;
                            if (i == 0)
                                N->MergePred = Last;
                        } else {
                            Last->MergeSuc[i] = Na;
                            Na->MergeSuc[i] = N;
                            if (i == 0) {
                                Na->MergePred = Last;
                                N->MergePred = Na;
                            }
                        }
                    }
                }
            }
            Last = N;
        }
        if (k < Dimension)
            fscanint(*File, &i);
        if (k == Dimension - 1)
            i = First->Id;
    }
    N = FirstNode;
    do
        if (!N->V)
            eprintf("TOUR_SECTION: Node is missing: %d", N->Id);
    while ((N = N->Suc) != FirstNode);
    if (File == &SubproblemTourFile) {
        do {
            if (N->FixedTo1 &&
                N->SubproblemPred != N->FixedTo1
                && N->SubproblemSuc != N->FixedTo1)
                eprintf("Fixed edge (%d, %d) "
                        "does not belong to subproblem tour", N->Id,
                        N->FixedTo1->Id);
            if (N->FixedTo2 && N->SubproblemPred != N->FixedTo2
                && N->SubproblemSuc != N->FixedTo2)
                eprintf("Fixed edge (%d, %d) "
                        "does not belong to subproblem tour", N->Id,
                        N->FixedTo2->Id);
        } while ((N = N->Suc) != FirstNode);
    }
    if (ProblemType == HPP)
        Dimension++;
    if (ProblemType == ATSP)
        Dimension *= 2;
    if (TraceLevel >= 1)
        ;
}

static void Read_TYPE()
{
    unsigned int i;

    if (!(Type = Copy(strtok(0, Delimiters))))
        eprintf("TYPE: string expected");
    for (i = 0; i < strlen(Type); i++)
        Type[i] = (char) toupper(Type[i]);
    if (!strcmp(Type, "TSP"))
        ProblemType = TSP;
    else if (!strcmp(Type, "ATSP"))
        ProblemType = ATSP;
    else if (!strcmp(Type, "SOP")) {
        ProblemType = SOP;
        eprintf("TYPE: Type not implemented: %s", Type);
    } else if (!strcmp(Type, "HCP"))
        ProblemType = HCP;
    else if (!strcmp(Type, "CVRP")) {
        ProblemType = CVRP;
        eprintf("TYPE: Type not implemented: %s", Type);
    } else if (!strcmp(Type, "TOUR")) {
        ProblemType = TOUR;
        eprintf("TYPE: Type not implemented: %s", Type);
    } else if (!strcmp(Type, "HPP"))
        ProblemType = HPP;
    else
        eprintf("Unknown TYPE: %s", Type);
}

/*
 The ReadTour function reads a tour from a file.
 
 The format is as follows:
 
 OPTIMUM = <real>
 Known optimal tour length. A run will be terminated as soon as a tour
 length less than or equal to optimum is achieved.
 Default: MINUS_INFINITY.
 
 TOUR_SECTION :
 A tour is specified in this section. The tour is given by a list of integers
 giving the sequence in which the nodes are visited in the tour. The tour is
 terminated by a -1.
 
 EOF
 Terminates the input data. The entry is optional.
 
 Other keywords in TSPLIB format may be included in the file, but they are
 ignored.
 */

void ReadTour(char *FileName, FILE ** File)
{
    char *Line, *Keyword, *Token;
    unsigned int i;
    int Done = 0;

    if (!(*File = fopen(FileName, "r")))
        eprintf("Cannot open tour file: \"%s\"", FileName);
    while ((Line = ReadLine(*File))) {
        if (!(Keyword = strtok(Line, Delimiters)))
            continue;
        for (i = 0; i < strlen(Keyword); i++)
            Keyword[i] = (char) toupper(Keyword[i]);
        if (!strcmp(Keyword, "COMMENT") ||
            !strcmp(Keyword, "DEMAND_SECTION") ||
            !strcmp(Keyword, "DEPOT_SECTION") ||
            !strcmp(Keyword, "DISPLAY_DATA_SECTION") ||
            !strcmp(Keyword, "DISPLAY_DATA_TYPE") ||
            !strcmp(Keyword, "EDGE_DATA_FORMAT") ||
            !strcmp(Keyword, "EDGE_DATA_SECTION") ||
            !strcmp(Keyword, "EDGE_WEIGHT_FORMAT") ||
            !strcmp(Keyword, "EDGE_WEIGHT_SECTION") ||
            !strcmp(Keyword, "EDGE_WEIGHT_TYPE") ||
            !strcmp(Keyword, "FIXED_EDGES_SECTION") ||
            !strcmp(Keyword, "NAME") ||
            !strcmp(Keyword, "NODE_COORD_SECTION") ||
            !strcmp(Keyword, "NODE_COORD_TYPE")
            || !strcmp(Keyword, "TYPE"));
        else if (strcmp(Keyword, "OPTIMUM") == 0) {
            if (!(Token = strtok(0, Delimiters)) ||
                !sscanf(Token, GainInputFormat, &Optimum))
                eprintf("[%s] (OPTIMUM): Integer expected", FileName);
        } else if (strcmp(Keyword, "DIMENSION") == 0) {
            int Dim = 0;
            if (!(Token = strtok(0, Delimiters)) ||
                !sscanf(Token, "%d", &Dim))
                eprintf("[%s] (DIMENSION): integer expected", FileName);
            if (Dim != DimensionSaved)
                eprintf
                    ("[%s] (DIMENSION): does not match problem dimension",
                     FileName);
        } else if (!strcmp(Keyword, "TOUR_SECTION")) {
            Read_TOUR_SECTION(File);
            Done = 1;
        } else if (!strcmp(Keyword, "EOF"))
            break;
        else
            eprintf("[%s] Unknown Keyword: %s", FileName, Keyword);
    }
    if (!Done)
        eprintf("Missing TOUR_SECTION in tour file: \"%s\"", FileName);
    fclose(*File);
}
