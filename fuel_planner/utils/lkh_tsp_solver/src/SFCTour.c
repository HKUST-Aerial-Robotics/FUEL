#include "LKH.h"

/*
 * The SFCTour function computes a tour using a space-filling curve.
 * See
 * 
 *    Loren K. Platzman and John J. Bartholdi III,
 *    Spacefilling curves and the planar travelling salesman problem,
 *    J. ACM, Vol. 36, 4, pp. 710-737 (1989).
 *
 * If SIERPINSKI is passed as parameter, the function uses a
 * Sierpinski curve. Otherwise, the function uses a Moore curve:
 *
 *     E. H. Moore,
 *     On Certain Crinkly Curves,
 *     Trans. Amer. Math Soc., 1, pp. 72-90 (1900).
 *
 * The function returns the cost of the resulting tour. 
 */

static int SierpinskiIndex(double x, double y);
static int MooreIndex(double x, double y);
static int compare(const void *Na, const void *Nb);

typedef int (*IndexFunction) (double x, double y);

GainType SFCTour(int CurveType)
{
    double XMin, XMax, YMin, YMax;
    Node *N, **Perm;
    int i;
    IndexFunction Index;
    GainType Cost;
    double EntryTime = GetTime();

    if (CurveType == SIERPINSKI) {
        if (TraceLevel >= 1)
            printff("Sierpinski = ");
        Index = SierpinskiIndex;
    } else {
        if (TraceLevel >= 1)
            printff("Moore = ");
        Index = MooreIndex;
    }
    N = FirstNode;
    XMin = XMax = N->X;
    YMin = YMax = N->Y;
    N->V = 0;
    while ((N = N->Suc) != FirstNode) {
        if (N->X < XMin)
            XMin = N->X;
        else if (N->X > XMax)
            XMax = N->X;
        if (N->Y < YMin)
            YMin = N->Y;
        else if (N->Y > YMax)
            YMax = N->Y;
    }
    if (XMax == XMin)
        XMax = XMin + 1;
    if (YMax == YMin)
        YMax = YMin + 1;

    Perm = (Node **) malloc(Dimension * sizeof(Node *));
    for (i = 0, N = FirstNode; i < Dimension; i++, N = N->Suc)
        (Perm[i] = N)->V =
            Index((N->X - XMin) / (XMax - XMin),
                  (N->Y - YMin) / (YMax - YMin));
    qsort(Perm, Dimension, sizeof(Node *), compare);
    for (i = 1; i < Dimension; i++)
        Follow(Perm[i], Perm[i - 1]);
    free(Perm);

    /* Assure that all fixed or common edges belong to the tour */
    N = FirstNode;
    do {
        N->LastV = 1;
        if (!FixedOrCommon(N, N->Suc) && N->CandidateSet) {
            Candidate *NN;
            for (NN = N->CandidateSet; NN->To; NN++) {
                if (!NN->To->LastV && FixedOrCommon(N, NN->To)) {
                    Follow(NN->To, N);
                    break;
                }
            }
        }
    } while ((N = N->Suc) != FirstNode);

    Cost = 0;
    N = FirstNode;
    do
        if (!Fixed(N, N->Suc))
            Cost += Distance(N, N->Suc);
    while ((N = N->Suc) != FirstNode);
    if (TraceLevel >= 1) {
        printff(GainFormat, Cost);
        if (Optimum != MINUS_INFINITY && Optimum != 0)
            printff(", Gap = %0.1f%%", 100.0 * (Cost - Optimum) / Optimum);
        printff(", Time = %0.2f sec.\n", fabs(GetTime() - EntryTime));
    }
    return Cost;
}

static int SierpinskiIndex(double x, double y)
{
    int idx = 0;
    double oldx;
    int i = 8 * sizeof(int);

    if (x > y) {
        idx = 1;
        x = 1 - x;
        y = 1 - y;
    }
    while (--i > 0) {
        idx *= 2;
        if (x + y > 1) {
            idx++;
            oldx = x;
            x = 1 - y;
            y = oldx;
        }
        if (--i <= 0)
            break;
        x *= 2;
        y *= 2;
        idx *= 2;
        if (y > 1) {
            idx++;
            oldx = x;
            x = y - 1;
            y = 1 - oldx;
        }
    }
    return idx;
}

static int MooreIndex(double x, double y)
{
    static const int Rank[5][4] =
        { {1, 0, 2, 3}, {2, 3, 1, 0}, {2, 1, 3, 0}, {0, 3, 1, 2},
    {0, 1, 3, 2}
    };
    static const int NextState[5][4] =
        { {2, 2, 3, 3}, {1, 3, 1, 2}, {2, 2, 4, 1}, {4, 1, 3, 3},
    {3, 4, 2, 4}
    };
    int idx = 0, State = 0;
    int i = 4 * sizeof(int);

    while (--i > 0) {
        int ax = x >= 0.5;
        int ay = y >= 0.5;
        int aq = (ax << 1) + ay;
        idx = 4 * idx + Rank[State][aq];
        State = NextState[State][aq];
        x = 2 * x - ax;
        y = 2 * y - ay;
    }
    return idx;
}

static int compare(const void *Na, const void *Nb)
{
    int NaV = (*(Node **) Na)->V;
    int NbV = (*(Node **) Nb)->V;
    return NaV < NbV ? -1 : NaV > NbV ? 1 : 0;
}
