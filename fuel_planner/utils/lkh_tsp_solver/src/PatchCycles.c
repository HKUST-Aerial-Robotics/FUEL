#include "Segment.h"
#include "LKH.h"
#include "Sequence.h"

/*
 * The PatchCycles function attempts to improve the tour by patching the
 * M >= 2 cycles that would appear if the move defined by t[1..2k] and
 * incl[1..2k] was made. If the composite move results in a shorter
 * tour, then the move is made, and the function returns the gain.
 *
 * On entry, Gain is the gain that could be obtained by making the non-
 * feasible move defined by t[1..2k] and incl[1..2k].
 *
 * The function tries to patch the cycles by interleaving the alternating
 * path represented by t with one or more alternating cycles.
 *
 * The function is called from BestKOptMove.
 */

static GainType PatchCyclesRec(int k, int m, int M, GainType G0);
static int ShortestCycle(int M, int k);
static int Cycle(Node * N, int k);

static int CurrentCycle, Patchwork = 0, RecLevel = 0;
#define MaxPatchwork Dimension

/*
 * The PatchCycles function tries to find a gainful move by patching the
 * cycles that would occur if the move represented by t[1..2k] and incl[1..2k]
 * was made using one one or more alternating cycles.
 * The alternating cycles are put in continuation of t, starting at 2k+1.
 */

GainType PatchCycles(int k, GainType Gain)
{
    Node *s1, *s2, *sStart, *sStop;
    GainType NewGain;
    int M, i;

    FindPermutation(k);
    M = Cycles(k);
    if (M == 1 && Gain > 0) {
        MakeKOptMove(k);
        return Gain;
    }
    if (M == 1 || M > PatchingC || k + M > NonsequentialMoveType)
        return 0;
    if (RecLevel == 0)
        Patchwork = 0;
    CurrentCycle = ShortestCycle(M, k);
    for (i = 0; i < k; i++) {
        if (cycle[p[2 * i]] != CurrentCycle)
            continue;
        sStart = t[p[2 * i]];
        sStop = t[p[2 * i + 1]];
        for (s1 = sStart; s1 != sStop; s1 = s2) {
            s2 = SUC(s1);
            if (FixedOrCommon(s1, s2))
                continue;
            if (++Patchwork > MaxPatchwork)
                return 0;
            t[2 * k + 1] = s1;
            t[2 * k + 2] = s2;
            MarkDeleted(s1, s2);
            /* Find a set of gainful alternating cycles */
            NewGain = PatchCyclesRec(k, 2, M, Gain + C(s1, s2));
            UnmarkDeleted(s1, s2);
            if (NewGain > 0)
                return NewGain;
        }
    }
    return 0;
}

static GainType PatchCyclesRec(int k, int m, int M, GainType G0)
{
    Node *s1, *s2, *s3, *s4, *s5, *s6, *S3 = 0, *S4 = 0;
    Candidate *Ns2, *Ns4;
    GainType G1, G2, G3, G4, Gain, CloseUpGain,
        BestCloseUpGain = PatchingAExtended ? MINUS_INFINITY : 0;
    int X4, X6;
    int i, NewCycle, *cycleSaved = 0, *pSaved = 0;
    int Breadth2 = 0, Breadth4;

    s1 = t[2 * k + 1];
    s2 = t[i = 2 * (k + m) - 2];
    incl[incl[i] = i + 1] = i;

    /* Choose (s2,s3) as a candidate edge emanating from s2 */
    for (Ns2 = s2->CandidateSet; (s3 = Ns2->To); Ns2++) {
        if (s3 == s2->Pred || s3 == s2->Suc || Added(s2, s3) ||
            (NewCycle = Cycle(s3, k)) == CurrentCycle)
            continue;
        if (++Breadth2 > MaxBreadth)
            break;
        MarkAdded(s2, s3);
        t[2 * (k + m) - 1] = s3;
        G1 = G0 - Ns2->Cost;
        /* Choose s4 as one of s3's two neighbors on the tour */
        for (X4 = 1; X4 <= 2; X4++) {
            s4 = X4 == 1 ? s3->Pred : s3->Suc;
            if (FixedOrCommon(s3, s4) || Deleted(s3, s4))
                continue;
            MarkDeleted(s3, s4);
            t[2 * (k + m)] = s4;
            G2 = G1 + C(s3, s4);
            if (M > 2) {
                if (!cycleSaved) {
                    cycleSaved = (int *) malloc(2 * k * sizeof(int));
                    memcpy(cycleSaved, cycle + 1, 2 * k * sizeof(int));
                }
                for (i = 1; i <= 2 * k; i++)
                    if (cycle[i] == NewCycle)
                        cycle[i] = CurrentCycle;
                /* Extend the current alternating path */
                if ((Gain = PatchCyclesRec(k, m + 1, M - 1, G2)) > 0) {
                    UnmarkAdded(s2, s3);
                    UnmarkDeleted(s3, s4);
                    goto End_PatchCyclesRec;
                }
                memcpy(cycle + 1, cycleSaved, 2 * k * sizeof(int));
                if (PatchingA >= 2 && Patchwork < MaxPatchwork &&
                    k + M < NonsequentialMoveType &&
                    !Forbidden(s4, s1) &&
                    (!PatchingARestricted || IsCandidate(s4, s1))) {
                    GainType Bound = BestCloseUpGain >= 0 ||
                        IsCandidate(s4, s1) ? BestCloseUpGain : 0;
                    if ((!c || G2 - c(s4, s1) > Bound) &&
                        (CloseUpGain = G2 - C(s4, s1)) > Bound) {
                        S3 = s3;
                        S4 = s4;
                        BestCloseUpGain = CloseUpGain;
                    }
                }
            } else if (!Forbidden(s4, s1) && (!c || G2 - c(s4, s1) > 0)
                       && (Gain = G2 - C(s4, s1)) > 0) {
                incl[incl[2 * k + 1] = 2 * (k + m)] = 2 * k + 1;
                MakeKOptMove(k + m);
                UnmarkAdded(s2, s3);
                UnmarkDeleted(s3, s4);
                goto End_PatchCyclesRec;
            }
            UnmarkDeleted(s3, s4);
        }
        UnmarkAdded(s2, s3);
    }
    if (M == 2 && !PatchingCRestricted) {
        /* Try to patch the two cycles by a sequential 3-opt move */
        incl[incl[2 * (k + m)] = 2 * (k + m) + 1] = 2 * (k + m);
        incl[incl[2 * k + 1] = 2 * (k + m) + 2] = 2 * k + 1;
        Breadth2 = 0;
        /* Choose (s2,s3) as a candidate edge emanating from s2 */
        for (Ns2 = s2->CandidateSet; (s3 = Ns2->To); Ns2++) {
            if (s3 == s2->Pred || s3 == s2->Suc || Added(s2, s3))
                continue;
            if (++Breadth2 > MaxBreadth)
                break;
            t[2 * (k + m) - 1] = s3;
            G1 = G0 - Ns2->Cost;
            NewCycle = Cycle(s3, k);
            /* Choose s4 as one of s3's two neighbors on the tour */
            for (X4 = 1; X4 <= 2; X4++) {
                s4 = X4 == 1 ? s3->Pred : s3->Suc;
                if (FixedOrCommon(s3, s4) || Deleted(s3, s4))
                    continue;
                t[2 * (k + m)] = s4;
                G2 = G1 + C(s3, s4);
                Breadth4 = 0;
                /* Choose (s4,s5) as a candidate edge emanating from s4 */
                for (Ns4 = s4->CandidateSet; (s5 = Ns4->To); Ns4++) {
                    if (s5 == s4->Pred || s5 == s4->Suc || s5 == s1 ||
                        Added(s4, s5) ||
                        (NewCycle == CurrentCycle &&
                         Cycle(s5, k) == CurrentCycle))
                        continue;
                    if (++Breadth4 > MaxBreadth)
                        break;
                    G3 = G2 - Ns4->Cost;
                    /* Choose s6 as one of s5's two neighbors on the tour */
                    for (X6 = 1; X6 <= 2; X6++) {
                        s6 = X6 == 1 ? s5->Pred : s5->Suc;
                        if (s6 == s1 || Forbidden(s6, s1)
                            || FixedOrCommon(s5, s6)
                            || Deleted(s5, s6)
                            || Added(s6, s1))
                            continue;
                        G4 = G3 + C(s5, s6);
                        if ((!c || G4 - c(s6, s1) > 0) &&
                            (Gain = G4 - C(s6, s1)) > 0) {
                            if (!pSaved) {
                                pSaved = (int *) malloc(2 * k * sizeof(int));
                                memcpy(pSaved, p + 1, 2 * k * sizeof(int));
                            }
                            t[2 * (k + m) + 1] = s5;
                            t[2 * (k + m) + 2] = s6;
                            if (FeasibleKOptMove(k + m + 1)) {
                                MakeKOptMove(k + m + 1);
                                goto End_PatchCyclesRec;
                            }
                            memcpy(p + 1, pSaved, 2 * k * sizeof(int));
                            for (i = 1; i <= 2 * k; i++)
                                q[p[i]] = i;
                        }
                    }
                }
            }
        }
    }
    Gain = 0;
    if (S4) {
        int OldCycle = CurrentCycle;
        if (!pSaved) {
            pSaved = (int *) malloc(2 * k * sizeof(int));
            memcpy(pSaved, p + 1, 2 * k * sizeof(int));
        }
        t[2 * (k + m) - 1] = S3;
        t[2 * (k + m)] = S4;
        incl[incl[2 * k + 1] = 2 * (k + m)] = 2 * k + 1;
        /* Find a new alternating cycle */
        PatchingA--;
        RecLevel++;
        MarkAdded(s2, S3);
        MarkDeleted(S3, S4);
        MarkAdded(S4, s1);
        Gain = PatchCycles(k + m, BestCloseUpGain);
        UnmarkAdded(s2, S3);
        UnmarkDeleted(S3, S4);
        UnmarkAdded(S4, s1);
        RecLevel--;
        PatchingA++;
        if (Gain <= 0) {
            memcpy(cycle + 1, cycleSaved, 2 * k * sizeof(int));
            memcpy(p + 1, pSaved, 2 * k * sizeof(int));
            for (i = 1; i <= 2 * k; i++)
                q[p[i]] = i;
            CurrentCycle = OldCycle;
        }
    }

  End_PatchCyclesRec:
    free(cycleSaved);
    free(pSaved);
    return Gain;
}

/*
 * The Cycle function returns the number of the cycle containing
 * a given node, N.
 *
 * Time complexity: O(log k).
 */

static int Cycle(Node * N, int k)
{
    /* Binary search */
    int Low = 1, High = k;
    while (Low < High) {
        int Mid = (Low + High) / 2;
        if (BETWEEN(t[p[2 * Low]], N, t[p[2 * Mid + 1]]))
            High = Mid;
        else
            Low = Mid + 1;
    }
    return cycle[p[2 * Low]];
}

/*
 * The ShortestCycle function returns the number of the cycle with
 * the smallest number of nodes. Note however that if the two-level
 * list is used, the number of nodes of each cycle is only approximate
 * (for efficiency reasons).
 *
 * Time complexity: O(k + M), where M = Cycles(k).
 *
 * The function may only be called after a call of the Cycles function.
 */

static int ShortestCycle(int M, int k)
{
    int i, Cycle, MinCycle = 0;
    int *Size, MinSize = INT_MAX;

    Size = (int *) calloc(1 + M, sizeof(int));
    p[0] = p[2 * k];
    for (i = 0; i < 2 * k; i += 2)
        Size[cycle[p[i]]] += SegmentSize(t[p[i]], t[p[i + 1]]);
    for (Cycle = 1; Cycle <= M; Cycle++) {
        if (Size[Cycle] < MinSize) {
            MinSize = Size[Cycle];
            MinCycle = Cycle;
        }
    }
    free(Size);
    return MinCycle;
}
