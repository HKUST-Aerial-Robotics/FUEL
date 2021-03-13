#include "Segment.h"
#include "LKH.h"
#include "Sequence.h"

/*
 * The BestKOptMove function makes edge exchanges. If possible, it makes a 
 * r-opt move (r >= 2) that improves the tour. Otherwise, it makes the most 
 * promising sequential K-opt move that fulfils the positive gain criterion. 
 * To prevent an infinity chain of moves the last edge in a K-opt move must
 * not previously have been included in the chain. 
 *
 * The edge (t[1],t[2]) is the first edge to be exchanged. G0 is a pointer to 
 * the accumulated gain.
 *
 * In case a K-opt move is found that improves the tour, the improvement of 
 * the cost is made available to the caller through the parameter Gain. 
 * If *Gain > 0, an improvement of the current tour has been found. In this
 * case the function returns 0.
 *
 * Otherwise, the best K-opt move is made, and a pointer to the node that was 
 * connected to t[1] (in order to close the tour) is returned. The new 
 * accumulated gain is made available to the caller through the parameter G0. 
 *
 * The function is called from the LinKernighan function. 
 */

static GainType BestG2;

static GainType BestKOptMoveRec(int k, GainType G0);

Node *BestKOptMove(Node * t1, Node * t2, GainType * G0, GainType * Gain)
{
    K = Swaps == 0 ? MoveType : SubsequentMoveType;
    *Gain = 0;
    t[1] = t1;
    t[2] = t2;
    T[2 * K] = 0;
    BestG2 = MINUS_INFINITY;

    /* 
     * Determine (T[3],T[4], ..., T[2K]) = (t[3],t[4], ..., t[2K])
     * such that
     *
     *     G[2 * K] = *G0 - C(t[2],T[3]) + C(T[3],T[4])
     *                    - C(T[4],T[5]) + C(T[5],T[6])
     *                      ...
     *                    - C(T[2K-3],T[2K-2]) + C(T[2K-1],T[2K])
     *
     * is maximum, and (T[2K-1],T[2K]) has not previously been included.
     * If during this process a legal move with *Gain > 0 is found, then 
     * make the move and exit BestKOptMove immediately.
     */

    MarkDeleted(t1, t2);
    *Gain = BestKOptMoveRec(2, *G0);
    UnmarkDeleted(t1, t2);

    if (*Gain <= 0 && T[2 * K]) {
        int i;
        memcpy(t + 1, T + 1, 2 * K * sizeof(Node *));
        for (i = 2; i < 2 * K; i += 2)
            incl[incl[i] = i + 1] = i;
        incl[incl[1] = 2 * K] = 1;
        MakeKOptMove(K);
        for (i = 1; i < 2 * K; i += 2)
            Exclude(T[i], T[i + 1]);
        *G0 = BestG2;
        return T[2 * K];
    }
    return 0;
}

static GainType BestKOptMoveRec(int k, GainType G0)
{
    Candidate *Nt2;
    Node *t1, *t2, *t3, *t4;
    GainType G1, G2, G3, Gain;
    int X4, i;
    int Breadth2 = 0;

    t1 = t[1];
    t2 = t[i = 2 * k - 2];
    incl[incl[i] = i + 1] = i;
    incl[incl[1] = i + 2] = 1;
    /* Choose (t2,t3) as a candidate edge emanating from t2 */
    for (Nt2 = t2->CandidateSet; (t3 = Nt2->To); Nt2++) {
        if (t3 == t2->Pred || t3 == t2->Suc ||
            ((G1 = G0 - Nt2->Cost) <= 0 && GainCriterionUsed &&
             ProblemType != HCP && ProblemType != HPP) ||
            Added(t2, t3))
            continue;
        if (++Breadth2 > MaxBreadth)
            break;
        MarkAdded(t2, t3);
        t[2 * k - 1] = t3;
        G[2 * k - 2] = G1 + t3->Pi;
        /* Choose t4 as one of t3's two neighbors on the tour */
        for (X4 = 1; X4 <= 2; X4++) {
            t4 = X4 == 1 ? PRED(t3) : SUC(t3);
            if (FixedOrCommon(t3, t4) || Deleted(t3, t4))
                continue;
            t[2 * k] = t4;
            G2 = G1 + C(t3, t4);
            G3 = MINUS_INFINITY;
            if (t4 != t1 && !Forbidden(t4, t1) && !Added(t4, t1) &&
                (!c || G2 - c(t4, t1) > 0) &&
                (G3 = G2 - C(t4, t1)) > 0 && FeasibleKOptMove(k)) {
                UnmarkAdded(t2, t3);
                MakeKOptMove(k);
                return G3;
            }
            if (Backtracking && !Excludable(t3, t4))
                continue;
            MarkDeleted(t3, t4);
            G[2 * k - 1] = G2 - t4->Pi;
            if (k < K) {
                if ((Gain = BestKOptMoveRec(k + 1, G2)) > 0) {
                    UnmarkAdded(t2, t3);
                    UnmarkDeleted(t3, t4);
                    return Gain;
                }
                incl[incl[1] = 2 * k] = 1;
            }
            if (t4 != t1 && !Forbidden(t4, t1) &&
                k + 1 < NonsequentialMoveType &&
                PatchingC >= 2 && PatchingA >= 1 &&
                (Swaps == 0 || SubsequentPatching)) {
                if (G3 == MINUS_INFINITY)
                    G3 = G2 - C(t4, t1);
                if ((PatchingCRestricted ? G3 > 0 && IsCandidate(t4, t1) :
                     PatchingCExtended ? G3 > 0
                     || IsCandidate(t4, t1) : G3 > 0)
                    && (Gain = PatchCycles(k, G3)) > 0) {
                    UnmarkAdded(t2, t3);
                    UnmarkDeleted(t3, t4);
                    return Gain;
                }
            }
            UnmarkDeleted(t3, t4);
            if (k == K && t4 != t1 && t3 != t1 && G3 <= 0 &&
                !Added(t4, t1) &&
                (!GainCriterionUsed || G2 - Precision >= t4->Cost)) {
                if (!Backtracking || Swaps > 0) {
                    if ((G2 > BestG2 ||
                         (G2 == BestG2 && !Near(t3, t4) &&
                          Near(T[2 * K - 1], T[2 * K]))) &&
                        Swaps < MaxSwaps &&
                        Excludable(t3, t4) && !InInputTour(t3, t4)) {
                        if (RestrictedSearch && K > 2 &&
                            ProblemType != HCP && ProblemType != HPP) {
                            /* Ignore the move if the gain does not vary */
                            G[0] = G[2 * K - 2];
                            G[1] = G[2 * K - 1];
                            for (i = 2 * K - 3; i >= 2; i--)
                                if (G[i] != G[i % 2])
                                    break;
                            if (i < 2)
                                continue;
                        }
                        if (FeasibleKOptMove(K)) {
                            BestG2 = G2;
                            memcpy(T + 1, t + 1, 2 * K * sizeof(Node *));
                        }
                    }
                } else if (MaxSwaps > 0 && FeasibleKOptMove(K)) {
                    Node *SUCt1 = SUC(t1);
                    MakeKOptMove(K);
                    for (i = 1; i < 2 * k; i += 2) {
                        Exclude(t[i], t[i + 1]);
                        UnmarkDeleted(t[i], t[i + 1]);
                    }
                    for (i = 2; i < 2 * k; i += 2)
                        UnmarkAdded(t[i], t[i + 1]);
                    memcpy(tSaved + 1, t + 1, 2 * k * sizeof(Node *));
                    while ((t4 = BestSubsequentMove(t1, t4, &G2, &Gain)));
                    if (Gain > 0) {
                        UnmarkAdded(t2, t3);
                        return Gain;
                    }
                    RestoreTour();
                    K = k;
                    memcpy(t + 1, tSaved + 1, 2 * K * sizeof(Node *));
                    for (i = 1; i < 2 * K - 2; i += 2)
                        MarkDeleted(t[i], t[i + 1]);
                    for (i = 2; i < 2 * K; i += 2)
                        MarkAdded(t[i], t[i + 1]);
                    for (i = 2; i < 2 * K; i += 2)
                        incl[incl[i] = i + 1] = i;
                    incl[incl[1] = 2 * K] = 1;
                    if (SUCt1 != SUC(t1))
                        Reversed ^= 1;
                    T[2 * K] = 0;
                }
            }
        }
        UnmarkAdded(t2, t3);
        if (t3 == t1)
            continue;
        /* Try to delete an added edge, (_,t3) or (t3,_) */
        for (i = 2 * k - 4; i >= 2; i--) {
            if (t3 == t[i]) {
                t4 = t[i ^ 1];
                if (t4 == t1 || Forbidden(t4, t1) || FixedOrCommon(t3, t4)
                    || Added(t4, t1))
                    continue;
                G2 = G1 + C(t3, t4);
                if ((!c || G2 - c(t4, t1) > 0)
                    && (Gain = G2 - C(t4, t1)) > 0) {
                    incl[incl[i ^ 1] = 1] = i ^ 1;
                    incl[incl[i] = 2 * k - 2] = i;
                    if (FeasibleKOptMove(k - 1)) {
                        MakeKOptMove(k - 1);
                        return Gain;
                    }
                    incl[incl[i ^ 1] = i] = i ^ 1;
                }
            }
        }
        incl[1] = 2 * k;
        incl[2 * k - 2] = 2 * k - 1;
    }
    return 0;
}
