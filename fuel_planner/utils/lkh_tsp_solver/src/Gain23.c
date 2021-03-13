#include "Segment.h"
#include "LKH.h"

/*
 * The Gain23 function attempts to improve a tour by making non-sequential
 * moves.
 *
 * The set of non-sequential moves considered consists of:
 *
 *  (1) any nonfeasible 2-opt move (producing two cycles) followed by any
 *      2- or 3-opt move which produces a feasible tour (by joining the
 *      two cycles);
 *
 *  (2) any nonfeasible 3-opt move (producing two cycles) followed by any
 *      2-opt move which produces a feasible tour (by joining the two
 *      cycles).
 *
 * The first and second move may in some cases be 4-opt in (2) and (1),
 * respectively. In (1) this can happen when a possible 3-opt move may
 * be extended to a nonfeasible 4-opt move. In (2) it can happen in those
 * cases where a sequential 4-opt extends a possible 3-opt move and
 * produces two cycles.
 *
 * The first move must have a positive gain. The second move is determined
 * by the BridgeGain function.
 */

/*
 The algorithm splits the set of possible moves up into a number disjoint
 subsets (called "cases"). When s1, s2, ..., s6 has been chosen, Case6 is
 used to discriminate between 7 cases. When s1, s2, ..., s8 has been
 chosen, Case8 is used to discriminate between 11 cases.
 
 A detailed description of the different cases can be found after the code.
 */

GainType Gain23()
{
    static Node *s1 = 0;
    static short OldReversed = 0;
    Node *s2, *s3, *s4, *s5, *s6 = 0, *s7, *s8 = 0, *s1Stop;
    Candidate *Ns2, *Ns4, *Ns6;
    GainType G0, G1, G2, G3, G4, G5, G6, Gain, Gain6;
    int X2, X4, X6, X8, Case6 = 0, Case8 = 0;
    int Breadth2, Breadth4, Breadth6;
    
    if (!s1 || s1->Subproblem != FirstNode->Subproblem)
        s1 = FirstNode;
    s1Stop = s1;
    for (X2 = 1; X2 <= 2; X2++) {
        Reversed = X2 == 1 ? OldReversed : (OldReversed ^= 1);
        do {
            s2 = SUC(s1);
            if (FixedOrCommon(s1, s2))
                continue;
            G0 = C(s1, s2);
            Breadth2 = 0;
            /* Choose (s2,s3) as a candidate edge emanating from s2 */
            for (Ns2 = s2->CandidateSet; (s3 = Ns2->To); Ns2++) {
                if (s3 == s2->Pred || s3 == s2->Suc)
                    continue;
                if (++Breadth2 > MaxBreadth)
                    break;
                G1 = G0 - Ns2->Cost;
                for (X4 = 1; X4 <= 2; X4++) {
                    s4 = X4 == 1 ? SUC(s3) : PRED(s3);
                    if (FixedOrCommon(s3, s4))
                        continue;
                    G2 = G1 + C(s3, s4);
                    /* Try any gainful nonfeasible 2-opt move
                     followed by a 2-, 3- or 4-opt move */
                    if (X4 == 1 && s4 != s1 && !Forbidden(s4, s1) &&
                        2 * SegmentSize(s2, s3) <= Dimension &&
                        (!c || G2 - c(s4, s1) > 0) &&
                        (G3 = G2 - C(s4, s1)) > 0 &&
                        (Gain = BridgeGain(s1, s2, s3, s4, 0, 0, 0, 0, 0,
                                           G3)) > 0)
                        return Gain;
                    if (X4 == 2 &&
                        !Forbidden(s4, s1) &&
                        (!c || G2 - c(s4, s1) > 0) &&
                        (Gain = G2 - C(s4, s1)) > 0) {
                        Swap1(s1, s2, s3);
                        return Gain;
                    }
                    if (G2 - s4->Cost <= 0)
                        continue;
                    Breadth4 = 0;
                    /* Try any gainful nonfeasible 3- or 4-opt move
                     folllowed by a 2-opt move */
                    /* Choose (s4,s5) as a candidate edge emanating from s4 */
                    for (Ns4 = s4->CandidateSet; (s5 = Ns4->To); Ns4++) {
                        if (s5 == s4->Pred || s5 == s4->Suc ||
                            (G3 = G2 - Ns4->Cost) <= 0)
                            continue;
                        if (++Breadth4 > MaxBreadth)
                            break;
                        /* Choose s6 as one of s5's two neighbors on the tour */
                        for (X6 = 1; X6 <= 2; X6++) {
                            if (X4 == 2) {
                                if (X6 == 1) {
                                    Case6 = 1 + !BETWEEN(s2, s5, s4);
                                    s6 = Case6 == 1 ? SUC(s5) : PRED(s5);
                                } else {
                                    s6 = s6 ==
                                    s5->Pred ? s5->Suc : s5->Pred;
                                    if (s5 == s1 || s6 == s1)
                                        continue;
                                    Case6 += 2;
                                }
                            } else if (BETWEEN(s2, s5, s3)) {
                                Case6 = 4 + X6;
                                s6 = X6 == 1 ? SUC(s5) : PRED(s5);
                                if (s6 == s1)
                                    continue;
                            } else {
                                if (X6 == 2)
                                    break;
                                Case6 = 7;
                                s6 = PRED(s5);
                            }
                            if (FixedOrCommon(s5, s6))
                                continue;
                            G4 = G3 + C(s5, s6);
                            Gain6 = 0;
                            if (!Forbidden(s6, s1) &&
                                (!c || G4 - c(s6, s1) > 0) &&
                                (Gain6 = G4 - C(s6, s1)) > 0) {
                                if (Case6 <= 2 || Case6 == 5 || Case6 == 6) {
                                    Make3OptMove(s1, s2, s3, s4, s5, s6,
                                                 Case6);
                                    return Gain6;
                                }
                                if ((Gain =
                                     BridgeGain(s1, s2, s3, s4, s5, s6, 0,
                                                0, Case6, Gain6)) > 0)
                                    return Gain;
                            }
                            Breadth6 = 0;
                            /* Choose (s6,s7) as a candidate edge
                             emanating from s6 */
                            for (Ns6 = s6->CandidateSet; (s7 = Ns6->To);
                                 Ns6++) {
                                if (s7 == s6->Pred || s7 == s6->Suc
                                    || (s6 == s2 && s7 == s3) || (s6 == s3
                                                                  && s7 ==
                                                                  s2)
                                    || (G5 = G4 - Ns6->Cost) <= 0)
                                    continue;
                                if (++Breadth6 > MaxBreadth)
                                    break;
                                /* Choose s8 as one of s7's two neighbors
                                 on the tour */
                                for (X8 = 1; X8 <= 2; X8++) {
                                    if (X8 == 1) {
                                        Case8 = Case6;
                                        switch (Case6) {
                                            case 1:
                                                s8 = BETWEEN(s2, s7,
                                                             s5) ? SUC(s7) :
                                                PRED(s7);
                                                break;
                                            case 2:
                                                s8 = BETWEEN(s3, s7,
                                                             s6) ? SUC(s7) :
                                                PRED(s7);
                                                break;
                                            case 3:
                                                if (BETWEEN(s5, s7, s4))
                                                    s8 = SUC(s7);
                                                else {
                                                    s8 = BETWEEN(s3, s7,
                                                                 s1) ? PRED(s7)
                                                    : SUC(s7);
                                                    Case8 = 17;
                                                }
                                                break;
                                            case 4:
                                                if (BETWEEN(s2, s7, s5))
                                                    s8 = BETWEEN(s2, s7,
                                                                 s4) ? SUC(s7)
                                                    : PRED(s7);
                                                else {
                                                    s8 = PRED(s7);
                                                    Case8 = 18;
                                                }
                                                break;
                                            case 5:
                                                s8 = PRED(s7);
                                                break;
                                            case 6:
                                                s8 = BETWEEN(s2, s7,
                                                             s3) ? SUC(s7) :
                                                PRED(s7);
                                                break;
                                            case 7:
                                                if (BETWEEN(s2, s7, s3))
                                                    s8 = SUC(s7);
                                                else {
                                                    s8 = BETWEEN(s5, s7,
                                                                 s1) ? PRED(s7)
                                                    : SUC(s7);
                                                    Case8 = 19;
                                                }
                                        }
                                    } else {
                                        if (Case8 >= 17 ||
                                            (Case6 != 3 && Case6 != 4
                                             && Case6 != 7))
                                            break;
                                        s8 = s8 ==
                                        s7->Pred ? s7->Suc : s7->Pred;
                                        Case8 += 8;
                                    }
                                    if (s8 == s1 ||
                                        (s7 == s1 && s8 == s2) ||
                                        (s7 == s3 && s8 == s4) ||
                                        (s7 == s4 && s8 == s3))
                                        continue;
                                    if (FixedOrCommon(s7, s8)
                                        || Forbidden(s8, s1))
                                        continue;
                                    G6 = G5 + C(s7, s8);
                                    if ((!c || G6 - c(s8, s1) > 0) &&
                                        (Gain = G6 - C(s8, s1)) > 0) {
                                        if (Case8 <= 15) {
                                            Make4OptMove(s1, s2, s3, s4,
                                                         s5, s6, s7, s8,
                                                         Case8);
                                            return Gain;
                                        }
                                        if (Gain > Gain6 &&
                                            (Gain =
                                             BridgeGain(s1, s2, s3, s4, s5,
                                                        s6, s7, s8, Case6,
                                                        Gain)) > 0)
                                            return Gain;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        while ((s1 = s2) != s1Stop);
    }
    return 0;
}

/*
 Below is shown the use of the variables X4, Case6 and Case8 to
 discriminate between cases considered by the algorithm.
 
 The notation
 
 ab-
 
 is used for a subtour that starts with the edge (sa,sb).
 For example, the tour
 
 12-43-
 
 contains the edges (s1,s2) and (s4,s3), in that order. A (*) signifies
 an infeasible solution. BridgeGain is called if the accumulated gain
 is possitive.
 
 X4 = 1:
 12-34-
 Case6 = 5:
 12-56-34- (*)
 Case8 = 5:
 12-87-56-34-, 12-56-87-34-, 12-56-34-87-
 Case8 = 13:
 12-78-56-34-, 12-56-78-34-, 12-56-34-78-
 Case6 = 6:
 12-65-34- (*)
 Case8 = 6:
 12-87-65-34-, 12-65-78-34-, 12-65-34-87-
 Case8 = 14:
 12-87-65-34-, 12-65-87-34-, 12-65-34-78-
 Case6 = 7:
 12-34-65- (*)
 Case8 = 7:
 12-78-34-65-
 Case8 = 15:
 12-87-34-65-
 Case8 = 19:
 12-34-87-65- (*), 12-34-78-65- (*)
 X4 = 2:
 12-43-
 Case6 = 1:
 12-56-43-
 Case8 = 1:
 12-78-56-43-, 12-56-87-43-, 12-56-43-87-
 Case6 = 2:
 12-43-65-
 Case8 = 2:
 12-87-43-65-, 12-43-78-65-, 12-43-65-87-
 Case6 = 3:
 12-65-43- (*)
 Case8 = 3:
 12-65-78-43-
 Case8 = 11:
 12-65-87-43-
 Case8 = 17:
 12-78-65-43- (*), 12-65-43-87- (*)
 Case6 = 4:
 12-43-56- (*)
 Case8 = 4:
 12-78-43-56-, 12-43-87-56-
 Case8 = 12:
 12-87-43-56-, 12-43-78-56-
 Case8 = 18:
 12-87-43-56- (*), 12-43-87-56- (*)
 */
