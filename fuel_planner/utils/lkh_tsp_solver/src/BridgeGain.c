#include "Segment.h"
#include "LKH.h"

/*
 * The BridgeGain function attempts to improve the tour by making a 
 * non-sequential move. 
 * 
 * The function is called by the Gain23 function. 
 *	
 * For any nonfeasible 2-opt move that would cause the current tour to be 
 * split into two separate tours, BridgGain may be called in order to find 
 * a (nonfeasible) 2- or 3-opt move that reconnects the two separate tours 
 * into a tour which is shorter than the original one. In some cases, the 
 * second move may even be 4-opt.
 * 
 * For any nonfeasible 3-opt move that would cause the current tour to be 
 * split into two separate tours, BridgGain may be called in order to find 
 * a (nonfeasible) 2-opt move that reconnects the two separate tours into 
 * a tour which is shorter than the original one.
 *	
 * The parameters s1, s2, ..., s8 denote the end nodes of edges that are 
 * part of the nonfeasible move suggested by Gain23. The parameter Case6 
 * is used to specify the move type (Case6 = 0 when 2-opt, Case6 = 3, 4 
 * or 7 when 3- or 4-opt). The parameter G  contains the gain achieved by 
 * making the move.
 *	
 * If the composite move results in a shorter tour, then the move is made, 
 * and the function returns the gain achieved.	
 */

GainType
BridgeGain(Node * s1, Node * s2, Node * s3, Node * s4,
           Node * s5, Node * s6, Node * s7, Node * s8, int Case6,
           GainType G)
{
    Node *t1, *t2, *t3, *t4, *t5, *t6, *t7, *t8, *u2 = 0, *u3 = 0;
    Candidate *Nt2, *Nt4, *Nt6;
    GainType G0, G1, G2, G3, G4, G5, G6, Gain;
    int X4;
    int Breadth2, Breadth4, Breadth6;

    /* From the original tour select a segment (u2 --> u3) which contains 
       as few nodes as possible */
    switch (Case6) {
    case 3:
        if (2 * SegmentSize(s5, s4) <= Dimension) {
            u2 = s5;
            u3 = s4;
        } else {
            u2 = s3;
            u3 = s6;
        }
        break;
    case 4:
        if (2 * SegmentSize(s2, s5) <= Dimension) {
            u2 = s2;
            u3 = s5;
        } else {
            u2 = s6;
            u3 = s1;
        }
        break;
    case 0:
    case 7:
        if (2 * SegmentSize(s2, s3) <= Dimension) {
            u2 = s2;
            u3 = s3;
        } else {
            u2 = s4;
            u3 = s1;
        }
    }

    /* Choose t1 between u2 and u3 */
    for (t1 = u2; t1 != u3; t1 = t2) {
        /* Choose t2 as the successor of t1 */
        t2 = SUC(t1);
        if ((t1 == s1 && t2 == s2) ||
            (t1 == s2 && t2 == s1) ||
            (t1 == s3 && t2 == s4) ||
            (t1 == s4 && t2 == s3) ||
            (t1 == s5 && t2 == s6) ||
            (t1 == s6 && t2 == s5) ||
            (t1 == s7 && t2 == s8) ||
            (t1 == s8 && t2 == s7) || FixedOrCommon(t1, t2))
            continue;
        G0 = G + C(t1, t2);
        /* Choose (t2,t3) as a candidate edge emanating from t2. 
           t3 must not be between u2 and u3 */
        Breadth2 = 0;
        for (Nt2 = t2->CandidateSet; (t3 = Nt2->To); Nt2++) {
            if (t3 == t2->Pred || t3 == t2->Suc || BETWEEN(u2, t3, u3))
                continue;
            G1 = G0 - Nt2->Cost;
            if (++Breadth2 > MaxBreadth)
                break;
            /* Choose t4 as one of t3's two neighbors on the tour */
            for (X4 = 1; X4 <= 2; X4++) {
                t4 = X4 == 1 ? SUC(t3) : PRED(t3);
                if (t4 == t2 ||
                    (t3 == s1 && t4 == s2) ||
                    (t3 == s2 && t4 == s1) ||
                    (t3 == s3 && t4 == s4) ||
                    (t3 == s4 && t4 == s3) ||
                    (t3 == s5 && t4 == s6) ||
                    (t3 == s6 && t4 == s5) ||
                    (t3 == s7 && t4 == s8) ||
                    (t3 == s8 && t4 == s7) || FixedOrCommon(t3, t4))
                    continue;
                G2 = G1 + C(t3, t4);
                /* Test if an improvement can be obtained */
                if (!Forbidden(t4, t1) && (!c || G2 - c(t4, t1) > 0)
                    && (Gain = G2 - C(t4, t1)) > 0) {
                    switch (Case6) {
                    case 0:
                        if (X4 == 1)
                            Swap3(s1, s2, s4, t3, t4, t1, s1, s3, s2);
                        else
                            Swap2(t1, t2, t3, s1, s2, s3);
                        return Gain;
                    case 3:
                        if ((X4 == 1) ==
                            (!BETWEEN(s2, t1, s6) && !BETWEEN(s2, t3, s6)))
                            Swap3(s1, s2, s3, t1, t2, t3, s5, s6, s1);
                        else
                            Swap4(s1, s2, s3, t1, t2, t4, s5, s6, s1, t2,
                                  t4, t1);
                        if (s8)
                            Swap1(s7, s8, s1);
                        return Gain;
                    case 4:
                        if ((X4 == 1) ==
                            (!BETWEEN(s3, t1, s5) && !BETWEEN(s3, t3, s5)))
                            Swap3(s1, s2, s3, t1, t2, t3, s5, s6, s1);
                        else
                            Swap4(s1, s2, s3, t1, t2, t4, s5, s6, s1, t2,
                                  t4, t1);
                        if (s8)
                            Swap1(s7, s8, s1);
                        return Gain;
                    case 7:
                        if ((X4 == 1) ==
                            (!BETWEEN(s4, t1, s6) && !BETWEEN(s4, t3, s6)))
                            Swap3(s5, s6, s1, t1, t2, t3, s3, s4, s5);
                        else
                            Swap4(s5, s6, s1, t1, t2, t4, s3, s4, s5, t2,
                                  t4, t1);
                        if (s8)
                            Swap1(s7, s8, s1);
                        return Gain;
                    }
                }
                /* If BridgeGain has been called with a nonfeasible 2-opt move,
                   then try to find a 3-opt or 4-opt move which, when composed 
                   with the 2-opt move, results in an improvement of the tour */
                if (Case6 != 0)
                    continue;
                Breadth4 = 0;
                /* Choose (t4,t5) as a candidate edge emanating from t4 */
                for (Nt4 = t4->CandidateSet; (t5 = Nt4->To); Nt4++) {
                    if (t5 == t4->Pred || t5 == t4->Suc || t5 == t1
                        || t5 == t2)
                        continue;
                    /* Choose t6 as one of t5's two neighbors on the tour.
                       Only one choice! */
                    t6 = X4 == 1
                        || BETWEEN(u2, t5, u3) ? PRED(t5) : SUC(t5);
                    if ((t5 == s1 && t6 == s2) || (t5 == s2 && t6 == s1)
                        || (t5 == s3 && t6 == s4) || (t5 == s4 && t6 == s3)
                        || FixedOrCommon(t5, t6))
                        continue;
                    G3 = G2 - Nt4->Cost;
                    G4 = G3 + C(t5, t6);
                    if (!Forbidden(t6, t1) &&
                        (!c || G4 - c(t6, t1) > 0) &&
                        (Gain = G4 - C(t6, t1)) > 0) {
                        if (X4 == 1)
                            Swap4(s1, s2, s4, t3, t4, t1, s1, s3, s2, t5,
                                  t6, t1);
                        else
                            Swap3(t1, t2, t3, s1, s2, s3, t5, t6, t1);
                        return Gain;
                    }
                    if (++Breadth4 > MaxBreadth)
                        break;
                    Breadth6 = 0;
                    /* Choose (t7,t8) as a candidate edge emanating from t7.
                       Only one choice! */
                    for (Nt6 = t6->CandidateSet; (t7 = Nt6->To); Nt6++) {
                        if (t7 == t6->Pred || t7 == t6->Suc)
                            continue;
                        /* Choose t8 as one of t7's two neighbors on the tour.
                           Only one choice! */
                        if (X4 == 1)
                            t8 = (BETWEEN(u2, t5, t1) ? BETWEEN(t5, t7, t1)
                                  : BETWEEN(t2, t5, u3) ? BETWEEN(u2, t7,
                                                                  t1)
                                  || BETWEEN(t5, t7, u3) : BETWEEN(SUC(u3),
                                                                   t5,
                                                                   t3) ?
                                  BETWEEN(u2, t7, u3)
                                  || BETWEEN(t5, t7, t3) : !BETWEEN(t4, t7,
                                                                    t6)) ?
                                PRED(t7) : SUC(t7);
                        else
                            t8 = (BETWEEN(u2, t5, t1) ?
                                  !BETWEEN(u2, t7, t6)
                                  && !BETWEEN(t2, t7, u3) : BETWEEN(t2, t5,
                                                                    u3) ?
                                  !BETWEEN(t2, t7, t6) : BETWEEN(SUC(u3),
                                                                 t5,
                                                                 t4) ?
                                  !BETWEEN(SUC(u3), t7, t5)
                                  && !BETWEEN(t3, t7,
                                              PRED(u2)) : !BETWEEN(t3, t7,
                                                                   t5)) ?
                                PRED(t7) : SUC(t7);
                        if (t8 == t1
                            || (t7 == t1 && t8 == t2) || (t7 == t3
                                                          && t8 == t4)
                            || (t7 == t4 && t8 == t3) || (t7 == s1
                                                          && t8 == s2)
                            || (t7 == s2 && t8 == s1) || (t7 == s3
                                                          && t8 == s4)
                            || (t7 == s4 && t8 == s3))
                            continue;
                        if (FixedOrCommon(t7, t8) || Forbidden(t8, t1))
                            continue;
                        G5 = G4 - Nt6->Cost;
                        G6 = G5 + C(t7, t8);
                        /* Test if an improvement can be achieved */
                        if ((!c || G6 - c(t8, t1) > 0) &&
                            (Gain = G6 - C(t8, t1)) > 0) {
                            if (X4 == 1)
                                Swap4(s1, s2, s4, t3, t4, t1, s1, s3, s2,
                                      t5, t6, t1);
                            else
                                Swap3(t1, t2, t3, s1, s2, s3, t5, t6, t1);
                            Swap1(t7, t8, t1);
                            return Gain;
                        }
                        if (++Breadth6 > MaxBreadth)
                            break;
                    }
                }
            }
        }
    }
    /* No improvement has been found */
    return 0;
}
