#include "Segment.h"
#include "LKH.h"

/*
 * The Best5OptMove function makes sequential edge exchanges. If possible, it
 * makes an r-opt move (r <= 5) that improves the tour. Otherwise, it makes
 * the most promising 5-opt move that fulfils the positive gain criterion.
 * To prevent an infinity chain of moves the last edge in a 5-opt move must
 * not previously have been included in the chain.
 *
 * The edge (t1,t2) is the first edge to be exchanged. G0 is a pointer to the
 * accumulated gain.
 *
 * In case a r-opt move is found that improves the tour, the improvement of
 * the cost is made available to the caller through the parameter Gain.
 * If *Gain > 0, an improvement of the current tour has been found. In this
 * case the function returns 0.
 *
 * Otherwise, the best 5-opt move is made, and a pointer to the node that was
 * connected to t1 (in order to close the tour) is returned. The new
 * accumulated gain is made available to the caller through the parameter G0.
 *
 * The function is called from the LinKernighan function.
 */

/* 
   The algorithm splits the set of possible moves up into a number disjoint
   subsets (called "cases"). When t1, t2, ..., t6 has been chosen, Case6 is
   used to discriminate between 8 cases. When t1, t2, ..., t8 has been chosen,
   Case8 is used to discriminate between 16 cases. When t1, t2, ..., t10 has
   been chosen, Case10 is used to discriminate between 52 cases.

   A description of the cases is given after the code.
*/

Node *Best5OptMove(Node * t1, Node * t2, GainType * G0, GainType * Gain)
{
    Node *t3, *t4, *t5, *t6 = 0, *t7, *t8 = 0, *t9 = 0, *t10 = 0;
    Node *T3 = 0, *T4 = 0, *T5 = 0, *T6 = 0, *T7 = 0, *T8 = 0, *T9 = 0,
        *T10 = 0;
    Candidate *Nt2, *Nt4, *Nt6, *Nt8;
    GainType G1, G2, G3, G4, G5, G6, G7, G8, BestG8 = MINUS_INFINITY;
    int Case6 = 0, Case8 = 0, Case10 = 0, BestCase10 = 0, X4, X6, X8, X10,
        BTW275 = 0, BTW674 = 0, BTW571 = 0, BTW376 = 0, BTW574 = 0,
        BTW671 = 0, BTW471 = 0, BTW673 = 0, BTW573 = 0, BTW273 = 0;
    int Breadth2 = 0, Breadth4, Breadth6, Breadth8;

    if (t2 != SUC(t1))
        Reversed ^= 1;

    /* Determine (T3,T4,T5,T6,T7,T8,T9,T10) = (t3,t4,t5,t6,t7,t8,t9,t10)
       such that

       G8 = *G0 - C(t2,T3) + C(T3,T4) - C(T4,T5) + C(T5,T6) - C(T6,T7) +
       C(T7,T8) - C(T8,T9) + C(T9,T10)

       is maximum (= BestG8), and (T9,T10) has not previously been included.
       If during this process a legal move with *Gain > 0 is found, then
       make the move and exit Best5OptMove immediately. */

    /* Choose (t2,t3) as a candidate edge emanating from t2 */
    for (Nt2 = t2->CandidateSet; (t3 = Nt2->To); Nt2++) {
        if (t3 == t2->Pred || t3 == t2->Suc ||
            ((G1 = *G0 - Nt2->Cost) <= 0 && GainCriterionUsed &&
             ProblemType != HCP && ProblemType != HPP))
            continue;
        if (++Breadth2 > MaxBreadth)
            break;
        /* Choose t4 as one of t3's two neighbors on the tour */
        for (X4 = ProblemType == ATSP ? 2 : 1; X4 <= 2; X4++) {
            t4 = X4 == 1 ? PRED(t3) : SUC(t3);
            if (FixedOrCommon(t3, t4))
                continue;
            G2 = G1 + C(t3, t4);
            if (X4 == 1 && !Forbidden(t4, t1) &&
                (!c || G2 - c(t4, t1) > 0) && (*Gain = G2 - C(t4, t1)) > 0)
            {
                Make2OptMove(t1, t2, t3, t4);
                return 0;
            }
            if (Backtracking && !Excludable(t3, t4))
                continue;
            Breadth4 = 0;
            /* Choose (t4,t5) as a candidate edge emanating from t4 */
            for (Nt4 = t4->CandidateSet; (t5 = Nt4->To); Nt4++) {
                if (t5 == t4->Pred || t5 == t4->Suc ||
                    ((G3 = G2 - Nt4->Cost) <= 0 && GainCriterionUsed &&
                     ProblemType != HCP && ProblemType != HPP))
                    continue;
                if (++Breadth4 > MaxBreadth)
                    break;
                /* Choose t6 as one of t5's two neighbors on the tour */
                for (X6 = 1; X6 <= 2; X6++) {
                    if (X4 == 1) {
                        if (X6 == 1) {
                            Case6 = 1 + !BETWEEN(t2, t5, t4);
                            t6 = Case6 == 1 ? SUC(t5) : PRED(t5);
                        } else {
                            t6 = t6 == t5->Pred ? t5->Suc : t5->Pred;
                            if ((t5 == t1 && t6 == t2) ||
                                (t5 == t2 && t6 == t1))
                                continue;
                            Case6 += 2;
                        }
                    } else if (BETWEEN(t2, t5, t3)) {
                        Case6 = 4 + X6;
                        t6 = X6 == 1 ? SUC(t5) : PRED(t5);
                        if (t6 == t1)
                            continue;
                    } else {
                        Case6 = 6 + X6;
                        t6 = X6 == 1 ? PRED(t5) : SUC(t5);
                        if (t6 == t2)
                            continue;
                    }
                    if (FixedOrCommon(t5, t6))
                        continue;
                    G4 = G3 + C(t5, t6);
                    if ((Case6 <= 2 || Case6 == 5 || Case6 == 6) &&
                        !Forbidden(t6, t1) &&
                        (!c || G4 - c(t6, t1) > 0) &&
                        (*Gain = G4 - C(t6, t1)) > 0) {
                        Make3OptMove(t1, t2, t3, t4, t5, t6, Case6);
                        return 0;
                    }
                    if (Backtracking && !Excludable(t5, t6))
                        continue;
                    Breadth6 = 0;
                    /* Choose (t6,t7) as a candidate edge emanating from t6 */
                    for (Nt6 = t6->CandidateSet; (t7 = Nt6->To); Nt6++) {
                        if (t7 == t6->Pred || t7 == t6->Suc ||
                            (t6 == t2 && t7 == t3) ||
                            (t6 == t3 && t7 == t2) ||
                            ((G5 = G4 - Nt6->Cost) <= 0 &&
                             GainCriterionUsed &&
                             ProblemType != HCP && ProblemType != HPP))
                            continue;
                        if (++Breadth6 > MaxBreadth)
                            break;
                        /* Choose t8 as one of t7's two neighbors on the tour */
                        for (X8 = 1; X8 <= 2; X8++) {
                            if (X8 == 1) {
                                Case8 = Case6;
                                switch (Case6) {
                                case 1:
                                    if ((BTW275 = BETWEEN(t2, t7, t5)))
                                        t8 = SUC(t7);
                                    else {
                                        t8 = PRED(t7);
                                        BTW674 = BETWEEN(t6, t7, t4);
                                    }
                                    break;
                                case 2:
                                    if ((BTW376 = BETWEEN(t3, t7, t6)))
                                        t8 = SUC(t7);
                                    else {
                                        t8 = PRED(t7);
                                        BTW571 = BETWEEN(t5, t7, t1);
                                    }
                                    break;
                                case 3:
                                    t8 = SUC(t7);
                                    BTW574 = BETWEEN(t5, t7, t4);
                                    break;
                                case 4:
                                    if ((BTW671 = BETWEEN(t6, t7, t1)))
                                        t8 = PRED(t7);
                                    else
                                        t8 = BETWEEN(t2, t7,
                                                     t4) ? SUC(t7) :
                                            PRED(t7);
                                    break;
                                case 5:
                                    t8 = PRED(t7);
                                    BTW471 = BETWEEN(t4, t7, t1);
                                    if (!BTW471)
                                        BTW673 = BETWEEN(t6, t7, t3);
                                    break;
                                case 6:
                                    if ((BTW471 = BETWEEN(t4, t7, t1)))
                                        t8 = PRED(t7);
                                    else {
                                        t8 = SUC(t7);
                                        BTW573 = BETWEEN(t5, t7, t3);
                                    }
                                    break;
                                case 7:
                                case 8:
                                    t8 = SUC(t7);
                                    BTW273 = BETWEEN(t2, t7, t3);
                                    break;
                                }
                            } else {
                                t8 = t8 == t7->Pred ? t7->Suc : t7->Pred;
                                Case8 += 8;
                            }
                            if ((t7 == t1 && t8 == t2) ||
                                (t7 == t2 && t8 == t1) ||
                                (t7 == t3 && t8 == t4) ||
                                (t7 == t4 && t8 == t3))
                                continue;
                            if (FixedOrCommon(t7, t8))
                                continue;
                            if (Case6 == 3 && !BTW574 &&
                                (X8 == 1) == BETWEEN(t3, t7, t1))
                                continue;
                            if (Case6 == 4 && BTW671 && X8 == 2)
                                break;
                            if (Case6 == 7 && !BTW273 &&
                                (X8 == 1) == BETWEEN(t5, t7, t1))
                                continue;
                            if (Case6 == 8 && !BTW273 &&
                                !BETWEEN(t4, t7, t5))
                                break;
                            G6 = G5 + C(t7, t8);
                            if (t8 != t1 &&
                                (Case6 == 3 ? BTW574 :
                                 Case6 == 4 ? !BTW671 :
                                 Case6 == 7 ? BTW273 :
                                 Case6 != 8 && X8 == 1) &&
                                !Forbidden(t8, t1) &&
                                (!c || G6 - c(t8, t1) > 0) &&
                                (*Gain = G6 - C(t8, t1)) > 0) {
                                Make4OptMove(t1, t2, t3, t4, t5, t6, t7,
                                             t8, Case8);
                                return 0;
                            }
                            if (Backtracking && !Excludable(t7, t8))
                                continue;
                            Breadth8 = 0;
                            /* Choose (t8,t9) as a candidate edge emanating 
                               from t8 */
                            for (Nt8 = t8->CandidateSet; (t9 = Nt8->To);
                                 Nt8++) {
                                if (t9 == t8->Pred || t9 == t8->Suc ||
                                    t9 == t1 ||
                                    (t8 == t2 && t9 == t3) ||
                                    (t8 == t3 && t9 == t2) ||
                                    (t8 == t4 && t9 == t5) ||
                                    (t8 == t5 && t9 == t4) ||
                                    ((G7 = G6 - Nt8->Cost) <= 0 &&
                                     GainCriterionUsed &&
                                     ProblemType != HCP &&
                                     ProblemType != HPP))
                                    continue;
                                if (++Breadth8 > MaxBreadth)
                                    break;
                                /* Choose t10 as one of t9's two neighbors
                                   on the tour */
                                for (X10 = 1; X10 <= 2; X10++) {
                                    if (X10 == 1) {
                                        t10 = 0;
                                        switch (Case8) {
                                        case 1:
                                            t10 = (BTW275 ?
                                                   BETWEEN(t8, t9, t5)
                                                   || BETWEEN(t3, t9,
                                                              t1) : BTW674
                                                   ? BETWEEN(t7, t9,
                                                             t1) :
                                                   BETWEEN(t7, t9,
                                                           t5)) ? PRED(t9)
                                                : SUC(t9);
                                            Case10 = 22;
                                            break;
                                        case 2:
                                            t10 = (BTW376 ?
                                                   BETWEEN(t8, t9, t4) :
                                                   BTW571 ?
                                                   BETWEEN(t7, t9, t1)
                                                   || BETWEEN(t3, t9,
                                                              t6) :
                                                   BETWEEN(t7, t9,
                                                           t1)) ? PRED(t9)
                                                : SUC(t9);
                                            Case10 = 23;
                                            break;
                                        case 3:
                                            if (BTW574) {
                                                t10 = BETWEEN(t5, t9, t1) ?
                                                    PRED(t9) : SUC(t9);
                                                Case10 = 24;
                                                break;
                                            }
                                            if (!BETWEEN(t5, t9, t4))
                                                break;
                                            t10 = SUC(t9);
                                            Case10 = 1;
                                            break;
                                        case 4:
                                            if (BTW671) {
                                                if (!BETWEEN(t2, t9, t5))
                                                    break;
                                                t10 = SUC(t9);
                                                Case10 = 2;
                                                break;
                                            }
                                            t10 = BETWEEN(t6, t9, t4) ?
                                                PRED(t9) : SUC(t9);
                                            Case10 = 25;
                                            break;
                                        case 5:
                                            t10 = (BTW471 ?
                                                   BETWEEN(t7, t9, t1) :
                                                   BTW673 ?
                                                   BETWEEN(t7, t9, t5) :
                                                   BETWEEN(t4, t9, t1)
                                                   || BETWEEN(t7, t9,
                                                              t5)) ?
                                                PRED(t9) : SUC(t9);
                                            Case10 = 26;
                                            break;
                                        case 6:
                                            t10 = (BTW471 ?
                                                   BETWEEN(t7, t9, t3) :
                                                   BTW573 ?
                                                   BETWEEN(t8, t9, t6) :
                                                   BETWEEN(t4, t9, t1)
                                                   || BETWEEN(t8, t9,
                                                              t6)) ?
                                                PRED(t9) : SUC(t9);
                                            Case10 = 27;
                                            break;
                                        case 7:
                                            if (BTW273) {
                                                t10 = BETWEEN(t5, t9, t3) ?
                                                    PRED(t9) : SUC(t9);
                                                Case10 = 28;
                                                break;
                                            }
                                            if (!BETWEEN(t2, t9, t3))
                                                break;
                                            t10 = SUC(t9);
                                            Case10 = 3;
                                            break;
                                        case 8:
                                            if (BTW273) {
                                                if (!BETWEEN(t4, t9, t5))
                                                    break;
                                                Case10 = 4;
                                            } else {
                                                if (!BETWEEN(t2, t9, t3))
                                                    break;
                                                Case10 = 5;
                                            }
                                            t10 = SUC(t9);
                                            break;
                                        case 9:
                                            if (BTW275) {
                                                if (!BETWEEN(t7, t9, t4))
                                                    break;
                                                t10 = SUC(t9);
                                                Case10 = 6;
                                                break;
                                            }
                                            if (!BTW674) {
                                                if (!BETWEEN(t2, t9, t7))
                                                    break;
                                                t10 = SUC(t9);
                                                Case10 = 7;
                                                break;
                                            }
                                            if (!BETWEEN(t6, t9, t7))
                                                break;
                                            t10 = SUC(t9);
                                            Case10 = 8;
                                            break;
                                        case 10:
                                            if (BTW376) {
                                                if (!BETWEEN(t7, t9, t6))
                                                    break;
                                                t10 = SUC(t9);
                                                Case10 = 9;
                                                break;
                                            }
                                            if (BTW571) {
                                                if (!BETWEEN(t2, t9, t7))
                                                    break;
                                                t10 = SUC(t9);
                                                Case10 = 10;
                                                break;
                                            }
                                            if (!BETWEEN(t3, t9, t6) &&
                                                !BETWEEN(t2, t9, t7))
                                                break;
                                            t10 = SUC(t9);
                                            Case10 = 11;
                                            break;
                                        case 11:
                                            if (BTW574) {
                                                t10 = BETWEEN(t3, t9, t1) ?
                                                    PRED(t9) : SUC(t9);
                                                Case10 = 29;
                                                break;
                                            }
                                            if (!BETWEEN(t5, t9, t4))
                                                break;
                                            t10 = SUC(t9);
                                            Case10 = 12;
                                            break;
                                        case 12:
                                            t10 = BETWEEN(t3, t9, t1) ?
                                                PRED(t9) : SUC(t9);
                                            Case10 = 30;
                                            break;
                                        case 13:
                                            if (BTW471) {
                                                if (!BETWEEN(t2, t9, t7))
                                                    break;
                                                t10 = SUC(t9);
                                                Case10 = 13;
                                                break;
                                            }
                                            if (BTW673) {
                                                if (!BETWEEN(t6, t9, t7))
                                                    break;
                                                t10 = SUC(t9);
                                                Case10 = 14;
                                                break;
                                            }
                                            if (!BETWEEN(t6, t9, t3) &&
                                                !BETWEEN(t2, t9, t7))
                                                break;
                                            t10 = SUC(t9);
                                            Case10 = 15;
                                            break;
                                        case 14:
                                            if (BTW471) {
                                                if (!BETWEEN(t2, t9, t7))
                                                    break;
                                                t10 = SUC(t9);
                                                Case10 = 16;
                                                break;
                                            }
                                            if (BTW573) {
                                                if (!BETWEEN(t7, t9, t3) &&
                                                    !BETWEEN(t2, t9, t6))
                                                    break;
                                                t10 = SUC(t9);
                                                Case10 = 17;
                                                break;
                                            }
                                            if (!BETWEEN(t7, t9, t6))
                                                break;
                                            t10 = SUC(t9);
                                            Case10 = 18;
                                            break;
                                        case 15:
                                            if (BTW273) {
                                                t10 = BETWEEN(t5, t9, t1) ?
                                                    PRED(t9) : SUC(t9);
                                                Case10 = 31;
                                                break;
                                            }
                                            if (!BETWEEN(t2, t9, t3))
                                                break;
                                            t10 = SUC(t9);
                                            Case10 = 19;
                                            break;
                                        case 16:
                                            if (BTW273) {
                                                if (!BETWEEN(t4, t9, t5))
                                                    break;
                                                Case10 = 20;
                                            } else {
                                                if (!BETWEEN(t2, t9, t3))
                                                    break;
                                                Case10 = 21;
                                            }
                                            t10 = SUC(t9);
                                            break;
                                        }
                                        if (!t10)
                                            break;
                                    } else {
                                        if (Case10 >= 22)
                                            continue;
                                        Case10 += 31;
                                        t10 =
                                            t10 ==
                                            t9->Pred ? t9->Suc : t9->Pred;
                                    }
                                    if (t10 == t1 ||
                                        (t9 == t3 && t10 == t4) ||
                                        (t9 == t4 && t10 == t3) ||
                                        (t9 == t5 && t10 == t6) ||
                                        (t9 == t6 && t10 == t5))
                                        continue;
                                    if (FixedOrCommon(t9, t10))
                                        continue;
                                    G8 = G7 + C(t9, t10);
                                    if (!Forbidden(t10, t1) &&
                                        (!c || G8 - c(t10, t1) > 0) &&
                                        (*Gain = G8 - C(t10, t1)) > 0) {
                                        Make5OptMove(t1, t2, t3, t4, t5,
                                                     t6, t7, t8, t9, t10,
                                                     Case10);
                                        return 0;
                                    }
                                    if (GainCriterionUsed &&
                                        G8 - Precision < t10->Cost)
                                        continue;
                                    if (!Backtracking || Swaps > 0) {
                                        if ((G8 > BestG8 ||
                                             (G8 == BestG8
                                              && !Near(t9, t10)
                                              && Near(T9, T10)))
                                            && Swaps < MaxSwaps
                                            && Excludable(t9, t10)
                                            && !InInputTour(t9, t10)) {
                                            /* Ignore the move if the gain
                                               does not vary */
                                            if (RestrictedSearch &&
                                                ProblemType != HCP &&
                                                ProblemType != HPP &&
                                                G2 - t4->Pi == G4 - t6->Pi
                                                && G4 - t6->Pi ==
                                                G6 - t8->Pi
                                                && G6 - t8->Pi ==
                                                G8 - t10->Pi
                                                && G3 + t5->Pi ==
                                                G1 + t3->Pi
                                                && G5 + t7->Pi ==
                                                G3 + t5->Pi
                                                && G7 + t9->Pi ==
                                                G5 + t7->Pi)
                                                continue;
                                            T3 = t3;
                                            T4 = t4;
                                            T5 = t5;
                                            T6 = t6;
                                            T7 = t7;
                                            T8 = t8;
                                            T9 = t9;
                                            T10 = t10;
                                            BestCase10 = Case10;
                                            BestG8 = G8;
                                        }
                                    } else if (MaxSwaps > 0) {
                                        GainType G = G8;
                                        Node *t = t10;
                                        Make5OptMove(t1, t2, t3, t4, t5,
                                                     t6, t7, t8, t9, t10,
                                                     Case10);
                                        Exclude(t1, t2);
                                        Exclude(t3, t4);
                                        Exclude(t5, t6);
                                        Exclude(t7, t8);
                                        Exclude(t9, t10);
                                        while ((t =
                                                BestSubsequentMove(t1, t,
                                                                   &G,
                                                                   Gain)));
                                        if (*Gain > 0)
                                            return 0;
                                        RestoreTour();
                                        if (t2 != SUC(t1))
                                            Reversed ^= 1;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    *Gain = 0;
    if (T10) {
        /* Make the best 5-opt move */
        Make5OptMove(t1, t2, T3, T4, T5, T6, T7, T8, T9, T10, BestCase10);
        Exclude(t1, t2);
        Exclude(T3, T4);
        Exclude(T5, T6);
        Exclude(T7, T8);
        Exclude(T9, T10);
        *G0 = BestG8;
    }
    return T10;
}

/*
   Below is shown the use of the variables X4, Case6, Case8 and Case10 to
   discriminate between the 148 cases considered by the algorithm.

   The notation

        ab-

   is used for a subtour that starts with the edge (ta,tb). For example
   the tour

        12-43-

   contains the edges (t1,t2) and (t4,t3), in that order.

   X4 = 1:
       12-43-
       Case6 = 1:
           12-56-43-
           Case8 = 1:
               12-78-56-43-, 12-56-87-43-, 12-56-43-87-
               Case10 = 22:
                   12-910-78-56-43-, 12-78-109-56-43-, 12-78-56-910-43-, 12-78-56-43-109-
                   12-910-56-87-43-, 12-56-910-87-43-, 12-56-87-109-43-, 12-56-87-43-109-
                   12-109-56-43-87-, 12-56-910-43-87-, 12-56-43-910-87-, 12-56-43-87-109-
           Case8 = 9:
               12-87-56-43-, 12-56-78-43-, 12-56-43-78-
               Case10 = 6:
                   12-87-910-56-43-, 12-87-56-910-43-
               Case10 = 7:
                   12-910-56-43-78-, 12-56-910-43-78-, 12-56-43-910-78-
               Case10 = 8:
                   12-56-910-78-43-
               Case10 = 37:
                   12-87-109-56-43-, 12-87-56-109-43-
               Case10 = 38:
                   12-109-56-43-78-, 12-56-109-43-78-, 12-56-43-109-78-
               Case10 = 39:
                   12-56-109-78-43-
       Case6 = 2:
           12-43-65-
           Case8 = 2:
               12-87-43-65-, 12-43-78-65-, 12-43-65-87-
               Case10 = 23:
                   12-910-87-43-65-, 12-87-910-43-65-, 12-87-43-910-65-, 12-87-43-65-910-
                   12-109-43-78-65-, 12-43-910-78-65-, 12-43-78-109-65-, 12-43-78-65-109-
                   12-910-43-65-87-, 12-43-109-65-87-, 12-43-65-910-87-, 12-43-65-87-109-
           Case8 = 10:
               12-78-43-65-, 12-43-87-65-, 12-43-65-78-
               Case10 = 9:
                   12-43-87-910-65-
               Case10 = 10:
                   12-910-43-65-78-, 12-43-910-65-78-, 12-43-65-910-78-
               Case10 = 11:
                   12-910-78-43-65-, 12-78-43-910-43-
               Case10 = 40:
                   12-43-87-109-65-
               Case10 = 41:
                   12-109-43-65-78-, 12-43-109-65-78-, 12-43-65-109-78-
               Case10 = 42:
                   12-109-78-43-65-, 12-78-43-109-43-
       Case6 = 3:
           12-65-43-
           Case8 = 3:
               12-78-65-43-, 12-65-78-43-
               Case10 = 24:
                   12-910-65-78-43-, 12-65-109-78-43-, 12-65-78-109-43-, 12-65-78-43-109-
               Case10 = 1:
                   12-78-65-910-43-
               Case10 = 32:
                   12-78-65-109-43
           Case8 = 11:
               12-65-87-43-, 12-65-43-87-
               Case10 = 29:
                   12-910-65-87-43-, 12-65-910-87-43-, 12-65-87-910-43-, 12-65-87-43-109-
               Case10 = 12:
                   12-65-910-43-87-
               Case10 = 43:
                   12-65-109-43-87-
       Case6 = 4:
           12-43-56-
           Case8 = 4:
               12-78-43-56, 12-43-87-56, 12-43-56-87-
               Case10 = 2:
                   12-910-43-56-87-, 12-43-910-56-87-
               Case10 = 25:
                   12-109-78-43-56-, 12-78-109-43-56-, 12-67-43-910-56-, 12-67-43-56-109-
                   12-109-43-87-56-, 12-43-910-87-56-, 12-43-87-910-56-, 12-43-87-56-109-
               Case10 = 33:
                   12-109-43-56-87-, 12-43-109-56-87-
           Case8 = 12:
               12-87-43-56-, 12-43-78-56-
           Case10 = 30:
               12-910-87-43-56-, 12-87-910-43-56-, 12-87-43-109-56-, 12-87-43-56-109-
               12-910-43-78-56-, 12-43-109-78-56-, 12-43-78-109-56-, 12-43-78-56-109-
   X4 = 2:
       12-34-
       Case6 = 5:
           12-56-34-
           Case8 = 5: 
               12-87-56-34-, 12-56-87-34-, 12-56-34-87-
               Case10 = 26:
                   12-910-87-56-34-, 12-87-109-56-34-, 12-87-56-910-34-, 12-87-56-34-109-
                   12-109-56-87-34-, 12-56-910-87-34-, 12-56-87-109-34-, 12-56-87-34-109-
                   12-910-56-34-87-, 12-56-910-34-87-, 12-56-34-910-87-, 12-56-34-87-109-
           Case8 = 13:
               12-78-56-34-, 12-56-78-34-, 12-56-34-78-
               Case10 = 13:
                   12-910-56-34-78-, 12-56-910-34-78-, 12-56-34-910-78-
               Case10 = 14:
                   12-56-910-78-34-
               Case10 = 15:
                   12-910-78-56-34-, 12-78-56-910-34-
               Case10 = 44:
                   12-109-56-34-78-, 12-56-109-34-78-, 12-56-34-109-78-
               Case10 = 45:
                   12-56-109-78-34-
               Case10 = 46:
                   12-109-78-56-34-, 12-78-56-109-34-
       Case6 = 6:
           12-65-34-
           Case8 = 6: 
               12-87-65-34-, 12-65-78-34-, 12-65-34-87-
               Case10 = 27:
                   12-910-87-65-34-, 12-87-109-65-34-, 12-87-65-910-34-, 12-87-65-34-109-
                   12-109-65-78-34-, 12-65-910-78-34-, 12-65-78-109-34-, 12-65-78-34-109-
                   12-910-65-34-87-, 12-65-910-34-87-, 12-65-34-910-87-, 12-65-34-87-109-
           Case8 = 14:
               12-87-65-34-, 12-65-87-34-, 12-65-34-78-
               Case10 = 16:
                   12-910-65-34-78-, 12-65-910-34-78-, 12-65-34-910-78-
               Case10 = 17:
                   12-910-65-87-34-, 12-65-87-910-34-
               Case10 = 18:
                   12-87-910-65-34-
               Case10 = 47:
                  12-109-65-34-78-, 12-65-109-34-78-, 12-65-34-109-78-
               Case10 = 48:
                  12-109-65-87-34-, 12-65-87-109-34-
               Case10 = 49:
                  12-87-109-65-34-
       Case6 = 7:
           12-34-65-
            Case8 = 7:
                12-78-34-65-, 12-34-78-65-
                Case10 = 28:
                    12-109-78-34-65-, 12-78-109-34-65-, 12-78-34-910-34-, 12-78-34-65-109-
                Case10 = 3:
                    12-910-34-78-65-
                Case10 = 34:
                    12-109-34-78-65-
            Case8 = 15:
                12-87-34-65-, 12-34-87-65-
                Case10 = 31:
                    12-910-87-34-65-, 12-87-910-34-65-, 12-87-34-910-65-, 12-87-34-65-109-
                Case10 = 19:
                    12-910-34-87-65-
                Case10 = 50:
                    12-109-34-87-65-
       Case6 = 8:
           12-34-56-
           Case8 = 8: 
               12-78-34-56-, 12-34-78-56-
               Case10 = 4:
                   12-78-34-910-56-
               Case10 = 5:
                   12-910-34-78-56-
               Case10 = 35:
                   12-78-34-109-56-
               Case10 = 36:
                   12-109-34-78-56-
           Case8 = 16:
               12-87-34-56-, 12-34-87-56-
               Case10 = 20:
                   12-87-34-910-56-
               Case10 = 21:
                   12-910-34-87-56-
               Case10 = 51:
                   12-87-34-109-56-
               Case10 = 52:
                   12-109-34-87-56-
*/
