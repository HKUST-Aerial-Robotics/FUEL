#include "Segment.h"
#include "LKH.h"

/*
 * The Best4OptMove function makes sequential edge exchanges. If possible, it 
 * makes an r-opt move (r <= 4) that improves the tour. Otherwise, it makes 
 * the most promising 4-opt move that fulfils the positive gain criterion. 
 * To prevent an infinity chain of moves the last edge in a 4-opt move must 
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
 * Otherwise, the best 4-opt move is made, and a pointer to the node that was 
 * connected to t1 (in order to close the tour) is returned. The new 
 * accumulated gain is made available to the caller through the parameter G0. 
 *
 * The function is called from the LinKernighan function. 
 */

/* 
   The algorithm splits the set of possible moves up into a number disjoint 
   subsets (called "cases"). When t1, t2, ..., t6 has been chosen, Case6 is 
   used to discriminate between 8 cases. When t1, t2, ..., t8 has been chosen, 
   Case8 is used to discriminate between 16 cases. 

   A description of the cases is given after the code.   
*/

Node *Best4OptMove(Node * t1, Node * t2, GainType * G0, GainType * Gain)
{
    Node *t3, *t4, *t5, *t6 = 0, *t7, *t8 = 0,
        *T3 = 0, *T4 = 0, *T5 = 0, *T6 = 0, *T7 = 0, *T8 = 0;
    Candidate *Nt2, *Nt4, *Nt6;
    GainType G1, G2, G3, G4, G5, G6, BestG6 = MINUS_INFINITY;
    int Case6 = 0, Case8 = 0, BestCase8 = 0, X4, X6, X8;
    int Breadth2 = 0, Breadth4, Breadth6;

    if (SUC(t1) != t2)
        Reversed ^= 1;

    /* 
     * Determine (T3,T4,T5,T6,T7,T8) = (t3,t4,t5,t6,t7,t8)
     * such that
     *
     *     G8 = *G0 - C(t2,T3) + C(T3,T4) 
     *              - C(T4,T5) + C(T5,T6)
     *              - C(T6,T7) + C(T7,T8)
     *
     * is maximum (= BestG6), and (T7,T8) has not previously been included.
     * If during this process a legal move with *Gain > 0 is found, then make
     * the move and exit Best4OptMove immediately. 
     */

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
                Swap1(t1, t2, t3);
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
                        if (X6 == 2)
                            break;
                        Case6 = 7;
                        t6 = PRED(t5);
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
                                t8 = 0;
                                switch (Case6) {
                                case 1:
                                    t8 = BETWEEN(t2, t7,
                                                 t5) ? SUC(t7) : PRED(t7);
                                    break;
                                case 2:
                                    t8 = BETWEEN(t3, t7,
                                                 t6) ? SUC(t7) : PRED(t7);
                                    break;
                                case 3:
                                    if (BETWEEN(t5, t7, t4))
                                        t8 = SUC(t7);
                                    break;
                                case 4:
                                    if (BETWEEN(t2, t7, t5))
                                        t8 = BETWEEN(t2, t7,
                                                     t4) ? SUC(t7) :
                                            PRED(t7);
                                    break;
                                case 5:
                                    t8 = PRED(t7);
                                    break;
                                case 6:
                                    t8 = BETWEEN(t2, t7,
                                                 t3) ? SUC(t7) : PRED(t7);
                                    break;
                                case 7:
                                    if (BETWEEN(t2, t7, t3))
                                        t8 = SUC(t7);
                                    break;
                                }
                                if (t8 == 0)
                                    break;
                            } else {
                                if (Case6 != 3 && Case6 != 4 && Case6 != 7)
                                    break;
                                t8 = t8 == t7->Pred ? t7->Suc : t7->Pred;
                                Case8 += 8;
                            }
                            if (t8 == t1 ||
                                (t7 == t3 && t8 == t4) ||
                                (t7 == t4 && t8 == t3))
                                continue;
                            if (FixedOrCommon(t7, t8))
                                continue;
                            G6 = G5 + C(t7, t8);
                            if (t8 != t1 &&
                                !Forbidden(t8, t1) &&
                                (!c || G6 - c(t8, t1) > 0) &&
                                (*Gain = G6 - C(t8, t1)) > 0) {
                                Make4OptMove(t1, t2, t3, t4, t5, t6, t7,
                                             t8, Case8);
                                return 0;
                            }
                            if (GainCriterionUsed &&
                                G6 - Precision < t8->Cost)
                                continue;
                            if (!Backtracking || Swaps > 0) {
                                if ((G6 > BestG6 ||
                                     (G6 == BestG6 && !Near(t7, t8) &&
                                      Near(T7, T8))) &&
                                    Swaps < MaxSwaps &&
                                    Excludable(t7, t8) &&
                                    !InInputTour(t7, t8)) {
                                    /* Ignore the move if the gain does 
                                       not vary */
                                    if (RestrictedSearch &&
                                        ProblemType != HCP &&
                                        ProblemType != HPP &&
                                        G2 - t4->Pi == G4 - t6->Pi &&
                                        G4 - t6->Pi == G6 - t8->Pi &&
                                        G3 + t5->Pi == G1 + t3->Pi &&
                                        G5 + t7->Pi == G3 + t5->Pi)
                                        continue;
                                    T3 = t3;
                                    T4 = t4;
                                    T5 = t5;
                                    T6 = t6;
                                    T7 = t7;
                                    T8 = t8;
                                    BestCase8 = Case8;
                                    BestG6 = G6;
                                }
                            } else if (MaxSwaps > 0) {
                                GainType G = G6;
                                Node *t = t8;
                                Make4OptMove(t1, t2, t3, t4, t5, t6, t7,
                                             t8, Case8);
                                Exclude(t1, t2);
                                Exclude(t3, t4);
                                Exclude(t5, t6);
                                Exclude(t7, t8);
                                while ((t =
                                        BestSubsequentMove(t1, t, &G,
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
    *Gain = 0;
    if (T8) {
        /* Make the best 4-opt move */
        Make4OptMove(t1, t2, T3, T4, T5, T6, T7, T8, BestCase8);
        Exclude(t1, t2), Exclude(T3, T4);
        Exclude(T5, T6);
        Exclude(T7, T8);
        *G0 = BestG6;
    }
    return T8;
}

/*
   Below is shown the use of the variables X4, Case6, Case8 and Case10 to 
   discriminate between the 20 cases considered by the algorithm. 

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
       Case6 = 2:   
           12-43-65-
           Case8 = 2: 
               12-87-43-65-, 12-43-78-65-, 12-43-65-87-
       Case6 = 3: 
           12-65-43-
           Case8 = 3:
               12-65-78-43-
           Case8 = 11:
               12-65-87-43-
       Case6 = 4: 
           12-43-56-
           Case8 = 4: 
               12-78-43-56, 12-43-87-56
           Case8 = 12:
               12-87-43-56-
   X4 = 2:
       12-34-
       Case6 = 5: 
           12-56-34-
           Case8 = 5: 
               12-87-56-34-, 12-56-87-34-, 12-56-34-87-
           Case8 = 13:
               12-56-87-34-
        Case6 = 6: 
           12-65-34-
           Case8 = 6: 
               12-78-65-34-, 12-65-34-87-
           Case8 = 14:
               12-65-87-34-        
       Case6 = 7: 
           12-34-65-
           Case8 = 7: 
               12-78-34-65-
           Case8 = 15:
               12-87-34-65-
*/
