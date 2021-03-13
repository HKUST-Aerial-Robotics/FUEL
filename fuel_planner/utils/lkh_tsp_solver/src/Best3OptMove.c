#include "Segment.h"
#include "LKH.h"

/*
 * The Best3OptMove function makes sequential edge exchanges. If possible, it 
 * makes an  r-opt move (r <= 3) that improves the tour. Otherwise, it makes 
 * the most promising 3-opt move that fulfils the positive gain criterion. 
 * To prevent an infinity chain of moves the last edge in a 3-opt move must 
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
 * Otherwise, the best 3-opt move is made, and a pointer to the node that was 
 * connected to t1 (in order to close the tour) is returned. The new 
 * accumulated gain is made available to the caller through the parameter G0. 
 *
 * The function is called from the LinKernighan function. 
 */

/* 
   The algorithm splits the set of possible moves up into a number disjoint 
   subsets (called "cases"). When t1, t2, ..., t6 has been chosen, Case6 is 
   used to discriminate between 8 cases.
 
   A description of the cases is given after the code.   
*/

Node *Best3OptMove(Node * t1, Node * t2, GainType * G0, GainType * Gain)
{
    Node *t3, *t4, *t5, *t6, *T3 = 0, *T4 = 0, *T5 = 0, *T6 = 0;
    Candidate *Nt2, *Nt4;
    GainType G1, G2, G3, G4, BestG4 = MINUS_INFINITY;
    int Case6, BestCase6 = 0, X4, X6;
    int Breadth2 = 0, Breadth4;

    if (SUC(t1) != t2)
        Reversed ^= 1;

    /* 
     * Determine (T3,T4,T5,T6) = (t3,t4,t5,t6)
     * such that 
     *
     *     G4 = *G0 - C(t2,T3) + C(T3,T4) 
     *              - C(T4,T5) + C(T5,T6)
     *
     * is maximum (= BestG4), and (T5,T6) has not previously been included.
     * If during this process a legal move with *Gain > 0 is found, then make
     * the move and exit Best3OptMove immediately. 
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
                     ProblemType != HCP && ProblemType != HPP) ||
                    (X4 == 2 && !BETWEEN(t2, t5, t3)))
                    continue;
                if (++Breadth4 > MaxBreadth)
                    break;
                /* Choose t6 as one of t5's two neighbors on the tour */
                for (X6 = 1; X6 <= X4; X6++) {
                    if (X4 == 1) {
                        Case6 = 1 + !BETWEEN(t2, t5, t4);
                        t6 = Case6 == 1 ? SUC(t5) : PRED(t5);
                    } else {
                        Case6 = 4 + X6;
                        t6 = X6 == 1 ? SUC(t5) : PRED(t5);
                        if (t6 == t1)
                            continue;
                    }
                    if (FixedOrCommon(t5, t6))
                        continue;
                    G4 = G3 + C(t5, t6);
                    if (!Forbidden(t6, t1) &&
                        (!c || G4 - c(t6, t1) > 0) &&
                        (*Gain = G4 - C(t6, t1)) > 0) {
                        Make3OptMove(t1, t2, t3, t4, t5, t6, Case6);
                        return 0;
                    }
                    if (GainCriterionUsed && G4 - Precision < t6->Cost)
                        continue;
                    if (!Backtracking || Swaps > 0) {
                        if ((G4 > BestG4 ||
                             (G4 == BestG4 && !Near(t5, t6) &&
                              Near(T5, T6))) &&
                            Swaps < MaxSwaps &&
                            Excludable(t5, t6) && !InInputTour(t5, t6)) {
                            /* Ignore the move if the gain does not vary */
                            if (RestrictedSearch &&
                                ProblemType != HCP &&
                                ProblemType != HPP &&
                                G2 - t4->Pi == G4 - t6->Pi &&
                                G3 + t5->Pi == G1 + t3->Pi)
                                continue;
                            T3 = t3;
                            T4 = t4;
                            T5 = t5;
                            T6 = t6;
                            BestCase6 = Case6;
                            BestG4 = G4;
                        }
                    } else if (MaxSwaps > 0) {
                        GainType G = G4;
                        Node *t = t6;
                        Make3OptMove(t1, t2, t3, t4, t5, t6, Case6);
                        Exclude(t1, t2);
                        Exclude(t3, t4);
                        Exclude(t5, t6);
                        while ((t = BestSubsequentMove(t1, t, &G, Gain)));
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
    *Gain = 0;
    if (T6) {
        /* Make the best 3-opt move */
        Make3OptMove(t1, t2, T3, T4, T5, T6, BestCase6);
        Exclude(t1, t2);
        Exclude(T3, T4);
        Exclude(T5, T6);
        *G0 = BestG4;
    }
    return T6;
}

/*
   Below is shown the use of the variables X4 and Case6 to discriminate between 
   the 4 cases considered by the algorithm. 

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
       Case6 = 2:   
           12-43-65-
   X4 = 2:
       12-34-
       Case6 = 5: 
           12-56-34-
       Case6 = 6: 
           12-65-34-
*/
