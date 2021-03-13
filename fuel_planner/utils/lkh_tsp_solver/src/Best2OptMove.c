#include "Segment.h"
#include "LKH.h"

/*
 * The Best2OptMove function makes sequential edge exchanges. If possible, it 
 * makes a 2-opt move that improves the tour. Otherwise, it makes the most 
 * promising 2-opt move that fulfils the positive gain criterion. To prevent 
 * an infinity chain of moves the last edge in a 2-opt move must not previously 
 * have been included in the chain. 
 *
 * The edge (t1,t2) is the first edge to be exchanged. G0 is a pointer to the 
 * accumulated gain.
 *
 * In case a 2-opt move is found that improves the tour, the improvement of 
 * the cost is made available to the caller through the parameter Gain. 
 * If *Gain > 0, an improvement of the current tour has been found. In this
 * case the function returns 0.
 *
 * Otherwise, the best 2-opt move is made, and a pointer to the node that was 
 * connected to t1 (in order to close the tour) is returned. The new 
 * accumulated gain is made available to the caller through the parameter G0. 
 *
 * The function is called from the LinKernighan function. 
 */

Node *Best2OptMove(Node * t1, Node * t2, GainType * G0, GainType * Gain)
{
    Node *t3, *t4, *T3 = 0, *T4 = 0;
    Candidate *Nt2;
    GainType G1, G2, BestG2 = MINUS_INFINITY;
    int Breadth2 = 0;

    if (ProblemType == ATSP)
        return 0;
    if (SUC(t1) != t2)
        Reversed ^= 1;

    /* 
     * Determine (T3,T4) = (t3,t4)
     * such that 
     *
     *     G4 = *G0 - C(t2,T3) + C(T3,T4)
     *
     * is maximum (= BestG2), and (T3,T4) has not previously been included.
     * If during this process a legal move with *Gain > 0 is found, then make
     * the move and exit Best2OptMove immediately 
     */

    /* Choose (t2,t3) as a candidate edge emanating from t2 */
    for (Nt2 = t2->CandidateSet; (t3 = Nt2->To); Nt2++) {
        if (t3 == t2->Pred || t3 == t2->Suc ||
            ((G1 = *G0 - Nt2->Cost) <= 0 && GainCriterionUsed &&
             ProblemType != HCP && ProblemType != HPP))
            continue;
        /* Choose t4 (only one choice gives a closed tour) */
        t4 = PRED(t3);
        if (FixedOrCommon(t3, t4))
            continue;
        G2 = G1 + C(t3, t4);
        if (!Forbidden(t4, t1) &&
            (!c || G2 - c(t4, t1) > 0) && (*Gain = G2 - C(t4, t1)) > 0) {
            Swap1(t1, t2, t3);
            return 0;
        }
        if (++Breadth2 > MaxBreadth)
            break;
        if (GainCriterionUsed && G2 - Precision < t4->Cost)
            continue;
        if (!Backtracking || Swaps > 0) {
            if ((G2 > BestG2 ||
                 (G2 == BestG2 && !Near(t3, t4) &&
                  Near(T3, T4))) &&
                Swaps < MaxSwaps &&
                Excludable(t3, t4) && !InInputTour(t3, t4)) {
                T3 = t3;
                T4 = t4;
                BestG2 = G2;
            }
        } else if (MaxSwaps > 0) {
            GainType G = G2;
            Node *t = t4;
            Make2OptMove(t1, t2, t3, t4);
            Exclude(t1, t2);
            Exclude(t3, t4);
            while ((t = BestSubsequentMove(t1, t, &G, Gain)));
            if (*Gain > 0)
                return 0;
            RestoreTour();
            if (t2 != SUC(t1))
                Reversed ^= 1;
        }
    }
    *Gain = 0;
    if (T4) {
        /* Make the best 2-opt move */
        Swap1(t1, t2, T3);
        Exclude(t1, t2);
        Exclude(T3, T4);
        *G0 = BestG2;
    }
    return T4;
}

/*
   Below is shown the use of the variable X4 to discriminate between 
   the 2 cases considered by the algorithm. 

   The notation

        ab-

   is used for a subtour that starts with the edge (ta,tb). For example 
   the tour 

        12-43-

   contains the edges (t1,t2) and (t4,t3), in that order. 

   X4 = 1:
       12-43-
   X4 = 2:
       12-34-
*/
