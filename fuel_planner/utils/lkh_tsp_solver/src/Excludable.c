#include "LKH.h"

/* 
 * The Excludable function is used to test if an edge, (ta,tb), 
 * of the tour may be excluded (when making a move). An edge is
 * excludable if it is on the original tour and has not previously 
 * been excluded (and inserted again) in the current series of moves.
 * If the edge is excludable, the function returns 1; otherwise 0.
 *
 * The function is called from the BestMove function in order to
 * test if the last edge to be excluded in a non-gainful r-opt move 
 * is excludable.  
 */

int Excludable(Node * ta, Node * tb)
{
    if (ta == tb->OldPred)
        return !tb->OldPredExcluded;
    if (ta == tb->OldSuc)
        return !tb->OldSucExcluded;
    return 0;
}
