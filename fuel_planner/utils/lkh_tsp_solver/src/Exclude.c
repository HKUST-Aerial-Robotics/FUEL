#include "LKH.h"

/* 
 * The Exclude function is used to register that an edge, (ta,tb), 
 * of the original tour has been excluded in a move. This is done by
 * setting the appropriate flag, OldPredExcluded or OldSucExcluded, 
 * for each of the two end nodes.
 */

void Exclude(Node * ta, Node * tb)
{
    if (ta == tb->Pred || ta == tb->Suc)
        return;
    if (ta == tb->OldPred)
        tb->OldPredExcluded = 1;
    else if (ta == tb->OldSuc)
        tb->OldSucExcluded = 1;
    if (tb == ta->OldPred)
        ta->OldPredExcluded = 1;
    else if (tb == ta->OldSuc)
        ta->OldSucExcluded = 1;
}
