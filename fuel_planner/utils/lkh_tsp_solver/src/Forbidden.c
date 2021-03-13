#include "LKH.h"

/* 
 * The Forbidden function is used to test if an edge, (ta,tb), 
 * is one of the forbidden edges (C(ta, tb) == M) in a solution of
 * an asymmetric traveling saleman problem. 
 * If the edge is forbidden, the function returns 1; otherwise 0.
 */

int Forbidden(const Node * ta, const Node * tb)
{
    return ProblemType == ATSP &&
        (ta->Id <= DimensionSaved) == (tb->Id <= DimensionSaved);
}
