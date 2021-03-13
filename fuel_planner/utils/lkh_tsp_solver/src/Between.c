#include "LKH.h"

/* 
 * The Between function is used to determine whether a node
 * is between two other nodes with respect to the current
 * orientation. The function is only used if the doubly linked
 * list representation is used for a tour; if the two-la
 * tree representation is used, the function Between_SL is used
 * instead.
 *	
 * Between(ta,tb,tc) returns 1 if node tb is between node ta and tc.
 * Otherwise, 0 is returned.
 *	
 * The function is called from the functions BestMove, Gain23,
 * BridgeGain, Make4OptMove, Make5OptMove and FindPermutation.
 */

int Between(const Node * ta, const Node * tb, const Node * tc)
{
    int a, b = tb->Rank, c;

    if (!Reversed) {
        a = ta->Rank;
        c = tc->Rank;
    } else {
        a = tc->Rank;
        c = ta->Rank;
    }
    return a <= c ? b >= a && b <= c : b >= a || b <= c;
}
