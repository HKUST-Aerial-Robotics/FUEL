#include "LKH.h"

/*
 * The Minimum1TreeCost function returns the cost of a minimum 1-tree.
 *
 * The minimum 1-tre is found by determining the minimum spanning tree and 
 * then adding an edge corresponding to the second nearest neighbor of one 
 * of the leaves of the tree (any node which has degree 1). The leaf chosen
 * is the one that has the longest second nearest neighbor distance.
 *
 * The V-value of a node is its degree minus 2. Therefore, Norm being the 
 * sum of squares of all V-values, is a measure of a minimum 1-tree/s 
 * discrepancy from a tour. If Norm is zero, then the 1-tree constitutes a 
 * tour, and an optimal tour has been found.
 */

GainType Minimum1TreeCost(int Sparse)
{
    Node *N, *N1 = 0;
    GainType Sum = 0;
    int Max = INT_MIN;

    MinimumSpanningTree(Sparse);
    N = FirstNode;
    do {
        N->V = -2;
        Sum += N->Pi;
    }
    while ((N = N->Suc) != FirstNode);
    Sum *= -2;
    while ((N = N->Suc) != FirstNode) {
        N->V++;
        N->Dad->V++;
        Sum += N->Cost;
        N->Next = 0;
    }
    FirstNode->Dad = FirstNode->Suc;
    FirstNode->Cost = FirstNode->Suc->Cost;
    do {
        if (N->V == -1) {
            Connect(N, Max, Sparse);
            if (N->NextCost > Max && N->Next) {
                N1 = N;
                Max = N->NextCost;
            }
        }
    }
    while ((N = N->Suc) != FirstNode);
    assert(N1);
    N1->Next->V++;
    N1->V++;
    Sum += N1->NextCost;
    Norm = 0;
    do
        Norm += N->V * N->V;
    while ((N = N->Suc) != FirstNode);
    if (N1 == FirstNode)
        N1->Suc->Dad = 0;
    else {
        FirstNode->Dad = 0;
        Precede(N1, FirstNode);
        FirstNode = N1;
    }
    if (Norm == 0) {
        for (N = FirstNode->Dad; N; N1 = N, N = N->Dad)
            Follow(N, N1);
        for (N = FirstNode->Suc; N != FirstNode; N = N->Suc) {
            N->Dad = N->Pred;
            N->Cost = D(N, N->Dad);
        }
        FirstNode->Suc->Dad = 0;
    }
    return Sum;
}
