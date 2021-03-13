#include "LKH.h"

/*
 * The CreateNNCandidateSet function creates for each node
 * a candidate set consisting of the K least costly neighbor edges.
 *
 * Time complexity is O(K*N*N).
 *
 * The function is called from the CreateCandidateSet and 
 * AddExtraCandidates functions. It is intended for use on non-geometric
 * and toroidal instances.
 */

static int compareX(const void *Na, const void *Nb);

void CreateNNCandidateSet(int K)
{
    Node **XNearList, **To, *Na, *Nb;
    int *Cost, d, a, b, k, Count, Forward;

    if (TraceLevel >= 2)
        printff("Creating NN candidate set ... \n");
    XNearList = (Node **) malloc(Dimension * sizeof(Node *));
    To = (Node **) malloc((K + 1) * sizeof(Node *));
    Cost = (int *) malloc((K + 1) * sizeof(int));
    for (Na = FirstNode, k = 0; k < Dimension; Na = Na->Suc, k++)
        XNearList[k] = Na;
    qsort(XNearList, Dimension, sizeof(Node *), compareX);
    for (a = 0; a < Dimension; a++) {
        Na = XNearList[a];
        Count = 0;
        for (Forward = 0; Forward <= 1; Forward++) {
            b = Forward ? a + 1 : a - 1;
            while (b >= 0 && b < Dimension) {
                Nb = XNearList[b];
                d = Distance(Na, Nb);
                k = Count < K ? Count++ : K;
                while (k > 0 && d < Cost[k - 1]) {
                    To[k] = To[k - 1];
                    Cost[k] = Cost[k - 1];
                    k--;
                }
                To[k] = Nb;
                Cost[k] = d;
                b = Forward ? b + 1 : b - 1;
            }
        }
        for (k = 0; k < Count; k++)
            AddCandidate(Na, To[k], D(Na, To[k]), 2);
    }
    free(Cost);
    free(To);
    free(XNearList);
    if (TraceLevel >= 2)
        ;
}

static int compareX(const void *Na, const void *Nb)
{
    double x1 = (*(Node **) Na)->X;
    double y1 = (*(Node **) Na)->Y;
    double x2 = (*(Node **) Nb)->X;
    double y2 = (*(Node **) Nb)->Y;
    return x1 < x2 ? -1 : x1 > x2 ? 1 : y1 < y2 ? -1 : y1 > y2 ? 1 : 0;
}
