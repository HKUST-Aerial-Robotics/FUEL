#include "LKH.h"
#include "gpx.h"

/*
 * The MergeWithTourGPX2 function attempts to find a short tour
 * by merging a given tour, T1, with another tour, T2.
 * T1 is given by the Suc pointers of its nodes.
 * T2 is given by the Next pointers of its nodes.
 *
 * The merging algorithm uses Generalized Partition Crossover 2,
 * GPX2, described in
 *
 *      R.Tinos, D. Whitley, and G. Ochoa (2017),
 *      A new generalized partition crossover for the traveling
 *      salesman problem: tunneling between local optima.
 */

GainType MergeWithTourGPX2()
{
    int NewDimension = 0;
    GainType Cost1 = 0, ShrunkCost1 = 0, ShrunkCost2 = 0, NewCost;
    Node *N, *First = 0, *Last;
    int *red, *blue, *offspring, i;

    N = FirstNode;
    do
        N->Suc->Pred = N->Next->Prev = N;
    while ((N = N->Suc) != FirstNode);
    i = 0;
    do {
        Cost1 += C(N, N->Suc) - N->Pi - N->Suc->Pi;
        if ((N->Suc == N->Prev || N->Suc == N->Next) &&
            (N->Pred == N->Prev || N->Pred == N->Next))
            N->V = 0;
        else {
            N->V = 1;
            NewDimension++;
            First = N;
        }
    } while ((N = N->Suc) != FirstNode);
    Cost1 /= Precision;
    if (NewDimension == 0)
        return Cost1;

    /* Shrink the tours. 
       OldPred and OldSuc represent the shrunken T1. 
       Prev and Next represent the shrunken T2 */
    N = First;
    Last = 0;
    do {
        if (N->V) {
            if (Last)
                (Last->OldSuc = N)->OldPred = Last;
            Last = N;
        }
    } while ((N = N->Suc) != First);
    (Last->OldSuc = First)->OldPred = Last;
    Last = 0;
    do {
        if (N->V) {
            if (Last) {
                Last->Next = N;
                if (Last != N->Prev)
                    N->Prev = Last;
            }
            Last = N;
        }
    } while ((N = N->Next) != First);
    Last->Next = First;
    if (Last != First->Prev)
        First->Prev = Last;

    n_cities = NewDimension;
    red = (int *) malloc(n_cities * sizeof(int));
    blue = (int *) malloc(n_cities * sizeof(int));
    offspring = (int *) malloc((n_cities + 1) * sizeof(int));
    Map2Node = (Node **) malloc(n_cities * sizeof(Node *));

    N = First;
    i = 0;
    do {
        Map2Node[i] = N;
        red[i] = N->Rank = i;
        i++;
        ShrunkCost1 += C(N, N->OldSuc) - N->Pi - N->OldSuc->Pi;
    } while ((N = N->OldSuc) != First);
    i = 0;
    do {
        blue[i++] = N->Rank;
        ShrunkCost2 += C(N, N->Next) - N->Pi - N->Next->Pi;
    } while ((N = N->Next) != First);
    ShrunkCost1 /= Precision;
    ShrunkCost2 /= Precision;

    /* Perform GPX2 recombination */
    NewCost = gpx(red, blue, offspring);
    
    free(red);
    free(blue);
    if (NewCost >= ShrunkCost1 || NewCost >= ShrunkCost2) {
        free(offspring);
        free(Map2Node);
        return Cost1;
    }
    offspring[n_cities] = offspring[0];
    for (i = 0; i < n_cities; i++) {
        N = Map2Node[offspring[i]];
        Node *NextN = Map2Node[offspring[i + 1]];
        N->OldSuc = NextN;
        NextN->OldPred = N;
    }
    free(offspring);
    free(Map2Node);

    /* Expand the offspring into a full tour */
    N = FirstNode;
    do
        N->Mark = 0;
    while ((N = N->Suc) != FirstNode);
    N = First;
    N->Mark = N;
    do {
        if (!N->Suc->Mark && (!N->V || !N->Suc->V))
            N->OldSuc = N->Suc;
        else if (!N->Pred->Mark && (!N->V || !N->Pred->V))
            N->OldSuc = N->Pred;
        else if (N->OldSuc->Mark)
            N->OldSuc = !N->OldPred->Mark ? N->OldPred : First;
        N->Mark = N;
    } while ((N = N->OldSuc) != First);
    
    Cost1 = 0;
    Hash = 0;
    do {
        Cost1 += C(N, N->OldSuc) - N->Pi - N->OldSuc->Pi;
        N->OldSuc->Pred = N;
        Hash ^= Rand[N->Id] * Rand[N->OldSuc->Id];
    }
    while ((N = N->Suc = N->OldSuc) != First);
    Cost1 /= Precision;
    if (TraceLevel >= 2)
        printff("GPX2: " GainFormat "\n", Cost1);
    return Cost1;
}
