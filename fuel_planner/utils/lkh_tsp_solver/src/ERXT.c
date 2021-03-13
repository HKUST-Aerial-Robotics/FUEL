#include "LKH.h"

/* 
 * The ERXT function applies the Edge Recombination Crossover operator (ERX)
 * on the two individuals (tours) represented by the Suc and Next references,
 * resepectively.
 * 
 * ERX was originally described in 
 *
 *     D. Whitley, T. Starkweather, and D. Fuquay,
 *     Scheduling Problems and the Traveling Salesman:
 *     the Genetic Edge Recombination Operator. 
 *     Proc. Third Int. Conf. on Genetic Algorithms and Their Applications
 *     (1989)
 * 
 * ERXT implements the variant of ERX based on tabu-edges (Edge-T) described in
 *
 *     Chuan-Kang Ting,
 *     Improving Edge Recombination through Alternate Inheritance and
 *     Greedy Manner.
 *     Lecture Notes in Computer Science 3004, pp. 207-216 (2004)
 *
 * However, ERXT does not implement the greedy strategy used by Edge-T for 
 * choosing foreign edges.
 */

static Node *FirstFree;
static int Tabu;

static Node *SelectNext(Node * N);

void ERXT()
{
    Node *N, *Next;
    int i;

    Tabu = 0;
    N = FirstNode;
    do {
        N->OldSuc = N->Suc;
        N->OldSuc->OldPred = N;
        N->Next->Prev = N;
        N->Suc->Pred = N;
        N->V = 0;
    }
    while ((N = N->Suc) != FirstNode);
    if (Dimension == DimensionSaved)
        FirstNode = &NodeSet[1 + Random() % Dimension];
    else {
        for (i = Random() % Dimension; i > 0; i--)
            FirstNode = FirstNode->Suc;
        if (FirstNode->Id <= DimensionSaved)
            FirstNode += DimensionSaved;
    }
    N = FirstNode;
    N->V = 1;
    FirstFree = N->Suc;
    N->Pred->Suc = N->Suc;
    N->Suc->Pred = N->Pred;
    for (i = 1; i < Dimension; i++) {
        Next = SelectNext(N);
        if (Next == FirstFree)
            FirstFree = Next->Suc;
        Next->Pred->Suc = Next->Suc;
        Next->Suc->Pred = Next->Pred;
        Link(N, Next);
        N = Next;
        N->V = 1;
    }
    Link(N, FirstNode);
}

/*
 * The EdgeCount function computes the number of unused edges emanating
 * from a given node, N.
 */

static int EdgeCount(Node * N)
{
    int Count = 0;
    Node *Next;

    if (!N->OldPred->V)
        Count++;
    if (!N->OldSuc->V)
        Count++;
    Next = N->Prev;
    if (!Next->V && Next != N->OldPred && Next != N->OldSuc)
        Count++;
    Next = N->Next;
    if (!Next->V && Next != N->OldPred && Next != N->OldSuc)
        Count++;
    return Count;
}

#define IsCommonEdge(Na, Nb)\
    (((Na)->OldPred == (Nb) || (Na)->OldSuc == (Nb)) &&\
     ((Na)->Prev == (Nb) || (Na)->Next == (Nb)))

/*
 * The SelectNext function select the next node to be added as a neighbor
 * to a given node, N.
 *
 * The function chooses the neighbor node with the highest priority (See Ting's
 * paper). If two or more possible neighbor nodes have the same priority,
 * then one of them is chosen randomly. If the node has no neighbors on the two
 * two tours, then the first node in the list of unused nodes is chosen.
 */

static Node *SelectNext(Node * N)
{
    Node *Next, *Alternative[4];
    int Alternatives = 0, Score, MaxScore = INT_MIN, i;

    for (i = 1; i <= 4; i++) {
        Next = i == 1 ? N->OldPred : i == 2 ? N->OldSuc :
            i == 3 ? N->Prev : N->Next;
        if (!Next->V &&
            (i <= 2 || (Next != N->OldPred && Next != N->OldSuc))) {
            if (Fixed(N, Next) || IsCommonEdge(N, Next))
                Score = INT_MAX;
            else
                Score = -EdgeCount(Next) - (i <= 2 ? Tabu : -Tabu);
            if (Score >= MaxScore) {
                if (Score > MaxScore)
                    Alternatives = 0;
                Alternative[Alternatives++] = Next;
                MaxScore = Score;
            }
        }
    }
    if (Alternatives > 0) {
        Next = Alternative[Random() % Alternatives];
        if (Next == N->OldPred || Next == N->OldSuc)
            Tabu++;
        if (Next == N->Prev || Next == N->Next)
            Tabu--;
        return Next;
    }
    Next = FirstFree;
    while (Forbidden(N, Next)) {
        Next = Next->Suc;
        if (Next == FirstFree)
            break;
    }
    return Next;
}
