#include "Segment.h"
#include "LKH.h"

/*
 * The Flip_SSL function performs a 2-opt move. Edges (t1,t2) and (t3,t4) 
 * are exchanged with edges (t2,t3) and (t4,t1). Node t4 is one of 
 * t3's two neighbors on the tour; which one is uniquely determined
 * by the orientation of (t1,t2).
 *
 * The function is only used if the three-level tree representation is used 
 * for a tour; if the doubly linked list representation is used, the Flip_SL
 * function is used instead.
 *
 * An average cost of O(n^(1/3)) per 2-opt move may be achieved using the 
 * three-level tree representation.
 *
 * See also the documentation for the Flip_SL function.
 */

static void SplitSegment(Node * t1, Node * t2);
static void SplitSSegment(Segment * t1, Segment * t2);
static void FlipNodes(Node * a, Node * b, Node * c, Node * d);
static void FlipSegments(Segment * a, Segment * b,
                         Segment * c, Segment * d);
static void FlipSSegments(SSegment * a, SSegment * b,
                          SSegment * c, SSegment * d);

#define SPLIT_CUTOFF 0.75

void Flip_SSL(Node * t1, Node * t2, Node * t3)
{
    Node *t4, *a, *b, *c, *d;
    Segment *P1, *P2, *P3, *P4;

    assert(t1->Pred == t2 || t1->Suc == t2);
    if (t3 == t2->Pred || t3 == t2->Suc)
        return;
    if (Groups == 1) {
        Flip(t1, t2, t3);
        return;
    }
    t4 = t2 == SUC(t1) ? PRED(t3) : SUC(t3);
    P1 = t1->Parent;
    P2 = t2->Parent;
    P3 = t3->Parent;
    P4 = t4->Parent;
    /* Split segments if needed */
    if (P1 != P3 && P2 != P4) {
        if (P1 == P2) {
            SplitSegment(t1, t2);
            P1 = t1->Parent;
            P2 = t2->Parent;
            P3 = t3->Parent;
            P4 = t4->Parent;
        }
        if (P3 == P4 && P1 != P3 && P2 != P4) {
            SplitSegment(t3, t4);
            P1 = t1->Parent;
            P2 = t2->Parent;
            P3 = t3->Parent;
            P4 = t4->Parent;
        }
    } else if ((P1 == P3
                && abs(t3->Rank - t1->Rank) > SPLIT_CUTOFF * GroupSize)
               || (P2 == P4
                   && abs(t4->Rank - t2->Rank) >
                   SPLIT_CUTOFF * GroupSize)) {
        if (P1 == P2) {
            SplitSegment(t1, t2);
            P1 = t1->Parent;
            P2 = t2->Parent;
            P3 = t3->Parent;
            P4 = t4->Parent;
        }
        if (P3 == P4) {
            SplitSegment(t3, t4);
            P1 = t1->Parent;
            P2 = t2->Parent;
            P3 = t3->Parent;
            P4 = t4->Parent;
        }
    }
    /* Check if it is possible to flip locally within a segment */
    b = 0;
    if (P1 == P3) {
        /* Either the t1 --> t3 path or the t2 --> t4 path lies 
           within one segment */
        if (t1->Rank < t3->Rank) {
            if (P1 == P2 && P1 == P4 && t2->Rank > t1->Rank) {
                a = t1;
                b = t2;
                c = t3;
                d = t4;
            } else {
                a = t2;
                b = t1;
                c = t4;
                d = t3;
            }
        } else {
            if (P1 == P2 && P1 == P4 && t2->Rank < t1->Rank) {
                a = t3;
                b = t4;
                c = t1;
                d = t2;
            } else {
                a = t4;
                b = t3;
                c = t2;
                d = t1;
            }
        }
    } else if (P2 == P4) {
        /* The t2 --> t4 path lies within one segment */
        if (t4->Rank < t2->Rank) {
            a = t3;
            b = t4;
            c = t1;
            d = t2;
        } else {
            a = t1;
            b = t2;
            c = t3;
            d = t4;
        }
    }
    if (b)
        /* Flip locally (b --> d) within a segment */
        FlipNodes(a, b, c, d);
    else {
        Segment *a, *b, *c, *d, *t1 = P1, *t2 = P2, *t3 = P3, *t4 = P4;
        SSegment *P1, *P2, *P3, *P4, *Q1;
        P1 = t1->Parent;
        P2 = t2->Parent;
        P3 = t3->Parent;
        P4 = t4->Parent;
        if (P1 != P3 && P2 != P4) {
            if (P1 == P2) {
                SplitSSegment(t1, t2);
                P1 = t1->Parent;
                P2 = t2->Parent;
                P3 = t3->Parent;
                P4 = t4->Parent;
            }
            if (P3 == P4 && P1 != P3 && P2 != P4) {
                SplitSSegment(t3, t4);
                P1 = t1->Parent;
                P2 = t2->Parent;
                P3 = t3->Parent;
                P4 = t4->Parent;
            }
        } else
            if ((P1 == P3
                 && abs(t3->Rank - t1->Rank) > SPLIT_CUTOFF * SGroupSize)
                || (P2 == P4
                    && abs(t4->Rank - t2->Rank) >
                    SPLIT_CUTOFF * SGroupSize)) {
            if (P1 == P2) {
                SplitSSegment(t1, t2);
                P1 = t1->Parent;
                P2 = t2->Parent;
                P3 = t3->Parent;
                P4 = t4->Parent;
            }
            if (P3 == P4) {
                SplitSSegment(t3, t4);
                P1 = t1->Parent;
                P2 = t2->Parent;
                P3 = t3->Parent;
                P4 = t4->Parent;
            }
        }
        b = 0;
        if (P1 == P3) {
            if (t1->Rank < t3->Rank) {
                if (P1 == P2 && P1 == P4 && t2->Rank > t1->Rank) {
                    a = t1;
                    b = t2;
                    c = t3;
                    d = t4;
                } else {
                    a = t2;
                    b = t1;
                    c = t4;
                    d = t3;
                }
            } else {
                if (P1 == P2 && P1 == P4 && t2->Rank < t1->Rank &&
                    t1->Rank - t2->Rank + 1 < Groups) {
                    a = t3;
                    b = t4;
                    c = t1;
                    d = t2;
                } else {
                    a = t4;
                    b = t3;
                    c = t2;
                    d = t1;
                }
            }
        } else if (P2 == P4) {
            if (t4->Rank < t2->Rank) {
                a = t3;
                b = t4;
                c = t1;
                d = t2;
            } else {
                a = t1;
                b = t2;
                c = t3;
                d = t4;
            }
        }
        if (b)
            /* Flip locally (b --> d) within a super segment */
            FlipSegments(a, b, c, d);
        else {
            int i;
            if (P1->Suc != P2) {
                a = t1;
                t1 = t2;
                t2 = a;
                a = t3;
                t3 = t4;
                t4 = a;
                Q1 = P1;
                P1 = P2;
                P2 = Q1;
                Q1 = P3;
                P3 = P4;
                P4 = Q1;
            }
            /* Find the sequence with the fewest segments */
            if ((i = P2->Rank - P3->Rank) <= 0)
                i += SGroups;
            if (2 * i > SGroups) {
                a = t3;
                t3 = t2;
                t2 = a;
                a = t1;
                t1 = t4;
                t4 = a;
                Q1 = P3;
                P3 = P2;
                P2 = Q1;
                Q1 = P1;
                P1 = P4;
                P4 = Q1;
            }
            /* Reverse the sequence of segments (P3 --> P1) */
            FlipSSegments(P4, P3, P2, P1);
            if (t3->Suc == t4)
                t3->Suc = t2;
            else
                t3->Pred = t2;
            if (t2->Suc == t1)
                t2->Suc = t3;
            else
                t2->Pred = t3;
            if (t1->Pred == t2)
                t1->Pred = t4;
            else
                t1->Suc = t4;
            if (t4->Pred == t3)
                t4->Pred = t1;
            else
                t4->Suc = t1;
        }
    }
    if (!b) {
        int Ct2t3 = C(t2, t3), Ct4t1 = C(t4, t1);
        if (t3->Suc == t4) {
            t3->Suc = t2;
            t3->SucCost = Ct2t3;
        } else {
            t3->Pred = t2;
            t3->PredCost = Ct2t3;
        }
        if (t2->Suc == t1) {
            t2->Suc = t3;
            t2->SucCost = Ct2t3;
        } else {
            t2->Pred = t3;
            t2->PredCost = Ct2t3;
        }
        if (t1->Pred == t2) {
            t1->Pred = t4;
            t1->PredCost = Ct4t1;
        } else {
            t1->Suc = t4;
            t1->SucCost = Ct4t1;
        }
        if (t4->Pred == t3) {
            t4->Pred = t1;
            t4->PredCost = Ct4t1;
        } else {
            t4->Suc = t1;
            t4->SucCost = Ct4t1;
        }
    }
    SwapStack[Swaps].t1 = t1;
    SwapStack[Swaps].t2 = t2;
    SwapStack[Swaps].t3 = t3;
    SwapStack[Swaps].t4 = t4;
    Swaps++;
    Hash ^= (Rand[t1->Id] * Rand[t2->Id]) ^
        (Rand[t3->Id] * Rand[t4->Id]) ^
        (Rand[t2->Id] * Rand[t3->Id]) ^ (Rand[t4->Id] * Rand[t1->Id]);
}

/*
   The SplitSegment function is called by the Flip_SSL function to split a 
   segment. Calling SplitSegment(t1,t2), where t1 and t2 are neighbors in 
   the same segment, causes the segment to be split between t1 and t2. 
   The smaller half is merged with its neighboring segment, thus keeping
   the number of segments fixed.
 */

static void SplitSegment(Node * t1, Node * t2)
{
    Segment *P = t1->Parent, *Q;
    Node *t, *u;
    int i, Count, Temp;

    if (t2->Rank < t1->Rank) {
        t = t1;
        t1 = t2;
        t2 = t;
    }
    Count = t1->Rank - P->First->Rank + 1;
    if (2 * Count < P->Size) {
        /* The left part of P is merged with its neighbouring segment, Q */
        Q = P->Reversed ? P->Suc : P->Pred;
        t = P->First->Pred;
        i = t->Rank;
        if (t == Q->Last) {
            if (t == Q->First && t->Suc != P->First) {
                u = t->Suc;
                t->Suc = t->Pred;
                t->Pred = u;
                Q->Reversed ^= 1;
                Temp = t->SucCost;
                t->SucCost = t->PredCost;
                t->PredCost = Temp;
            }
            for (t = P->First; t != t2; t = t->Suc) {
                t->Parent = Q;
                t->Rank = ++i;
            }
            Q->Last = t1;
        } else {
            for (t = P->First; t != t2; t = u) {
                t->Parent = Q;
                t->Rank = --i;
                u = t->Suc;
                t->Suc = t->Pred;
                t->Pred = u;
                Temp = t->SucCost;
                t->SucCost = t->PredCost;
                t->PredCost = Temp;
            }
            Q->First = t1;
        }
        P->First = t2;
    } else {
        /* The right part of P is merged with its neighbouring segment, Q */
        Q = P->Reversed ? P->Pred : P->Suc;
        t = P->Last->Suc;
        i = t->Rank;
        if (t == Q->First) {
            if (t == Q->Last && t->Pred != P->Last) {
                u = t->Suc;
                t->Suc = t->Pred;
                t->Pred = u;
                Q->Reversed ^= 1;
                Temp = t->SucCost;
                t->SucCost = t->PredCost;
                t->PredCost = Temp;
            }
            for (t = P->Last; t != t1; t = t->Pred) {
                t->Parent = Q;
                t->Rank = --i;
            }
            Q->First = t2;
        } else {
            for (t = P->Last; t != t1; t = u) {
                t->Parent = Q;
                t->Rank = ++i;
                u = t->Pred;
                t->Pred = t->Suc;
                t->Suc = u;
                Temp = t->SucCost;
                t->SucCost = t->PredCost;
                t->PredCost = Temp;
            }
            Q->Last = t2;
        }
        Count = P->Size - Count;
        P->Last = t1;
    }
    P->Size -= Count;
    Q->Size += Count;
}

/*
   The SplitSSegment function is called by the Flip_SSL function to split a
   super segment. Calling SplitSSegment(t1,t2), where t1 and t2 are 
   neighbors in the same super segment, causes the super segment to be split 
   between t1 and t2.
   The smaller half is merged with its neighboring super segment, thus 
   keeping the number of super segments fixed.
 */

void SplitSSegment(Segment * t1, Segment * t2)
{
    SSegment *P = t1->Parent, *Q;
    Segment *t, *u;
    int i, Count;

    if (t2->Rank < t1->Rank) {
        t = t1;
        t1 = t2;
        t2 = t;
    }
    Count = t1->Rank - P->First->Rank + 1;
    if (2 * Count < P->Size) {
        /* The left part of P is merged with its neighbouring segment, Q */
        Q = P->Reversed ? P->Suc : P->Pred;
        t = P->First->Pred;
        i = t->Rank;
        if (t == Q->Last) {
            if (t == Q->First && t->Suc != P->First) {
                u = t->Suc;
                t->Suc = t->Pred;
                t->Pred = u;
                Q->Reversed ^= 1;
                t->Reversed ^= 1;
            }
            for (t = P->First; t != t2; t = t->Suc) {
                t->Parent = Q;
                t->Rank = ++i;
            }
            Q->Last = t1;
        } else {
            for (t = P->First; t != t2; t = u) {
                t->Parent = Q;
                t->Rank = --i;
                u = t->Suc;
                t->Suc = t->Pred;
                t->Pred = u;
                t->Reversed ^= 1;
            }
            Q->First = t1;
        }
        P->First = t2;
    } else {
        /* The right part of P is merged with its neighbouring segment, Q */
        Q = P->Reversed ? P->Pred : P->Suc;
        t = P->Last->Suc;
        i = t->Rank;
        if (t == Q->First) {
            if (t == Q->Last && t->Pred != P->Last) {
                u = t->Suc;
                t->Suc = t->Pred;
                t->Pred = u;
                Q->Reversed ^= 1;
                t->Reversed ^= 1;
            }
            for (t = P->Last; t != t1; t = t->Pred) {
                t->Parent = Q;
                t->Rank = --i;
            }
            Q->First = t2;
        } else {
            for (t = P->Last; t != t1; t = u) {
                t->Parent = Q;
                t->Rank = ++i;
                u = t->Pred;
                t->Pred = t->Suc;
                t->Suc = u;
                t->Reversed ^= 1;
            }
            Q->Last = t2;
        }
        Count = P->Size - Count;
        P->Last = t1;
    }
    P->Size -= Count;
    Q->Size += Count;
}

static void FlipNodes(Node * a, Node * b, Node * c, Node * d)
{
    Node *s1, *s2 = b;
    int i = d->Rank, Temp, Cbc = C(b, c), Cda = C(d, a);

    d->Suc = 0;
    while ((s1 = s2)) {
        s2 = s1->Suc;
        s1->Suc = s1->Pred;
        s1->Pred = s2;
        s1->Rank = i--;
        Temp = s1->SucCost;
        s1->SucCost = s1->PredCost;
        s1->PredCost = Temp;
    }
    d->Pred = a;
    b->Suc = c;
    d->PredCost = Cda;
    b->SucCost = Cbc;
    if (a->Suc == b) {
        a->Suc = d;
        a->SucCost = d->PredCost;
    } else {
        a->Pred = d;
        a->PredCost = d->PredCost;
    }
    if (c->Pred == d) {
        c->Pred = b;
        c->PredCost = b->SucCost;
    } else {
        c->Suc = b;
        c->SucCost = b->SucCost;
    }
    if (b->Parent->First == b)
        b->Parent->First = d;
    else if (d->Parent->First == d)
        d->Parent->First = b;
    if (b->Parent->Last == b)
        b->Parent->Last = d;
    else if (d->Parent->Last == d)
        d->Parent->Last = b;
}

static void FlipSegments(Segment * a, Segment * b,
                         Segment * c, Segment * d)
{
    int i = d->Rank;
    Segment *s1, *s2 = b;

    d->Suc = 0;
    while ((s1 = s2)) {
        s2 = s1->Suc;
        s1->Suc = s1->Pred;
        s1->Pred = s2;
        s1->Rank = i--;
        s1->Reversed ^= 1;
    }
    d->Pred = a;
    b->Suc = c;
    if (a->Suc == b)
        a->Suc = d;
    else
        a->Pred = d;
    if (c->Pred == d)
        c->Pred = b;
    else
        c->Suc = b;
    if (b->Parent->First == b)
        b->Parent->First = d;
    else if (d->Parent->First == d)
        d->Parent->First = b;
    if (b->Parent->Last == b)
        b->Parent->Last = d;
    else if (d->Parent->Last == d)
        d->Parent->Last = b;
}

static void FlipSSegments(SSegment * a, SSegment * b,
                          SSegment * c, SSegment * d)
{
    int i = d->Rank;
    SSegment *s1, *s2 = b;

    d->Suc = 0;
    while ((s1 = s2)) {
        s2 = s1->Suc;
        s1->Suc = s1->Pred;
        s1->Pred = s2;
        s1->Rank = i--;
        s1->Reversed ^= 1;
    }
    d->Pred = a;
    b->Suc = c;
    if (a->Suc == b)
        a->Suc = d;
    else
        a->Pred = d;
    if (c->Pred == d)
        c->Pred = b;
    else
        c->Suc = b;
}
