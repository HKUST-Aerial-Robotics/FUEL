#ifndef _SEGMENT_H
#define _SEGMENT_H

#ifndef ONE_LEVEL_TREE
#ifndef TWO_LEVEL_TREE
#ifndef THREE_LEVEL_TREE
#define TWO_LEVEL_TREE
#endif
#endif
#endif

/*
 * This header specifies the interface for accessing and manipulating a
 * tour.
 *
 * If THREE_LEVEL_TREE is defined the three-level tree representation is used
 * for a tour.  If TWO_LEVEL_TREE is defined the three-level tree
 * representation is used.  Otherwise the linked list representation is used.
 *
 * All representations support the following primitive operations:
 *
 *     (1) find the predecessor of a node in the tour with respect
 *         to a chosen orientation (PRED);
 *
 *     (2) find the successor of a node in the tour with respect to
 *         a chosen orientation (SUC);
 *
 *     (3) determine whether a given node is between two other nodes
 *         in the tour with respect to a chosen orientation (BETWEEN);
 *
 *     (4) make a 2-opt move (FLIP).
 *
 * The default representation is the two-level tree representation.
 * In order to use the linked list representation, uncomment the
 * following preprocessor command line.
 */

#ifdef THREE_LEVEL_TREE
#define PRED(a)                                                                                         \
  (Reversed == ((a)->Parent->Reversed != (a)->Parent->Parent->Reversed) ? (a)->Pred : (a)->Suc)
#define SUC(a)                                                                                          \
  (Reversed == ((a)->Parent->Reversed != (a)->Parent->Parent->Reversed) ? (a)->Suc : (a)->Pred)
#define BETWEEN(a, b, c) Between_SSL(a, b, c)
#define FLIP(a, b, c, d) Flip_SSL(a, b, c)
#endif
#ifdef TWO_LEVEL_TREE
#define PRED(a) (Reversed == (a)->Parent->Reversed ? (a)->Pred : (a)->Suc)
#define SUC(a) (Reversed == (a)->Parent->Reversed ? (a)->Suc : (a)->Pred)
#define BETWEEN(a, b, c) Between_SL(a, b, c)
#define FLIP(a, b, c, d) Flip_SL(a, b, c)
#endif
#ifdef ONE_LEVEL_TREE
#define PRED(a) (Reversed ? (a)->Suc : (a)->Pred)
#define SUC(a) (Reversed ? (a)->Pred : (a)->Suc)
#define BETWEEN(a, b, c) Between(a, b, c)
#define FLIP(a, b, c, d) Flip(a, b, c)
#endif

#define Swap1(a1, a2, a3) FLIP(a1, a2, a3, 0)
#define Swap2(a1, a2, a3, b1, b2, b3) (Swap1(a1, a2, a3), Swap1(b1, b2, b3))
#define Swap3(a1, a2, a3, b1, b2, b3, c1, c2, c3) (Swap2(a1, a2, a3, b1, b2, b3), Swap1(c1, c2, c3))
#define Swap4(a1, a2, a3, b1, b2, b3, c1, c2, c3, d1, d2, d3)                                           \
  (Swap3(a1, a2, a3, b1, b2, b3, c1, c2, c3), Swap1(d1, d2, d3))
#define Swap5(a1, a2, a3, b1, b2, b3, c1, c2, c3, d1, d2, d3, e1, e2, e3)                               \
  (Swap4(a1, a2, a3, b1, b2, b3, c1, c2, c3, d1, d2, d3), Swap1(e1, e2, e3))

#endif
