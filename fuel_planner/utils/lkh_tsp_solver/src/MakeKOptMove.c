#include "Sequence.h"
#include "Segment.h"

/*
 * The MakeKOptMove function makes a K-opt move (K >= 2) using sorting by 
 * reversals.
 *   
 * Let t[1:2K] be the sequence of nodes used in the K-opt move.  
 * 
 *    {(t[2i-1],t[2i]) | 1 <= i <= K} is the set of edges to be excluded 
 *                                    from the tour. 
 
 *    {(t[2i],t[incl[2i]]) | 1 <= i <= K} is the set of edges to be 
 *                                        included in the tour.
 *   
 * And let p[1:2K] be a permutation corresponding to the sequence in which 
 * the nodes occur on the tour.
 *   
 * Then the move corresponds to sorting p by reversals. 
 *   
 * MakeKOptMove finds the minimum number of reversals and makes the 
 * corresponding series of 2-opt moves (swaps).
 *   
 * The implementation is based upon the algorithm for sorting signed 
 * permutations by reversals given in
 *   
 *    A, Bergeron,
 *    "A Very Elementary Presentation of the Hannenhalli-Pevzner Theory",
 *    Lecture Notes in Computer Science, 2089, 106-117 (2001). 
 */

static void Reverse(int i, int j);
static int Score(int Left, int Right, int K);

void MakeKOptMove(int K)
{
    int i, j, Best_i = 0, Best_j = 0, BestScore, s;

    FindPermutation(K);
  FindNextReversal:
    /* Find the oriented reversal that has maximal score */
    BestScore = -1;
    for (i = 1; i <= 2 * K - 2; i++) {
        j = q[incl[p[i]]];
        if (j >= i + 2 && (i & 1) == (j & 1) &&
            (s = i & 1 ? Score(i + 1, j, K) :
             Score(i, j - 1, K)) > BestScore) {
            BestScore = s;
            Best_i = i;
            Best_j = j;
        }
    }
    if (BestScore >= 0) {
        i = Best_i;
        j = Best_j;
        if (i & 1) {
            Swap1(t[p[i + 1]], t[p[i]], t[p[j]]);
            Reverse(i + 1, j);
        } else {
            Swap1(t[p[i - 1]], t[p[i]], t[p[j]]);
            Reverse(i, j - 1);
        }
        goto FindNextReversal;
    }
    /* No more oriented reversals. Cut a simpe hurdle, if any.
     * Note that there can be no super hurdles */
    for (i = 1; i <= 2 * K - 3; i += 2) {
        j = q[incl[p[i]]];
        if (j >= i + 3) {
            Swap1(t[p[i]], t[p[i + 1]], t[p[j]]);
            Reverse(i + 1, j - 1);
            goto FindNextReversal;
        }
    }
}

/*
 * The Reverse function reverses the sequence of elements in p[i:j].
 * The inverse permutation q is updated accordingly.
 */

static void Reverse(int i, int j)
{
    for (; i < j; i++, j--) {
        int pi = p[i];
        q[p[i] = p[j]] = i;
        q[p[j] = pi] = j;
    }
}

/*
 * The Score function computes the score of a reversal. The score is the 
 * number of oriented pairs in the resulting reversal.
 */

static int Score(int Left, int Right, int K)
{
    int Count = 0, i, j;

    Reverse(Left, Right);
    for (i = 1; i <= 2 * K - 2; i++) {
        j = q[incl[p[i]]];
        if (j >= i + 2 && (i & 1) == (j & 1))
            Count++;
    }
    Reverse(Left, Right);
    return Count;
}
