#include "Segment.h"
#include "LKH.h"

/*
 * The Make5OptMove function makes a 5-opt move by calling the macro Swap4 
 * or Swap5.
 */

void
Make5OptMove(Node * t1, Node * t2, Node * t3, Node * t4,
             Node * t5, Node * t6, Node * t7, Node * t8,
             Node * t9, Node * t10, int Case)
{
    if (SUC(t1) != t2)
        Reversed ^= 1;
    switch (Case) {
    case 1:
        Swap4(t1, t2, t3, t8, t7, t6, t10, t9, t8, t10, t5, t4);
        return;
    case 2:
        if (BETWEEN(t2, t9, t4))
            Swap4(t1, t2, t3, t5, t6, t7, t10, t9, t8, t5, t10, t1);
        else
            Swap4(t1, t2, t3, t7, t8, t9, t6, t5, t4, t7, t10, t1);
        return;
    case 3:
        Swap4(t3, t4, t5, t7, t8, t9, t1, t2, t3, t7, t10, t1);
        return;
    case 4:
        Swap5(t5, t6, t8, t1, t2, t3, t10, t9, t8, t1, t4, t5, t6, t10,
              t1);
        return;
    case 5:
        Swap5(t5, t6, t10, t1, t2, t3, t6, t10, t1, t8, t7, t6, t8, t4,
              t5);
        return;
    case 6:
        Swap4(t1, t2, t3, t9, t10, t1, t7, t8, t9, t6, t5, t4);
        return;
    case 7:
        if (BETWEEN(t3, t9, t7))
            Swap4(t3, t4, t5, t8, t7, t6, t10, t9, t8, t1, t2, t3);
        else if (BETWEEN(t6, t9, t4))
            Swap4(t3, t4, t5, t8, t7, t6, t9, t10, t1, t9, t2, t3);
        else
            Swap4(t1, t2, t3, t6, t5, t4, t7, t8, t9, t7, t10, t1);
        return;
    case 8:
        Swap4(t3, t4, t5, t9, t10, t1, t8, t7, t6, t8, t3, t2);
        return;
    case 9:
        Swap4(t10, t9, t8, t5, t6, t7, t1, t2, t3, t1, t4, t5);
        return;
    case 10:
        if (BETWEEN(t5, t9, t7))
            Swap4(t5, t6, t7, t9, t10, t1, t4, t3, t2, t4, t9, t8);
        else if (BETWEEN(t3, t9, t6))
            Swap4(t1, t2, t3, t6, t5, t4, t7, t8, t9, t7, t10, t1);
        else
            Swap4(t1, t2, t3, t9, t10, t1, t5, t6, t7, t5, t8, t9);
        return;
    case 11:
        if (BETWEEN(t3, t9, t6))
            Swap4(t1, t2, t3, t6, t5, t4, t9, t10, t1, t7, t8, t9);
        else
            Swap4(t5, t6, t7, t10, t9, t8, t2, t1, t10, t4, t3, t2);
        return;
    case 12:
        Swap4(t1, t2, t3, t8, t7, t6, t10, t9, t8, t5, t10, t1);
        return;
    case 13:
        if (BETWEEN(t4, t9, t7))
            Swap5(t7, t8, t10, t5, t6, t7, t1, t2, t3, t5, t9, t1, t9, t1,
                  t10);
        else if (BETWEEN(t6, t9, t3))
            Swap5(t10, t9, t1, t7, t8, t9, t3, t4, t5, t3, t6, t7, t3, t1,
                  t10);
        else
            Swap5(t10, t9, t1, t4, t3, t2, t5, t6, t7, t5, t8, t10, t9, t1,
                  t10);
        return;
    case 14:
        Swap5(t10, t9, t1, t5, t6, t7, t5, t8, t9, t3, t4, t5, t3, t1,
              t10);
        return;
    case 15:
        if (BETWEEN(t6, t9, t3))
            Swap5(t10, t9, t1, t3, t4, t5, t6, t3, t2, t8, t7, t6, t9, t1,
                  t10);
        else
            Swap5(t1, t2, t6, t3, t4, t5, t8, t7, t6, t10, t9, t8, t2, t10,
                  t1);
        return;
    case 16:
        if (BETWEEN(t4, t9, t7))
            Swap4(t3, t4, t5, t8, t7, t6, t9, t10, t1, t8, t3, t2);
        else if (BETWEEN(t5, t9, t3))
            Swap4(t3, t4, t5, t9, t10, t1, t6, t3, t2, t7, t8, t9);
        else
            Swap4(t3, t4, t5, t1, t2, t3, t7, t8, t9, t7, t10, t1);
        return;
    case 17:
        if (BETWEEN(t7, t9, t3))
            Swap4(t3, t4, t5, t7, t8, t9, t2, t1, t10, t3, t6, t7);
        else
            Swap4(t7, t8, t9, t2, t1, t10, t3, t4, t5, t3, t6, t7);
        return;
    case 18:
        Swap4(t3, t4, t5, t7, t8, t9, t3, t6, t7, t1, t2, t3);
        return;
    case 19:
        Swap4(t7, t8, t9, t1, t2, t3, t6, t5, t4, t7, t10, t1);
        return;
    case 20:
        Swap4(t7, t8, t9, t3, t4, t5, t10, t7, t6, t3, t10, t1);
        return;
    case 21:
        Swap4(t5, t6, t7, t5, t8, t9, t1, t2, t3, t4, t1, t10);
        return;
    case 22:
        Swap4(t1, t2, t3, t6, t5, t4, t7, t8, t1, t9, t10, t1);
        return;
    case 23:
        Swap4(t1, t2, t3, t6, t5, t4, t7, t8, t1, t9, t10, t1);
        return;
    case 24:
        Swap4(t1, t2, t3, t8, t7, t6, t5, t8, t1, t9, t10, t1);
        return;
    case 25:
        Swap4(t1, t2, t3, t8, t7, t6, t5, t8, t1, t9, t10, t1);
        return;
    case 26:
        if (!BETWEEN(t2, t7, t3))
            Swap4(t5, t6, t7, t2, t1, t4, t1, t4, t5, t9, t10, t1);
        else if (BETWEEN(t2, t7, t6))
            Swap4(t5, t6, t7, t5, t8, t3, t3, t8, t1, t9, t10, t1);
        else
            Swap4(t1, t2, t7, t7, t2, t3, t4, t7, t6, t9, t10, t1);
        return;
    case 27:
        Swap4(t3, t4, t5, t6, t3, t2, t1, t6, t7, t9, t10, t1);
        return;
    case 28:
        Swap4(t6, t5, t8, t2, t1, t4, t8, t5, t4, t9, t10, t1);
        return;
    case 29:
        Swap4(t1, t2, t7, t3, t4, t5, t3, t6, t7, t9, t10, t1);
        return;
    case 30:
        if (BETWEEN(t3, t7, t5))
            Swap4(t3, t4, t5, t7, t8, t1, t7, t2, t3, t9, t10, t1);
        else
            Swap4(t3, t4, t5, t3, t6, t7, t1, t2, t3, t9, t10, t1);
        return;
    case 31:
        Swap4(t3, t4, t5, t3, t6, t7, t8, t3, t2, t9, t10, t1);
        return;
    case 32:
        Swap4(t1, t2, t3, t7, t8, t9, t6, t5, t4, t7, t10, t1);
        return;
    case 33:
        if (BETWEEN(t3, t9, t5))
            Swap4(t1, t2, t3, t5, t6, t7, t10, t9, t8, t5, t10, t1);
        else
            Swap4(t1, t2, t3, t7, t8, t9, t7, t10, t1, t5, t6, t7);
        return;
    case 34:
        Swap4(t7, t8, t9, t1, t2, t3, t1, t4, t5, t7, t10, t1);
        return;
    case 35:
        Swap4(t9, t10, t1, t5, t6, t7, t4, t3, t2, t9, t4, t5);
        return;
    case 36:
        Swap4(t9, t10, t1, t7, t8, t9, t3, t4, t5, t6, t3, t2);
        return;
    case 37:
        if (BETWEEN(t6, t9, t4))
            Swap4(t1, t2, t3, t6, t5, t4, t9, t10, t1, t8, t7, t6);
        else
            Swap4(t9, t10, t1, t3, t4, t5, t3, t6, t7, t3, t8, t9);
        return;
    case 38:
        if (BETWEEN(t3, t9, t7))
            Swap4(t1, t2, t3, t7, t8, t9, t6, t5, t4, t6, t1, t10);
        else if (BETWEEN(t6, t9, t4))
            Swap4(t1, t2, t3, t6, t5, t4, t7, t8, t9, t7, t10, t1);
        else
            Swap4(t3, t4, t5, t9, t10, t1, t8, t7, t6, t3, t8, t9);
        return;
    case 39:
        Swap4(t1, t2, t3, t7, t8, t9, t5, t6, t7, t1, t4, t5);
        return;
    case 40:
        Swap4(t9, t10, t1, t4, t3, t2, t5, t6, t7, t5, t8, t9);
        return;
    case 41:
        if (BETWEEN(t5, t9, t7))
            Swap4(t7, t8, t9, t1, t2, t3, t6, t5, t4, t7, t10, t1);
        else if (BETWEEN(t3, t9, t6))
            Swap4(t1, t2, t3, t5, t6, t7, t9, t10, t1, t5, t8, t9);
        else
            Swap4(t5, t6, t7, t9, t10, t1, t2, t9, t8, t3, t4, t5);
        return;
    case 42:
        if (BETWEEN(t3, t9, t6))
            Swap4(t7, t8, t9, t5, t6, t7, t1, t2, t3, t1, t4, t5);
        else
            Swap4(t9, t10, t1, t5, t6, t7, t3, t4, t5, t3, t8, t9);
        return;
    case 43:
        Swap4(t1, t2, t3, t7, t8, t9, t6, t5, t4, t7, t10, t1);
        return;
    case 44:
        if (BETWEEN(t4, t9, t7))
            Swap4(t7, t8, t9, t5, t6, t7, t1, t2, t3, t5, t10, t1);
        else if (BETWEEN(t6, t9, t3))
            Swap4(t9, t10, t1, t5, t6, t7, t3, t4, t5, t3, t8, t9);
        else
            Swap4(t7, t8, t9, t1, t2, t3, t6, t5, t4, t7, t10, t1);
        return;
    case 45:
        Swap4(t9, t10, t1, t3, t4, t5, t7, t8, t9, t3, t6, t7);
        return;
    case 46:
        Swap4(t7, t8, t9, t5, t6, t7, t3, t4, t5, t1, t2, t3);
        return;
    case 47:
        if (BETWEEN(t4, t9, t7))
            Swap4(t5, t6, t7, t1, t2, t3, t9, t10, t1, t5, t8, t9);
        else if (BETWEEN(t5, t9, t3))
            Swap4(t9, t10, t1, t7, t8, t9, t5, t6, t7, t3, t4, t5);
        else
            Swap4(t7, t8, t9, t3, t4, t5, t3, t6, t7, t2, t1, t10);
        return;
    case 48:
        if (BETWEEN(t7, t9, t3))
            Swap4(t3, t4, t5, t8, t7, t6, t2, t1, t10, t8, t3, t2);
        else
            Swap4(t3, t4, t5, t7, t8, t9, t3, t6, t7, t1, t2, t3);
        return;
    case 49:
        Swap4(t9, t10, t1, t5, t6, t7, t3, t4, t5, t3, t8, t9);
        return;
    case 50:
        Swap4(t3, t4, t5, t3, t6, t7, t9, t10, t1, t8, t3, t2);
        return;
    case 51:
        Swap4(t5, t6, t7, t1, t2, t3, t9, t10, t1, t4, t9, t8);
        return;
    case 52:
        Swap4(t5, t6, t7, t3, t4, t5, t9, t10, t1, t3, t8, t9);
        return;
    default:
        eprintf("Make5OptMove: Internal error");
    }
}
