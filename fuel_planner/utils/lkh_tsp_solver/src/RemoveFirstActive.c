#include "LKH.h"

/* 
 * The RemoveFirstActive function removes the first node in the list 
 * of "active" nodes (i.e., nodes to be tried as an anchor node, t1,
 * by the LinKernighan algorithm).
 *
 * The function returns a pointer to the removed node. 
 *
 * The list must not be empty before the call. 
 */

Node *RemoveFirstActive()
{
    Node *N = FirstActive;
    if (FirstActive == LastActive)
        FirstActive = LastActive = 0;
    else
        LastActive->Next = FirstActive = FirstActive->Next;
    if (N)
        N->Next = 0;
    return N;
}
