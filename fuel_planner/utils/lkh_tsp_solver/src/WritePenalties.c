#include "LKH.h"

/*
 * The WritePenalties function writes node penalties (Pi-values)
 * to file PiFileName. 
 *
 * The first line of the file contains the number of nodes.
 *
 * Each of the following lines is of the form
 *       <integer> <integer>
 * where the first integer is a node number, and the second integer 
 * is the Pi-value associated with the node.
 *
 * The function is called from the CreateCandidateSet function. 
 */

void WritePenalties()
{
    Node *N;

    if (PiFileName == 0 || !(PiFile = fopen(PiFileName, "w")))
        return;
    if (TraceLevel >= 1)
        printff("Writing PI_FILE: \"%s\" ... ", PiFileName);
    fprintf(PiFile, "%d\n", Dimension);
    N = FirstNode;
    do
        fprintf(PiFile, "%d %d\n", N->Id, N->Pi);
    while ((N = N->Suc) != FirstNode);
    fprintf(PiFile, "-1\nEOF\n");
    fclose(PiFile);
    if (TraceLevel >= 1)
        printff("done\n", PiFileName);
}
