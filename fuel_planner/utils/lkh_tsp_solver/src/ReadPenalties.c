#include "LKH.h"

/*
 * The ReadPenalties function attempts to read node penalties (Pi-values)
 * from file. 
 *
 * The first line of the file contains the number of nodes.
 *
 * Each of the following lines is of the form
 *       <integer> <integer>
 * where the first integer is a node number, and the second integer 
 * is the Pi-value associated with the node.
 *
 * If reading succeeds, the function returns 1; otherwise 0.
 *
 * The function is called from the CreateCandidateSet function. 
 */

int ReadPenalties()
{
    int i, Id;
    Node *Na, *Nb = 0;
    static int PenaltiesRead = 0;

    if (PiFileName == 0)
        return 0;
    if (PenaltiesRead || !strcmp(PiFileName, "0"))
        return PenaltiesRead = 1;
    if (!(PiFile = fopen(PiFileName, "r")))
        return 0;
    if (TraceLevel >= 1)
        printff("Reading PI_FILE: \"%s\" ... ", PiFileName);
    fscanint(PiFile, &i);
    if (i != Dimension)
        eprintf("PI_FILE \"%s\" does not match problem", PiFileName);
    fscanint(PiFile, &Id);
    assert(Id >= 1 && Id <= Dimension);
    FirstNode = Na = &NodeSet[Id];
    fscanint(PiFile, &Na->Pi);
    for (i = 2; i <= Dimension; i++) {
        fscanint(PiFile, &Id);
        assert(Id >= 1 && Id <= Dimension);
        Nb = &NodeSet[Id];
        fscanint(PiFile, &Nb->Pi);
        Nb->Pred = Na;
        Na->Suc = Nb;
        Na = Nb;
    }
    FirstNode->Pred = Nb;
    Nb->Suc = FirstNode;
    fclose(PiFile);
    if (TraceLevel >= 1)
        ;
    return PenaltiesRead = 1;
}
