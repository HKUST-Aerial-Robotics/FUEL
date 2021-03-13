#include "LKH.h"

/*
 * The Distance_SPECIAL function may be used to specify a user defined
 * distance fuction. The function is used when the EDGE_WEIGHT_TYPE is
 * SPECIAL. 
 * 
 * Example:
 *  
 *      int Distance_SPECIAL(Node * Na, Node * Nb) 
 *      {
 *           double dx = Na->X - Nb->X;
 *           double dy = Na->Y - Nb->Y;
 *           return (int) (1000 * sqrt(dx * dx + dy * dy) + 0.5);
 *      }           
 */

int Distance_SPECIAL(Node * Na, Node * Nb)
{
    const double GridSize = 1000000.0;
    double dx = Na->X - Nb->X;
    double dy = Na->Y - Nb->Y;
    if (dx < 0)
        dx = -dx;
    if (dy < 0)
        dy = -dy;
    if (GridSize - dx < dx)
        dx = GridSize - dx;
    if (GridSize - dy < dy)
        dy = GridSize - dy;
    return (int) (sqrt(dx * dx + dy * dy) + 0.5);
}
