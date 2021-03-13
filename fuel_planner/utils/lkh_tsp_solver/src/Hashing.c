#include "Hashing.h"

/*
 * The functions HashInitialize, HashInsert and HashSearch is used
 * to maintain a hash table of tours. 
 *
 * A hash function maps tours to locations in a hash table. Each time 
 * a tour improvement has been found, the hash table is consulted to 
 * see whether the new tour happens to be local optimum found earlier. 
 * If this is the case, fruitless checkout time is avoided. 
 */

/*
 * HashInitialize(T) empties the hash table T.  
 * Empty entries have Cost equal to MINUS_INFINITY. 
 */

void HashInitialize(HashTable * T)
{
    int i;

    for (i = 0; i < HashTableSize; i++) {
        T->Entry[i].Hash = UINT_MAX;
        T->Entry[i].Cost = MINUS_INFINITY;
    }
    T->Count = 0;
}

/*
 * HashInsert(T,H,Cost) inserts H and Cost (the cost of the tour) in 
 * the table T in a location given by the hash value H. 
 *
 * Collisions are handled by double hashing.
 *
 * The table size is fixed. If the load factor becomes greater than 
 * a specified maximum, MaxLoadFactor, no more insertions will be
 * made. However, if the table entry given by H has a cost greater 
 * than or equal Cost, then Cost of this entry replaces its pervious 
 * value.      
 */

void HashInsert(HashTable * T, unsigned Hash, GainType Cost)
{
    int i = Hash % HashTableSize;
    if (T->Count >= MaxLoadFactor * HashTableSize) {
        if (Cost > T->Entry[i].Cost)
            return;
    } else {
        int p = Hash % 97 + 1;
        while (T->Entry[i].Cost != MINUS_INFINITY)
            if ((i -= p) < 0)
                i += HashTableSize;
        T->Count++;
    }
    T->Entry[i].Hash = Hash;
    T->Entry[i].Cost = Cost;
}

/*
 * HashSearch(T,H,Cost) returns 1 if table T has an entry containing 
 * Cost and H. Otherwise, the function returns 0.
 */

int HashSearch(HashTable * T, unsigned Hash, GainType Cost)
{
    int i, p;

    i = Hash % HashTableSize;
    p = Hash % 97 + 1;
    while ((T->Entry[i].Hash != Hash || T->Entry[i].Cost != Cost)
           && T->Entry[i].Cost != MINUS_INFINITY)
        if ((i -= p) < 0)
            i += HashTableSize;
    return T->Entry[i].Hash == Hash;
}
