#ifndef _HASHING_H
#define _HASHING_H

/*
 * This header specifies the interface for hashing.
 */

#include "GainType.h"

#define HashTableSize 65521 /* Largest prime less than USHRT_MAX */
#define MaxLoadFactor 0.75

typedef struct HashTableEntry {
  unsigned Hash;
  GainType Cost;
} HashTableEntry;

typedef struct HashTable {
  HashTableEntry Entry[HashTableSize];
  int Count; /* Number of occupied entries */
} HashTable;

void HashInitialize(HashTable* T);

void HashInsert(HashTable* T, unsigned Hash, GainType Cost);

int HashSearch(HashTable* T, unsigned Hash, GainType Cost);

#endif
