#ifndef _GENETIC_H
#define _GENETIC_H

/*
 * This header specifies the interface for the genetic algorithm part of LKH.
 */

typedef void (*CrossoverFunction)();

int MaxPopulationSize; /* The maximum size of the population */
int PopulationSize;    /* The current size of the population */

CrossoverFunction Crossover;

int** Population;  /* Array of individuals (solution tours) */
GainType* Fitness; /* The fitness (tour cost) of each individual */

void AddToPopulation(GainType Cost);
void ApplyCrossover(int i, int j);
void FreePopulation();
int HasFitness(GainType Cost);
int LinearSelection(int Size, double Bias);
GainType MergeTourWithIndividual(int i);
void PrintPopulation();
void ReplaceIndividualWithTour(int i, GainType Cost);
int ReplacementIndividual(GainType Cost);

void ERXT();

#endif
