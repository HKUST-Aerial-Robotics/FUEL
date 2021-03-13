#ifndef _GPX_H
#define _GPX_H

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#define new_int(n) ((int*)calloc(n, sizeof(int)))
#define new_tour(n) ((tour*)calloc(n, sizeof(tour)))
#define new_gate_structure(n) ((gate_structure*)calloc(n, sizeof(gate_structure)))

typedef struct Adj {
  int vertex;
  struct Adj* nextAdj;
} Adj;

typedef struct Graph {
  int numVertices;
  Adj **firstAdj, **lastAdj;
} Graph;

Graph* new_Graph(int n);
void insertEdge(Graph* g, int v1, int v2);
void freeGraph(Graph* g);
void compCon(Graph* g, int* vector_comp);

typedef struct {
  int num;
  int time;
} gate_structure;

typedef struct {
  gate_structure* inputs;
  gate_structure* outputs;
  gate_structure first_entry;
  gate_structure last_exit;
  GainType fitness;
} tour;

GainType gpx(int* solution_blue, int* solution_red, int* offspring);
void new_candidates(int* vector_comp, int n_new);
void free_candidates(void);
void findInputs(int* sol_blue, int* sol_red);
void testComp(int cand);
int testUnfeasibleComp(int* sol_blue);
void fusion(int* sol_blue, int* sol_red);
void fusionB(int* sol_blue, int* sol_red);
void fusionB_v2(int* sol_blue, int* sol_red);
GainType off_gen(int* sol_blue, int* sol_red, int* offspring, int* label_list);

int n_cities, n_cand;
int n_partitions_size2, n_partitions_before_fusion, n_partitions_after_fusion1,
    n_partitions_after_fusion2, n_partitions_after_fusion3;
int n_partitions_after_fusion4, n_partitions_after_fusion5, n_partitions_after_fusionB;
Node** Map2Node;

int* alloc_vectori(int lines);
int** alloc_matrixi(int lines, int collums);
void dealloc_matrixi(int** Matrix, int lines);
int weight(int i, int j);
int d4_vertices_id(int* solution_blue, int* solution_red, int* d4_vertices, int* common_edges_blue,
                   int* common_edges_red);
void insert_ghost(int* solution, int* solution_p2, int* d4_vertices, int* label_list_inv);
void tourTable(int* solution_blue_p2, int* solution_red_p2, int* solution_red, int* label_list,
               int* label_list_inv, int* vector_comp, int n_new, int* common_edges_blue_p2,
               int* common_edges_red_p2);

#endif
