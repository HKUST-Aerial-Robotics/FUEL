/******************************************************************************\
*                Generalized Partition Crossover 2                            *
*                                                                             *	
* Reference:  R.Tinos, D. Whitley, and G. Ochoa (2017).                       *
* A new generalized partition crossover for the traveling salesman problem:   *
* tunneling between local optima.                                             *
*                                                                             *
* Programmed by R. Tinos. Adapted for LKH by K. Helsgaun.                     *
*                                                                             *
\*****************************************************************************/
#include "LKH.h"
#include "gpx.h"

// GPX2
GainType gpx(int *solution_blue, int *solution_red, int *offspring)
{
    int i, j, *d4_vertices, n_d4_vertices, *common_edges_blue,
        *common_edges_red;
    int *common_edges_p2_blue, *common_edges_p2_red, *label_list,
        *label_list_inv, n_new;
    int *solution_blue_p2, *solution_red_p2, *vector_comp;
    GainType fitness_offspring;

    // Step 1: Identifying the vertices with degree 4 and the common edges
    d4_vertices = alloc_vectori(n_cities);
    common_edges_blue = alloc_vectori(n_cities);
    common_edges_red = alloc_vectori(n_cities);
    n_d4_vertices =
        d4_vertices_id(solution_blue, solution_red, d4_vertices,
                       common_edges_blue, common_edges_red);

    // Step 2: Insert ghost nodes
    n_new = n_cities + n_d4_vertices;
    // size of the new solutions: n_cities + number of ghost nodes
    label_list = alloc_vectori(n_new);
    // label_list: label for each node (including the ghost nodes)
    label_list_inv = alloc_vectori(n_cities);
    // label_list_inv: inverse of label_list
    j = 0;      // counter for the vertices with degree 4 (ghost nodes)
    for (i = 0; i < n_cities; i++) {
        label_list_inv[i] = -1;
        if (d4_vertices[i] == 1) {
            label_list[n_cities + j] = i;
            label_list_inv[i] = n_cities + j;
            j++;
        }
        label_list[i] = i;
    }
    // inserting the ghost nodes in solutions blue and red
    solution_blue_p2 = alloc_vectori(n_new);
    // solution blue with the ghost nodes
    solution_red_p2 = alloc_vectori(n_new);
    // solution red with the ghost nodes
    insert_ghost(solution_blue, solution_blue_p2, d4_vertices,
                 label_list_inv);
    insert_ghost(solution_red, solution_red_p2, d4_vertices,
                 label_list_inv);
    // identifying the common edges for the new solution
    common_edges_p2_blue = alloc_vectori(n_new);
    common_edges_p2_red = alloc_vectori(n_new);
    j = 0;
    for (i = 0; i < n_cities; i++) {
        common_edges_p2_blue[i] = common_edges_blue[i];
        common_edges_p2_red[i] = common_edges_red[i];
        if (d4_vertices[i] == 1) {
            common_edges_p2_blue[i] = 1;
            common_edges_p2_red[i] = 1;
            common_edges_p2_blue[n_cities + j] = common_edges_blue[i];
            common_edges_p2_red[n_cities + j] = common_edges_red[i];
            j++;
        }
    }

    // Step 3: creating the tour tables and finding the connected components
    vector_comp = alloc_vectori(n_new);
    // candidate component for each node (size n_new)
    // identify connected comp. using tour table
    tourTable(solution_blue_p2, solution_red_p2, solution_red, label_list,
              label_list_inv, vector_comp, n_new, common_edges_p2_blue,
              common_edges_p2_red);

    // Step 4: Creating the candidate components
    new_candidates(vector_comp, n_new); // object candidate recombination
    // component
    free(vector_comp);

    // Step 5: Finding the inputs and outputs of each candidate component
    findInputs(solution_blue_p2, solution_red_p2);

    // Step 6: testing the candidate components
    // Step 6.a: test components using simplified internal graphs
    for (i = 0; i < n_cand; i++)
        testComp(i);    // test component i

    // Step 6.b: test unfeasible components using simplified external graphs
    testUnfeasibleComp(solution_blue_p2);

    // Step 7.a: fusions of the candidate components that are neighbours
    //           (with more than to cutting points)
    fusion(solution_blue_p2, solution_red_p2);
    // if candidate i did not pass the test and has conditions, apply
    // fusion with the neighbour with more connections
    fusion(solution_blue_p2, solution_red_p2);
    // if candidate i did not pass the test and has conditions, apply
    // fusion with the neighbour with more connections
    fusion(solution_blue_p2, solution_red_p2);
    // if candidate i did not pass the test and has conditions, apply
    // fusion with the neighbour with more connections

    // Step 7.b: fusions of the candidate components in order to create
    //           partitions with two cutting points
    fusionB(solution_blue_p2, solution_red_p2);
    // if candidate i did not pass the test and has conditions, apply fusionB
    // to find fusions of partitions in order to have partitions with 2 cutting
    // points

    // Selecting the best between the blue and red path in each component
    fitness_offspring = off_gen(solution_blue_p2, solution_red_p2,
                                offspring, label_list);
    free_candidates();
    free(label_list);
    free(label_list_inv);
    free(d4_vertices);
    free(common_edges_blue);
    free(common_edges_p2_blue);
    free(common_edges_red);
    free(common_edges_p2_red);
    free(solution_blue_p2);
    free(solution_red_p2);
    return fitness_offspring;
}

// Compute the weights
int weight(int i, int j)
{
    Node *a = Map2Node[i], *b = Map2Node[j];
    return (C(a, b) - a->Pi - b->Pi) / Precision;
}

// Identifying the vertices with degree 4 and common edges
int d4_vertices_id(int *solution_blue, int *solution_red, int *d4_vertices,
                   int *common_edges_blue, int *common_edges_red)
{
    int i, aux, aux2, **M_aux, n_d4_vertices;

    //create a matrix (n_cities x 4) with all edges;
    M_aux = alloc_matrixi(n_cities, 4);

    for (i = 1; i < n_cities - 1; i++) {
        aux = solution_blue[i];
        M_aux[aux][0] = solution_blue[i + 1];
        M_aux[aux][1] = solution_blue[i - 1];
        aux = solution_red[i];
        M_aux[aux][2] = solution_red[i + 1];
        M_aux[aux][3] = solution_red[i - 1];
    }
    aux = solution_blue[0];
    M_aux[aux][0] = solution_blue[1];
    M_aux[aux][1] = solution_blue[n_cities - 1];
    aux = solution_red[0];
    M_aux[aux][2] = solution_red[1];
    M_aux[aux][3] = solution_red[n_cities - 1];
    aux = solution_blue[n_cities - 1];
    M_aux[aux][0] = solution_blue[0];
    M_aux[aux][1] = solution_blue[n_cities - 2];
    aux = solution_red[n_cities - 1];
    M_aux[aux][2] = solution_red[0];
    M_aux[aux][3] = solution_red[n_cities - 2];
    n_d4_vertices = 0; // number of degree 4 vertices

    for (i = 0; i < n_cities; i++) {
        d4_vertices[i] = 1;
        // d4_vertices: binary vector
        // (1: element is a degree 4 vertex; 0: otherwise);
        common_edges_blue[i] = 0;
        common_edges_red[i] = 0;
        aux = M_aux[i][0];
        aux2 = M_aux[i][2];
        if (aux == aux2 || aux == M_aux[i][3]) {
            d4_vertices[i] = 0;
            common_edges_blue[i] = 1;
            if (aux == aux2)
                common_edges_red[i] = 1;
        }
        aux = M_aux[i][1];
        if (aux == aux2 || aux == M_aux[i][3]) {
            d4_vertices[i] = 0;
            if (aux == aux2)
                common_edges_red[i] = 1;
        }
        if (d4_vertices[i] == 1)
            n_d4_vertices++;
    }

    dealloc_matrixi(M_aux, n_cities);
    return n_d4_vertices;
}

// Insert ghost nodes in the solution
void insert_ghost(int *solution, int *solution_p2, int *d4_vertices,
                  int *label_list_inv)
{
    int i, j, aux;

    j = 0;
    for (i = 0; i < n_cities; i++) {
        aux = solution[i];
        solution_p2[j] = aux;
        j++;
        if (d4_vertices[aux] == 1)
            solution_p2[j++] = label_list_inv[aux];
    }
}

// Finding the ghost pair (returns -1 if node has not a ghost pair)
int ghostPair(int *label_list, int *label_list_inv, int entry)
{
    return entry >
        n_cities - 1 ? label_list[entry] : label_list_inv[entry];
}

// Table code for the reverse solution
int tableCode(int ghost_a, int ghost_b, int ghost_c, int a, int b, int c,
              int common_a, int common_b, int ghost_flag)
{
    int ga, gb, gc;

    // vertices with degree 2
    if (common_a == 1 && common_b == 1)
        return -1;
    // vertices with degree 3 or 4
    if (ghost_a == -1)
        ga = 0;
    else
        ga = 1;
    if (ghost_b == -1)
        gb = 0;
    else
        gb = 1;
    if (ghost_c == -1)
        gc = 0;
    else
        gc = 1;
    if (ga == 0 && gb == 0 && gc == 0)
        return common_b == 1 ? a : c;
    if (ga == 0 && gb == 0 && gc == 1)
        return ghost_c;
    if (ga == 1 && gb == 0 && gc == 0)
        return a;
    if (ghost_flag == 0)
        return gc == 0 ? c : ghost_c;
    return a;
}

// correcting the number of entries 
// (removing common paths and assigned components)
// test if simplified graphs outside unfesible candidate component are equal
// Observation: this is equivalent of testing if all entries for a component
// are grouped after removing the feasible components (identified according
// to testComp) of the list of candidate entries
void simplifyPaths(int *solution_blue_p2, int n_new, int *vector_comp,
                   int *vector_cand, int *n_entries, int n_cand)
{
    int i, j, k, aux, *comp_seq, *inp_comp_seq;

    comp_seq = alloc_vectori(n_new);
    // sequence of components for all entries/exits in unfeasible components
    // in the order given by sol_blue
    inp_comp_seq = alloc_vectori(n_cand);
    // records the number of entries/exits in each component in comp_seq
    // creating comp_seq
    j = 0;      // j is the effective size of comp_seq
    k = solution_blue_p2[0];
    aux = vector_cand[k];
    if (vector_comp[k] == -1) {
        if (aux != vector_cand[solution_blue_p2[n_new - 1]])
            comp_seq[j++] = aux;
        if (aux != vector_cand[solution_blue_p2[1]])
            comp_seq[j++] = aux;
    }
    for (i = 1; i < n_new - 1; i++) {
        k = solution_blue_p2[i];
        aux = vector_cand[k];
        if (vector_comp[k] == -1) {
            if (aux != vector_cand[solution_blue_p2[i - 1]])
                comp_seq[j++] = aux;
            if (aux != vector_cand[solution_blue_p2[i + 1]])
                comp_seq[j++] = aux;
        }
    }
    k = solution_blue_p2[n_new - 1];
    aux = vector_cand[k];
    if (vector_comp[k] == -1) {
        if (aux != vector_cand[solution_blue_p2[n_new - 2]])
            comp_seq[j++] = aux;
        if (aux != vector_cand[solution_blue_p2[0]])
            comp_seq[j++] = aux;
    }
    for (i = 0; i < n_cand; i++)
        inp_comp_seq[i] = 0;

    // testing by checking the grouping of the components
    // (i.e., testing if the number of entries is 2)
    if (j > 0) {
        aux = comp_seq[0];
        if (aux != comp_seq[j - 1])
            inp_comp_seq[aux] = inp_comp_seq[aux] + 1;
        if (aux != comp_seq[1])
            inp_comp_seq[aux] = inp_comp_seq[aux] + 1;
        for (i = 1; i < j - 1; i++) {
            aux = comp_seq[i];
            if (aux != comp_seq[i - 1])
                inp_comp_seq[aux] = inp_comp_seq[aux] + 1;
            if (aux != comp_seq[i + 1])
                inp_comp_seq[aux] = inp_comp_seq[aux] + 1;
        }
        aux = comp_seq[j - 1];
        if (aux != comp_seq[j - 2])
            inp_comp_seq[aux] = inp_comp_seq[aux] + 1;
        if (aux != comp_seq[0])
            inp_comp_seq[aux] = inp_comp_seq[aux] + 1;
        for (i = 0; i < n_cand; i++)
            if (n_entries[i] > 2 && inp_comp_seq[i] == 2)
                n_entries[i] = 2;
    }
    free(inp_comp_seq);
    free(comp_seq);
}

// Filling the first columns of the tour table
void tourTable_fill(int **Tour_table, int *d2_vertices,
                    int *solution_blue_p2, int *solution_red_p2,
                    int *solution_red, int *label_list,
                    int *label_list_inv, int *common_edges_blue_p2,
                    int *common_edges_red_p2, int n_new)
{
    int i, sol1, sol2, ghost_a, ghost_b, ghost_c, common_a, common_b,
        common_c, a, b, c;

    // Inserting in the table the blue and red tours (col. 0-1)
    for (i = 0; i < n_new - 1; i++) {
        // Inserting in the table the tour for the blue tour
        sol1 = solution_blue_p2[i];
        sol2 = solution_blue_p2[i + 1];
        if (common_edges_blue_p2[sol1] == 0) {
            Tour_table[sol1][0] = sol2;
            Tour_table[sol2][0] = sol1;
        } else {
            Tour_table[sol1][3] = sol2;
            Tour_table[sol2][3] = sol1;
        }
        // Inserting in the table the tour for the direct red tour
        sol1 = solution_red_p2[i];
        sol2 = solution_red_p2[i + 1];
        if (common_edges_red_p2[sol1] == 0) {
            Tour_table[sol1][1] = sol2;
            Tour_table[sol2][1] = sol1;
        }
    }
    sol1 = solution_blue_p2[n_new - 1];
    sol2 = solution_blue_p2[0];
    if (common_edges_blue_p2[sol1] == 0) {
        Tour_table[sol1][0] = sol2;
        Tour_table[sol2][0] = sol1;
    } else {
        Tour_table[sol1][3] = sol2;
        Tour_table[sol2][3] = sol1;
    }
    sol1 = solution_red_p2[n_new - 1];
    sol2 = solution_red_p2[0];
    if (common_edges_red_p2[sol1] == 0) {
        Tour_table[sol1][1] = sol2;
        Tour_table[sol2][1] = sol1;
    }
    // Inserting in the table the reverse red tours (col. 2)        
    a = solution_red[n_cities - 1];
    ghost_a = ghostPair(label_list, label_list_inv, a);
    if (ghost_a == -1)
        common_a = common_edges_red_p2[a];
    else
        common_a = common_edges_red_p2[ghost_a];
    b = solution_red[0];
    ghost_b = ghostPair(label_list, label_list_inv, b);
    if (ghost_b == -1)
        common_b = common_edges_red_p2[b];
    else
        common_b = common_edges_red_p2[ghost_b];
    c = solution_red[1];
    ghost_c = ghostPair(label_list, label_list_inv, c);
    if (ghost_c == -1)
        common_c = common_edges_red_p2[c];
    else
        common_c = common_edges_red_p2[ghost_c];
    Tour_table[b][2] = tableCode(ghost_a, ghost_b, ghost_c, a, b, c,
                                 common_a, common_b, 0);
    if (ghost_b != -1)
        Tour_table[ghost_b][2] = tableCode(ghost_a, ghost_b, ghost_c,
                                           a, b, c, common_a, common_b, 1);
    for (i = 1; i < n_cities - 1; i++) {
        a = b;
        ghost_a = ghost_b;
        common_a = common_b;
        b = c;
        ghost_b = ghost_c;
        common_b = common_c;
        c = solution_red[i + 1];
        ghost_c = ghostPair(label_list, label_list_inv, c);
        if (ghost_c == -1)
            common_c = common_edges_red_p2[c];
        else
            common_c = common_edges_red_p2[ghost_c];
        Tour_table[b][2] = tableCode(ghost_a, ghost_b, ghost_c,
                                     a, b, c, common_a, common_b, 0);
        if (ghost_b != -1)
            Tour_table[ghost_b][2] = tableCode(ghost_a, ghost_b, ghost_c,
                                               a, b, c, common_a, common_b,
                                               1);
    }
    a = b;
    ghost_a = ghost_b;
    common_a = common_b;
    b = c;
    ghost_b = ghost_c;
    common_b = common_c;
    c = solution_red[0];
    ghost_c = ghostPair(label_list, label_list_inv, c);
    if (ghost_c == -1)
        common_c = common_edges_red_p2[c];
    else
        common_c = common_edges_red_p2[ghost_c];
    Tour_table[b][2] = tableCode(ghost_a, ghost_b, ghost_c,
                                 a, b, c, common_a, common_b, 0);
    if (ghost_b != -1)
        Tour_table[ghost_b][2] = tableCode(ghost_a, ghost_b, ghost_c,
                                           a, b, c, common_a, common_b, 1);
}

// Identifying the vertices with degree 2
void d2_vertices_id(int *d2_vertices, int *solution_blue_p2,
                    int *common_edges_blue_p2, int n_new)
{
    int i;

    if (common_edges_blue_p2[solution_blue_p2[0]] == 1 &&
        common_edges_blue_p2[solution_blue_p2[n_new - 1]] == 1)
        d2_vertices[solution_blue_p2[0]] = 1;
    else
        d2_vertices[solution_blue_p2[0]] = 0;
    for (i = 1; i < n_new; i++) {
        if (common_edges_blue_p2[solution_blue_p2[i]] == 1 &&
            common_edges_blue_p2[solution_blue_p2[i - 1]] == 1)
            d2_vertices[solution_blue_p2[i]] = 1;
        else
            d2_vertices[solution_blue_p2[i]] = 0;
    }
}

// fixing the labels (in order to avoid gaps)
void labelsFix(int *vector_comp, int n_comp, int n_new)
{
    int i, j, *gap_labels, *new_label;

    gap_labels = alloc_vectori(n_comp);
    new_label = alloc_vectori(n_comp);
    for (i = 0; i < n_comp; i++) {
        gap_labels[i] = 1;
        new_label[i] = i;
    }
    for (i = 0; i < n_new; i++)
        gap_labels[vector_comp[i]] = 0;
    i = 0;
    j = n_comp - 1;
    while (j > i) {
        while (i < n_comp && gap_labels[i] == 0)
            i++;
        while (j >= 0 && gap_labels[j] == 1)
            j--;
        if (j > i) {
            new_label[j] = i;
            gap_labels[i] = 0;
            gap_labels[j] = 1;
        }
        i++;
        j--;
    }
    for (i = 0; i < n_new; i++)
        vector_comp[i] = new_label[vector_comp[i]];
    free(new_label);
    free(gap_labels);
}

// Finding the connected components using the tours table:
// one partition each time
void tourTable(int *solution_blue_p2, int *solution_red_p2,
               int *solution_red, int *label_list, int *label_list_inv,
               int *vector_comp, int n_new, int *common_edges_blue_p2,
               int *common_edges_red_p2)
{
    int i, k, cand_dir, cand_rev, n_comp, sol1, sol2, sol3, sol4, start,
        edge_tour, ghost_pair, n_rounds = 0, n_rounds_max = 1000;
    int min_size_dir, min_size_rev, red_chosen, cand_mcuts, cand_mcuts_dir,
        cand_mcuts_rev, min_size_dir_index, min_size_rev_index;
    int **Tour_table, *assigned_dir, *assigned_rev, *size_dir, *size_rev,
        *vector_comp_red;
    int *d2_vertices, *visited, *recently_assigned, *entries_flag_rev,
        *n_entries_dir, *n_entries_rev, *vector_cand_dir, *vector_cand_rev;

    // Memory allocation
    d2_vertices = alloc_vectori(n_new);
    visited = alloc_vectori(n_new);           // indicates the visited nodes
    vector_comp_red = alloc_vectori(n_new);   // indicates if ghost pair comes
                                              // from dir (0) or rev (1) red
    recently_assigned = alloc_vectori(n_new); // indicates the recently
                                              // assigned nodes
                                              // (for reversing ghost nodes)
    entries_flag_rev = alloc_vectori(n_new);  // auxiliary vector used for
                                              // checking direction of the
                                              // entries
    vector_cand_dir = alloc_vectori(n_new);   // auxiliary vector for
                                              // Tour_table (:,4)
    vector_cand_rev = alloc_vectori(n_new);   // auxiliary vector for
                                              // Tour_table (:,5)

    Tour_table = alloc_matrixi(n_new, 6);
    // Tours table
    // lines: vertices; 
    // columns: 
    //     0 - next single vertex in blue tour
    //     1 - next single vertex in direct red tour
    //     2 - next single vertex in reverse red tour
    //     3 - next common vertex 
    //     4 - candidate to connected component following the direct red tour
    //     5 - candidate to connected component following the reverse red tour
    // Obs.: all vertices has degree 3 or 2                                                                         
    // identifying the vertices with degree 2
    d2_vertices_id(d2_vertices, solution_blue_p2, common_edges_blue_p2,
                   n_new);
    // filling col. 0-3 of the tours table
    tourTable_fill(Tour_table, d2_vertices, solution_blue_p2,
                   solution_red_p2, solution_red, label_list,
                   label_list_inv, common_edges_blue_p2,
                   common_edges_red_p2, n_new);

    // remember that candidates with only one vertex should exist
    // (between common edges) connected components for vertices with degree 2
    // (each one has a label)
    n_comp = 0;
    for (i = 0; i < n_new; i++) {
        vector_comp_red[i] = -1;
        // -1 means that it was not assigned;
        // if assigned, can be 0 (dir. tour) or 1 (rev. tour)
        if (d2_vertices[i] == 1) {
            vector_comp[i] = n_comp;
            n_comp++;
        } else
            vector_comp[i] = -1; // indicates that vertex i was not
                                 // assigned yet                      
    }

    // finding the candidates to connected components (AB cycles) with any
    // number of cuts
    do {
        n_rounds++;
        // assigning the components 
        cand_mcuts_dir = 0;
        cand_mcuts_rev = 0;
        for (i = 0; i < n_new; i++) {
            if (vector_comp[i] == -1)
                visited[i] = 0;
            else
                visited[i] = 1;
            Tour_table[i][4] = -1;
            vector_cand_dir[i] = -1;
        }
        // folowing direct red tour
        // AB Cycles: direct red tour
        // all assigned become visited                          
        cand_dir = 0;
        for (i = 0; i < n_new; i++) {
            if (visited[i] == 0) {
                start = i;
                edge_tour = 0; // 0 for blue edge and 1 for red edge
                do {
                    Tour_table[i][4] = cand_dir;
                    vector_cand_dir[i] = cand_dir;
                    visited[i] = 1;
                    if (edge_tour == 0) {
                        i = Tour_table[i][0]; // get blue edge
                        edge_tour = 1;
                    } else {
                        i = Tour_table[i][1]; // get direct red edge
                        edge_tour = 0;
                    }
                } while (i != start);
                cand_dir++;
            }
        }
        // finding the number of entries and size       
        n_entries_dir = alloc_vectori(cand_dir);
        assigned_dir = alloc_vectori(cand_dir);
        size_dir = alloc_vectori(cand_dir);
        for (i = 0; i < cand_dir; i++) {
            n_entries_dir[i] = 0;
            size_dir[i] = 0;
        }
        for (i = 0; i < n_new; i++) {
            k = Tour_table[i][4];
            if (k != -1) {
                size_dir[k] = size_dir[k] + 1;
                if (k != Tour_table[Tour_table[i][3]][4]) {
                    n_entries_dir[k] = n_entries_dir[k] + 1;
                }
            }
        }
        // correcting the number of entries (removing common paths and
        // assigned components)
        simplifyPaths(solution_blue_p2, n_new, vector_comp,
                      vector_cand_dir, n_entries_dir, cand_dir);
        // following reverse red tour
        // AB Cycles: reverse red tour          
        // all assigned become visited
        cand_rev = 0;
        for (i = 0; i < n_new; i++) {
            if (vector_comp[i] == -1)
                visited[i] = 0;
            else
                visited[i] = 1;
            Tour_table[i][5] = -1;
            vector_cand_rev[i] = -1;
            entries_flag_rev[i] = 0;
            // 1 indicates that one of the entries for candidate cand_rev was
            // alredy assigned for direct red tour
            // obs.: the effective size is the number of candidates)
        }
        for (i = 0; i < n_new; i++) {
            if (visited[i] == 0) {
                start = i;
                edge_tour = 0; // 0 for blue edge and 1 for red edge
                do {
                    Tour_table[i][5] = cand_rev;
                    vector_cand_rev[i] = cand_rev;
                    ghost_pair = ghostPair(label_list, label_list_inv, i);
                    if (ghost_pair != -1) {
                        //check if i and ghost pair (if exists
                        if (vector_comp_red[i] == 0
                            || vector_comp_red[ghost_pair] == 0)
                            entries_flag_rev[cand_rev] = 1;
                        // 1 indicates that one of the entries for candidate 
                        // cand_rev was alredy assigned for direct red tour
                    }
                    visited[i] = 1;
                    if (edge_tour == 0) {
                        i = Tour_table[i][0]; // get blue edge
                        edge_tour = 1;
                    } else {
                        i = Tour_table[i][2]; // get reverse red edge
                        edge_tour = 0;
                    }
                } while (i != start);
                cand_rev++;
            }
        }
        // finding the number of entries and size
        n_entries_rev = alloc_vectori(cand_rev);
        assigned_rev = alloc_vectori(cand_rev);
        size_rev = alloc_vectori(cand_rev);
        for (i = 0; i < cand_rev; i++) {
            n_entries_rev[i] = 0;
            size_rev[i] = 0;
        }
        for (i = 0; i < n_new; i++) {
            k = Tour_table[i][5];
            if (k != -1) {
                size_rev[k] = size_rev[k] + 1;
                if (k != Tour_table[Tour_table[i][3]][5])
                    n_entries_rev[k] = n_entries_rev[k] + 1;
            }
        }
        // correcting the number of entries (removing common paths and
        // assigned components)
        simplifyPaths(solution_blue_p2, n_new, vector_comp,
                      vector_cand_rev, n_entries_rev, cand_rev);
        // Assigning the true candidates 
        // new labels for direct red tour
        min_size_dir = n_new; // minimum size for the candidates
        min_size_dir_index = -1;
        for (i = 0; i < cand_dir; i++) {
            cand_mcuts_dir++;
            assigned_dir[i] = n_comp; // new label
            n_comp++;
            if (size_dir[i] < min_size_dir ||
                (size_dir[i] == min_size_dir && n_entries_dir[i] == 2)) {
                min_size_dir = size_dir[i];
                min_size_dir_index = i;
            }
        }
        // new labels for reverse red tour
        min_size_rev = n_new; // minimum size for the candidates
        min_size_rev_index = -1;
        for (i = 0; i < cand_rev; i++) {
            if (entries_flag_rev[i] == 0) {
                cand_mcuts_rev++;
                assigned_rev[i] = n_comp; // new label
                n_comp++;
                if (size_rev[i] < min_size_rev ||
                    (size_rev[i] == min_size_rev
                     && n_entries_rev[i] == 2)) {
                    min_size_rev = size_rev[i];
                    min_size_rev_index = i;
                }
            } else
                assigned_rev[i] = -1;
        }
        cand_mcuts = cand_mcuts_dir + cand_mcuts_rev;
        if (cand_mcuts > 0 && n_rounds <= n_rounds_max) {
            // assigning components 
            // choose all components in one tour (only one) that has size
            // equal or smaller than the minimum size of the other component
            // use the number of entries when there is a tie
            if (min_size_rev < min_size_dir)
                red_chosen = 1; // 0 for direct and 1 for reverse
            else if (min_size_rev > min_size_dir)
                red_chosen = 0; // 0 for direct and 1 for reverse
            else {
                // Tie
                if (min_size_dir_index == -1)
                    red_chosen = 1;     // 0 for direct and 1 for reverse
                else {
                    if (min_size_rev_index == -1)
                        red_chosen = 0; // 0 for direct and 1 for reverse
                    else if (n_entries_dir[min_size_dir_index] == 2)
                        red_chosen = 0; // 0 for direct and 1 for reverse                                 
                    else if (n_entries_rev[min_size_rev_index] == 2)
                        red_chosen = 1; // 0 for direct and 1 for reverse                                 
                    else if (n_entries_rev[min_size_rev_index] <
                             n_entries_dir[min_size_dir_index])
                        red_chosen = 1; // 0 for direct and 1 for reverse 
                    else
                        red_chosen = 0; // 0 for direct and 1 for reverse
                }

            }
            for (i = 0; i < cand_rev; i++) {
                if (assigned_rev[i] != -1) {
                    if (red_chosen == 0 || size_rev[i] > min_size_dir) {
                        assigned_rev[i] = -1;
                        cand_mcuts_rev--;
                    }
                }
            }
            if (red_chosen == 1 && cand_mcuts_rev == 0) {
                red_chosen = 0;
                min_size_rev = min_size_dir;
            }
            for (i = 0; i < cand_dir; i++) {
                if (red_chosen == 1 || size_dir[i] > min_size_rev) {
                    assigned_dir[i] = -1;
                    cand_mcuts_dir--;
                }
            }
            cand_mcuts = cand_mcuts_dir + cand_mcuts_rev;
            if (cand_mcuts > 0) {
                for (i = 0; i < n_new; i++) {
                    if (vector_comp[i] == -1) {
                        if (red_chosen == 0
                            && assigned_dir[Tour_table[i][4]] != -1) {
                            vector_comp[i] =
                                assigned_dir[Tour_table[i][4]];
                            // assigning component
                            // recording dir. red tour                      
                            if (vector_comp_red[i] == -1) {
                                ghost_pair =
                                    ghostPair(label_list, label_list_inv,
                                              i);
                                if (ghost_pair != -1) {
                                    vector_comp_red[i] = 0;
                                    // zero means that it comes from the
                                    // direct red tour
                                    vector_comp_red[ghost_pair] = 0;
                                    // zero means that it comes from the
                                    // direct red tour
                                    // reversing the ghost nodes (changingi
                                    // direction in  Table) of the red tours
                                    // exchanging the reverse red edge for i
                                    // and ghost node
                                    sol1 = i;
                                    sol2 = ghost_pair;
                                    sol3 = Tour_table[sol1][2];
                                    sol4 = Tour_table[sol2][2];
                                    Tour_table[sol1][2] = sol4;
                                    Tour_table[sol4][2] = sol1;
                                    Tour_table[sol2][2] = sol3;
                                    Tour_table[sol3][2] = sol2;
                                }
                            }
                        } else if (red_chosen == 1
                                   && assigned_rev[Tour_table[i][5]] !=
                                   -1) {
                            vector_comp[i] =
                                assigned_rev[Tour_table[i][5]];
                            // assigning component                      
                            // recording dir. red tour                      
                            if (vector_comp_red[i] == -1) {
                                ghost_pair =
                                    ghostPair(label_list, label_list_inv,
                                              i);
                                if (ghost_pair != -1) {
                                    vector_comp_red[i] = 1;
                                    // zero means that it comes from the
                                    // direct red tour
                                    vector_comp_red[ghost_pair] = 1;
                                    // zero means that it comes from thei
                                    // direct red tour
                                    // reversing the ghost nodes (changingi
                                    // direction in Table) of the red tours
                                    // exchanging the reverse red edge for i
                                    // and ghost node
                                    sol1 = i;
                                    sol2 = ghost_pair;
                                    sol3 = Tour_table[sol1][1];
                                    sol4 = Tour_table[sol2][1];
                                    Tour_table[sol1][1] = sol4;
                                    Tour_table[sol4][1] = sol1;
                                    Tour_table[sol2][1] = sol3;
                                    Tour_table[sol3][1] = sol2;
                                }
                            }
                        }
                    }
                }
            }
        } else {
            // When the maximum number of rounds is reached, assign from
            // direct red tour
            for (i = 0; i < cand_dir; i++) {
                assigned_dir[i] = n_comp;
                n_comp++;
            }
            // assigning new labels 
            for (i = 0; i < n_new; i++) {
                if (vector_comp[i] == -1)
                    vector_comp[i] = assigned_dir[Tour_table[i][4]];
            }
        }
        free(n_entries_dir);
        free(assigned_dir);
        free(size_dir);
        free(n_entries_rev);
        free(assigned_rev);
        free(size_rev);
    } while (cand_mcuts > 0 && n_rounds <= n_rounds_max);

    labelsFix(vector_comp, n_comp, n_new); //fixing the labels

    // change ghost nodes in red tour
    for (i = 0; i < n_new; i++) {
        ghost_pair =
            ghostPair(label_list, label_list_inv, solution_red_p2[i]);
        if (vector_comp_red[solution_red_p2[i]] == 1 && ghost_pair > -1)
            solution_red_p2[i] = ghost_pair;
    }

    // Deallocating memory
    free(vector_comp_red);
    dealloc_matrixi(Tour_table, n_new);
    free(d2_vertices);
    free(visited);
    free(recently_assigned);
    free(entries_flag_rev);
    free(vector_cand_dir);
    free(vector_cand_rev);
}

/******************************************************************************\
*                                   Tour                                       *
\******************************************************************************/

static int *id;        // vector with the id(candidate component) of each node
static int *size;      // vector
static int *n_inputs;
static int *n_outputs;
static tour *blue, *red;
static unsigned int n; // size of the tours 
static int **M_neigh;  // neighbourhood matrix: first collumn indicates the
                       // number of neighbours, and the collumns 2 and 3
                       // indicate the index of the neighbours
static int **M_neigh2; // neighbourhood matrix: the collumns indicate the
                       // number of i conections to the neighbours indicated
                       // in collumns 2 and 3
int *test; // test of the candidates: 1 - true component; 0 - otherwise
static int isequal(Graph * G1, Graph * G2); // test if two graphs are equal

void new_candidates(int *vector_comp, int n_new)
{
    int i, j;

    n = n_new;
    n_cand = 0; // number of candidate components
    for (i = 0; i < n; i++)
        if (vector_comp[i] > n_cand)
            n_cand = vector_comp[i]; // remember that the first component has
                                     // label 0
    n_cand++;
    size = new_int(n_cand); // size of each component
    id = new_int(n);
    n_inputs = new_int(n_cand);
    n_outputs = new_int(n_cand);
    M_neigh = alloc_matrixi(n_cand, 3);
    M_neigh2 = alloc_matrixi(n_cand, 2);
    for (i = 0; i < n_cand; i++)
        size[i] = 0;
    for (i = 0; i < n; i++) {
        id[i] = vector_comp[i];
        j = id[i];
        size[j] = size[j] + 1;
    }
    test = new_int(n_cand);
    blue = new_tour(n_cand);
    red = new_tour(n_cand);
}

void free_candidates(void)
{
    int i;

    free(size);
    free(id);
    free(n_inputs);
    free(n_outputs);
    dealloc_matrixi(M_neigh, n_cand);
    dealloc_matrixi(M_neigh2, n_cand);
    for (i = 0; i < n_cand; i++) {
        free(blue[i].inputs);
        free(blue[i].outputs);
        free(red[i].inputs);
        free(red[i].outputs);
    }
    free(test);
    free(blue);
    free(red);
}

// find the inputs of the candidate components
void findInputs(int *sol_blue, int *sol_red)
{
    int i, j, k, aux, aux2, i_l, i_h, comp_size, n_reduc, *sol_blue_reduc,
        *sol_red_reduc, *sol_blue_reduc_t, *sol_red_reduc_t;
    gate_structure gate;

    for (i = 0; i < n_cand; i++) {
        comp_size = (int) ceil(size[i] / 2); // the inputs/outputs are created
                                             // with maximum size
        blue[i].inputs = new_gate_structure(comp_size);
        blue[i].outputs = new_gate_structure(comp_size);
        blue[i].first_entry.num = -1;
        red[i].inputs = new_gate_structure(comp_size);
        red[i].outputs = new_gate_structure(comp_size);
        red[i].first_entry.num = -1;
    }

    // Solutions without common edges
    sol_blue_reduc = alloc_vectori(n);
    sol_red_reduc = alloc_vectori(n);
    sol_blue_reduc_t = alloc_vectori(n);
    sol_red_reduc_t = alloc_vectori(n);
    j = k = 0;
    for (i = 0; i < n; i++) {
        aux = sol_blue[i];
        if (size[id[aux]] > 1) {
            sol_blue_reduc[j] = aux;
            sol_blue_reduc_t[j] = i;
            j++;
        }
        aux = sol_red[i];
        if (size[id[aux]] > 1) {
            sol_red_reduc[k] = aux;
            sol_red_reduc_t[k] = i;
            k++;
        }
    }
    n_reduc = j;

    // Blue Tour
    for (i = 0; i < n_cand; i++) {
        n_inputs[i] = 0;
        n_outputs[i] = 0;
        M_neigh[i][0] = 0; // the first column indicates the number
                           // of neighbours       
    }
    for (i = 0; i < n_reduc; i++) {
        gate.num = sol_blue_reduc[i];
        gate.time = sol_blue_reduc_t[i];
        if (i == 0)
            i_l = n_reduc - 1;
        else
            i_l = i - 1;
        if (i == (n_reduc - 1))
            i_h = 0;
        else
            i_h = i + 1;
        aux = id[sol_blue_reduc[i]];
        aux2 = id[sol_blue_reduc[i_l]];
        if (aux != aux2) {
            blue[aux].inputs[n_inputs[aux]] = gate;
            n_inputs[aux] = n_inputs[aux] + 1;
            if (blue[aux].first_entry.num == -1)
                blue[aux].first_entry = gate;
            // records the first entry in the candidate for the blue tour
            // Updating the neighbourhood relations
            if (M_neigh[aux][0] == 0) {
                M_neigh[aux][0] = 1;
                M_neigh[aux][1] = aux2;
                M_neigh2[aux][0] = 1;
            } else if (M_neigh[aux][0] == 1) {
                if (M_neigh[aux][1] == aux2)
                    M_neigh2[aux][0] = M_neigh2[aux][0] + 1;
                else {
                    M_neigh[aux][0] = 2;
                    M_neigh[aux][2] = aux2;
                    M_neigh2[aux][1] = 1;
                }
            } else {
                if (M_neigh[aux][1] == aux2)
                    M_neigh2[aux][0] = M_neigh2[aux][0] + 1;
                else if (M_neigh[aux][2] == aux2)
                    M_neigh2[aux][1] = M_neigh2[aux][1] + 1;
                else
                    M_neigh[aux][0] = M_neigh[aux][0] + 1;
            }
        }
        aux2 = id[sol_blue_reduc[i_h]];
        if (aux != aux2) {
            blue[aux].outputs[n_outputs[aux]] = gate;
            n_outputs[aux] = n_outputs[aux] + 1;
            blue[aux].last_exit = gate;
            // records the last exit in the candidate for the blue tour
            // Updating the neighbourhood relations
            if (M_neigh[aux][0] == 0) {
                M_neigh[aux][0] = 1;
                M_neigh[aux][1] = aux2;
                M_neigh2[aux][0] = 1;
            } else if (M_neigh[aux][0] == 1) {
                if (M_neigh[aux][1] == aux2)
                    M_neigh2[aux][0] = M_neigh2[aux][0] + 1;
                else {
                    M_neigh[aux][0] = 2;
                    M_neigh[aux][2] = aux2;
                    M_neigh2[aux][1] = 1;
                }
            } else {
                if (M_neigh[aux][1] == aux2)
                    M_neigh2[aux][0] = M_neigh2[aux][0] + 1;
                else if (M_neigh[aux][2] == aux2)
                    M_neigh2[aux][1] = M_neigh2[aux][1] + 1;
                else
                    M_neigh[aux][0] = M_neigh[aux][0] + 1;
            }
        }
    }

    // Red Tour
    for (i = 0; i < n_cand; i++) {
        n_inputs[i] = 0;
        n_outputs[i] = 0;
    }
    for (i = 0; i < n_reduc; i++) {
        gate.num = sol_red_reduc[i];
        gate.time = sol_red_reduc_t[i];
        if (i == 0)
            i_l = n_reduc - 1;
        else
            i_l = i - 1;
        if (i == (n_reduc - 1))
            i_h = 0;
        else
            i_h = i + 1;
        aux = id[sol_red_reduc[i]];
        if (aux != id[sol_red_reduc[i_l]]) {
            red[aux].inputs[n_inputs[aux]] = gate;
            n_inputs[aux] = n_inputs[aux] + 1;
            if (red[aux].first_entry.num == -1)
                red[aux].first_entry = gate;
            // records the first entry in the candidate for the red tour 
        }
        if (aux != id[sol_red_reduc[i_h]]) {
            red[aux].outputs[n_outputs[aux]] = gate;
            n_outputs[aux] = n_outputs[aux] + 1;
            red[aux].last_exit = gate;
            // records the last exit in the candidate for the red tour      
        }
    }

    free(sol_blue_reduc);
    free(sol_red_reduc);
    free(sol_blue_reduc_t);
    free(sol_red_reduc_t);
}

// test if graphs are equal (obs.: remember that each vertex has 0 or 1 edge)
int isequal(Graph * G1, Graph * G2)
{
    int i = 0, G1_empty, G2_empty, equal = 1;

    while (equal && i < G1->numVertices) {
        G1_empty = !G1->firstAdj[i]; // check if node i has an empty edge list
        G2_empty = !G2->firstAdj[i]; // check if node i has an empty edge list
        if (G1_empty != G2_empty)
            equal = 0;
        else {
            if (!G1_empty) {
                Adj *a = G1->firstAdj[i];
                // first edge of the adjacency list for node i
                Adj *b = G2->firstAdj[i];
                // first edge of the adjacency list for node i
                if (a->vertex != b->vertex)
                    equal = 0;
            }
        }
        i++;
    }
    return equal;
}

void testComp(int cand)
{
    int i, *inp_out_blue_inv, *inp_out_red;

    if (size[cand] <= 1)
        test[cand] = -1;
    else {
        if (n_inputs[cand] < 1)
            test[cand] = 0;
        else {
            if (n_inputs[cand] == 1)
                test[cand] = 1;
            else {
                // Graphs for blue and red tours                        
                inp_out_blue_inv = new_int(n);
                inp_out_red = new_int(2 * n_inputs[cand]);
                // Two cases: Case 1 - first input before first output;
                //            Case 2 - otherwise
                // Obs.: remember that the inputs and outputs are
                // inserted according to the flow                               
                if (blue[cand].inputs[0].time < blue[cand].outputs[0].time) {
                    for (i = 0; i < n_inputs[cand]; i++) {
                        inp_out_blue_inv[blue[cand].inputs[i].num] = 2 * i;
                        inp_out_blue_inv[blue[cand].outputs[i].num] =
                            2 * i + 1;
                    }
                } else {
                    for (i = 0; i < n_inputs[cand]; i++) {
                        inp_out_blue_inv[blue[cand].outputs[i].num] =
                            2 * i;
                        inp_out_blue_inv[blue[cand].inputs[i].num] =
                            2 * i + 1;
                    }
                }
                if (red[cand].inputs[0].time < red[cand].outputs[0].time) {
                    for (i = 0; i < n_inputs[cand]; i++) {
                        inp_out_red[2 * i] = red[cand].inputs[i].num;
                        inp_out_red[2 * i + 1] = red[cand].outputs[i].num;
                    }
                } else {
                    for (i = 0; i < n_inputs[cand]; i++) {
                        inp_out_red[2 * i] = red[cand].outputs[i].num;
                        inp_out_red[2 * i + 1] = red[cand].inputs[i].num;
                    }
                }

                Graph *Gs_blue = new_Graph(2 * n_inputs[cand]);
                // simplified graph for the blue path inside the component cand
                Graph *Gs_red = new_Graph(2 * n_inputs[cand]);
                // simplified graph for the red path inside the component cand
                // Two cases: Case 1 - first input before first output;
                //            Case 2 - otherwise
                // Obs.: remember that the inputs and outputs are inserted
                // according to the flow
                if (blue[cand].inputs[0].time < blue[cand].outputs[0].time) {
                    for (i = 0; i < (2 * n_inputs[cand] - 1); i = i + 2) {
                        insertEdge(Gs_blue, i, i + 1);
                        insertEdge(Gs_blue, i + 1, i);
                    }
                } else {
                    for (i = 1; i < (2 * n_inputs[cand] - 2); i = i + 2) {
                        insertEdge(Gs_blue, i, i + 1);
                        insertEdge(Gs_blue, i + 1, i);
                    }
                    insertEdge(Gs_blue, 2 * n_inputs[cand] - 1, 0);
                    insertEdge(Gs_blue, 0, 2 * n_inputs[cand] - 1);
                }
                if (red[cand].inputs[0].time < red[cand].outputs[0].time) {
                    for (i = 0; i < (2 * n_inputs[cand] - 1); i = i + 2) {
                        insertEdge(Gs_red,
                                   inp_out_blue_inv[inp_out_red[i]],
                                   inp_out_blue_inv[inp_out_red[i + 1]]);
                        insertEdge(Gs_red,
                                   inp_out_blue_inv[inp_out_red[i + 1]],
                                   inp_out_blue_inv[inp_out_red[i]]);
                    }
                } else {
                    for (i = 1; i < (2 * n_inputs[cand] - 2); i = i + 2) {
                        insertEdge(Gs_red,
                                   inp_out_blue_inv[inp_out_red[i]],
                                   inp_out_blue_inv[inp_out_red[i + 1]]);
                        insertEdge(Gs_red,
                                   inp_out_blue_inv[inp_out_red[i + 1]],
                                   inp_out_blue_inv[inp_out_red[i]]);
                    }
                    insertEdge(Gs_red,
                               inp_out_blue_inv[inp_out_red
                                                [2 * n_inputs[cand] - 1]],
                               inp_out_blue_inv[inp_out_red[0]]);
                    insertEdge(Gs_red,
                               inp_out_blue_inv[inp_out_red[0]],
                               inp_out_blue_inv[inp_out_red
                                                [2 * n_inputs[cand] - 1]]);
                }

                // Comparing the two graphs                                     
                test[cand] = isequal(Gs_blue, Gs_red);
                free(Gs_red);
                free(Gs_blue);
                free(inp_out_blue_inv);
                free(inp_out_red);
            }
        }
    }
}

// test if simplified graphs outside unfesible candidate component are equal
// Observation: this is equivalent of testing if all entries for a component
// are grouped after removing the feasible components (identified according to
// testComp) of the list of candidate entries
int testUnfeasibleComp(int *sol_blue)
{
    int i, j, aux, *comp_seq, *inp_comp_seq, n_newpart = 0;

    comp_seq = alloc_vectori(n); // sequence of all entries in unfeasible
                                 // components in the order given by sol_blue
    inp_comp_seq = alloc_vectori(n_cand);
    // records the number of entries in
    // each component in comp_seq
    // creating comp_seq
    j = 0; // j is the effective size of comp_seq
    aux = id[sol_blue[0]];
    if (test[aux] == 0 && aux != id[sol_blue[n - 1]])
        comp_seq[j++] = aux;
    inp_comp_seq[aux] = 0;
    for (i = 1; i < n; i++) {
        aux = id[sol_blue[i]];
        if (test[aux] == 0 && aux != id[sol_blue[i - 1]])
            comp_seq[j++] = aux;
        inp_comp_seq[aux] = 0;
    }

    // testing by checking the grouping of the components
    // (i.e., testing if the number of entries is 2
    if (j > 0) {
        aux = comp_seq[0];
        if (aux != comp_seq[j - 1])
            inp_comp_seq[aux] = inp_comp_seq[aux] + 1;
        for (i = 1; i < j; i++) {
            aux = comp_seq[i];
            if (aux != comp_seq[i - 1])
                inp_comp_seq[aux] = inp_comp_seq[aux] + 1;
        }
        for (i = 0; i < n_cand; i++)
            if (test[i] == 0 && inp_comp_seq[i] == 1) {
                test[i] = 1;
                n_newpart++;
            }
    }
    free(inp_comp_seq);
    free(comp_seq);
    return n_newpart;
}

// if candidate cand did not pass the test and has conditions,
// apply fusion with the neighbour with more connections
// (for more than 2 cutting points)
void fusion(int *sol_blue, int *sol_red)
{
    int i, cand, aux, n_fusions = 0, *neigh_vec_cond, *neigh_vec_ind;

    neigh_vec_cond = new_int(n_cand);
    // cond= -1 if cand is a true component or if it is between two
    // common edges;   = 0 stil does not chosen;
    //                 = 1 already chosen, and id will be changed;
    //                 = 2 already chosen but id will not be changed 
    neigh_vec_ind = new_int(n_cand); // neigh_vec_ind: indicates the neighbour
                                     // that will be fusioned with cand
    for (cand = 0; cand < n_cand; cand++)
        neigh_vec_cond[cand] = test[cand] == 1 || size[cand] <= 1 ? -1 : 0;

    for (cand = 0; cand < n_cand; cand++) {
        if (neigh_vec_cond[cand] == 0) {
            if (M_neigh[cand][0] == 1) {
                aux = M_neigh[cand][1];
                if (neigh_vec_cond[aux] == 0) {
                    neigh_vec_ind[cand] = aux;
                    neigh_vec_cond[cand] = 1;
                    neigh_vec_cond[aux] = 2;
                    n_fusions++;
                }
            } else if (M_neigh[cand][0] == 2) {
                if (M_neigh2[cand][0] > M_neigh2[cand][1]) {
                    aux = M_neigh[cand][1];
                    if (neigh_vec_cond[aux] == 0) {
                        neigh_vec_ind[cand] = aux;
                        neigh_vec_cond[cand] = 1;
                        neigh_vec_cond[aux] = 2;
                        n_fusions++;
                    }
                } else {
                    aux = M_neigh[cand][2];
                    if (neigh_vec_cond[aux] == 0) {
                        neigh_vec_ind[cand] = aux;
                        neigh_vec_cond[cand] = 1;
                        neigh_vec_cond[aux] = 2;
                        n_fusions++;
                    }
                }
            }

        }
    }

    if (n_fusions > 0) {
        // Reseting tour structures blue and red 
        for (cand = 0; cand < n_cand; cand++) {
            free(blue[cand].inputs);
            free(blue[cand].outputs);
            free(red[cand].inputs);
            free(red[cand].outputs);
        }
        free(blue);
        free(red);
        blue = new_tour(n_cand);
        red = new_tour(n_cand);

        // Making the fusions
        for (i = 0; i < n; i++) {
            aux = id[i];
            if (neigh_vec_cond[aux] == 1) {
                size[aux] = size[aux] - 1;
                id[i] = neigh_vec_ind[aux];
                aux = id[i];
                size[aux] = size[aux] + 1;
            }
        }

        // Repeating Step 5: Finding the inputs and outputs of each
        // candidate component
        findInputs(sol_blue, sol_red);
        // Repeating Step 6: testing the candidate components
        for (cand = 0; cand < n_cand; cand++)
            if (neigh_vec_cond[cand] == 2)
                testComp(cand); // test component cand                  
    }

    free(neigh_vec_cond);
    free(neigh_vec_ind);
}

// fusions of the candidate components in order to create partitions with two
// cutting points
void fusionB(int *sol_blue, int *sol_red)
{
    int i, cand, n_cand_seq = 0, previous_cand, next_cand, n_cand_new,
        n_rounds = 0, n_rounds_max = 1000;
    int *cand_seq, *cand_seq_cut, *assigned_cand, *new_label,
        *vector_new_cand, *new_component, n_newpart;

    // Memory allocation
    cand_seq = alloc_vectori(n); // list of entries and exits of
                                 // unfeasible candidates
    cand_seq_cut = alloc_vectori(n);
    new_label = alloc_vectori(n_cand);
    new_component = alloc_vectori(n_cand);
    Graph *G_cand = new_Graph(n_cand);

    // Walking in the blue tour and finding the high level cuts
    previous_cand = id[sol_blue[n - 1]];
    for (i = 0; i < n; i++) {
        cand = id[sol_blue[i]]; // candidate for vertex i of the blue tour
        if (i == (n - 1))
            next_cand = id[sol_blue[0]];
        else
            next_cand = id[sol_blue[i + 1]];
        // test if it is an unfeasible partition
        if (test[cand] == 0 && n_inputs[cand] > 0 && size[cand] > 1) {
            if (cand != previous_cand || cand != next_cand) {
                cand_seq[n_cand_seq] = cand;
                // checking if it is a first common entry or last common exit
                if (sol_blue[i] == blue[cand].first_entry.num &&
                    (blue[cand].first_entry.num ==
                     red[cand].first_entry.num
                     || blue[cand].first_entry.num ==
                     red[cand].last_exit.num))
                    cand_seq_cut[n_cand_seq] = 1; // it is a first common entry
                else if (sol_blue[i] == blue[cand].last_exit.num &&
                         (blue[cand].last_exit.num ==
                          red[cand].last_exit.num ||
                          blue[cand].last_exit.num ==
                          red[cand].first_entry.num))
                    cand_seq_cut[n_cand_seq] = 1; // it is a last common exit       
                else
                    cand_seq_cut[n_cand_seq] = 0;
                n_cand_seq++;
            }
        }
        previous_cand = cand;
    }
    // building the graph with conections between unfeasible components,
    // but without main entries and main exits
    if (n_cand_seq > 0) {
        for (i = 0; i < n_cand_seq - 1; i++) {
            if (cand_seq[i] != cand_seq[i + 1] &&
                cand_seq_cut[i] == 0 && cand_seq_cut[i + 1] == 0) {
                // insert edge between candidates
                insertEdge(G_cand, cand_seq[i], cand_seq[i + 1]);
                // insert edge between candidates                               
                insertEdge(G_cand, cand_seq[i + 1], cand_seq[i]);
            }
        }
        if (cand_seq[n_cand_seq - 1] != cand_seq[0] &&
            cand_seq_cut[n_cand_seq - 1] == 0 && cand_seq_cut[0] == 0) {
            // insert edge between candidates
            insertEdge(G_cand, cand_seq[n_cand_seq - 1], cand_seq[0]);
            // insert edge between candidates                           
            insertEdge(G_cand, cand_seq[0], cand_seq[n_cand_seq - 1]);
        }
    }
    // Walking in the red tour and finding the high level cuts
    n_cand_seq = 0;
    previous_cand = id[sol_red[n - 1]];
    for (i = 0; i < n; i++) {
        cand = id[sol_red[i]]; // candidate for vertex i of the red tour
        if (i == (n - 1))
            next_cand = id[sol_red[0]];
        else
            next_cand = id[sol_red[i + 1]];
        // test if it is an unfeasible partition
        if (test[cand] == 0 && n_inputs[cand] > 0 && size[cand] > 1) {
            if (cand != previous_cand || cand != next_cand) {
                cand_seq[n_cand_seq] = cand;
                // checking if it is a first common entry or last common exit
                if (sol_red[i] == red[cand].first_entry.num &&
                    (red[cand].first_entry.num ==
                     blue[cand].first_entry.num
                     || red[cand].first_entry.num ==
                     blue[cand].last_exit.num))
                    cand_seq_cut[n_cand_seq] = 1; // it is a first common entry
                else if (sol_red[i] == red[cand].last_exit.num &&
                         (red[cand].last_exit.num ==
                          blue[cand].last_exit.num ||
                          red[cand].last_exit.num ==
                          blue[cand].first_entry.num))
                    cand_seq_cut[n_cand_seq] = 1; // it is a last common exit       
                else
                    cand_seq_cut[n_cand_seq] = 0;
                n_cand_seq++;
            }
        }
        previous_cand = cand;
    }
    // building the graph with connections between unfeasible components,i
    // but without main entries and main exits
    if (n_cand_seq > 0) {
        for (i = 0; i < n_cand_seq - 1; i++) {
            if (cand_seq[i] != cand_seq[i + 1] &&
                cand_seq_cut[i] == 0 && cand_seq_cut[i + 1] == 0) {
                // insert edge between candidates
                insertEdge(G_cand, cand_seq[i], cand_seq[i + 1]);
                // insert edge between candidates                               
                insertEdge(G_cand, cand_seq[i + 1], cand_seq[i]);
            }
        }
        if (cand_seq[n_cand_seq - 1] != cand_seq[0] &&
            cand_seq_cut[n_cand_seq - 1] == 0 && cand_seq_cut[0] == 0) {
            // insert edge between candidates
            insertEdge(G_cand, cand_seq[n_cand_seq - 1], cand_seq[0]);
            // insert edge between candidates
            insertEdge(G_cand, cand_seq[0], cand_seq[n_cand_seq - 1]);
        }
    }

    for (cand = 0; cand < n_cand; cand++)
        new_label[cand] = cand;

    if (n_cand_seq > 0) {
        vector_new_cand = alloc_vectori(n_cand); // fusion of candidates        
        compCon(G_cand, vector_new_cand); // find the connected components of
                                          // the graph
        // new label
        n_cand_new = -1;
        for (i = 0; i < n_cand; i++) {
            new_component[i] = 0;
            if (n_cand_new < vector_new_cand[i])
                n_cand_new = vector_new_cand[i];
        }
        if (n_cand_new > -1) {
            n_cand_new++;
            assigned_cand = alloc_vectori(n_cand_new);
            for (cand = 0; cand < n_cand_new; cand++)
                assigned_cand[cand] = -1;
            for (cand = 0; cand < n_cand; cand++) {
                if (test[cand] == 0 && n_inputs[cand] > 0
                    && size[cand] > 1) {
                    if (assigned_cand[vector_new_cand[cand]] == -1) {
                        assigned_cand[vector_new_cand[cand]] = cand;
                        new_component[cand] = 1;
                    }
                    new_label[cand] = assigned_cand[vector_new_cand[cand]];
                }
            }
            free(assigned_cand);

            // Reseting tour structures blue and red 
            for (cand = 0; cand < n_cand; cand++) {
                free(blue[cand].inputs);
                free(blue[cand].outputs);
                free(red[cand].inputs);
                free(red[cand].outputs);
            }
            free(blue);
            free(red);
            blue = new_tour(n_cand);
            red = new_tour(n_cand);

            // Making the fusions
            for (i = 0; i < n; i++) {
                cand = id[i];
                if (new_label[cand] != cand) {
                    size[cand] = size[cand] - 1;
                    id[i] = new_label[cand];
                    cand = id[i];
                    size[cand] = size[cand] + 1;
                }
            }

            // Repeating Step 5: Finding the inputs and outputs of each
            // candidate component
            findInputs(sol_blue, sol_red);

            // Repeating Step 6: testing the candidate components
            // this procedure is O(n)
            for (cand = 0; cand < n_cand; cand++)
                if (new_component[cand] == 1)
                    testComp(cand);

            // Testing unfeasible partitions 
            do {
                n_rounds++;
                n_newpart = testUnfeasibleComp(sol_blue);
            } while (n_newpart > 0 && n_rounds <= n_rounds_max);
        }

        free(vector_new_cand);
    }
    free(G_cand);
    free(cand_seq);
    free(cand_seq_cut);
    free(new_label);
    free(new_component);
}

// select between the blue and red paths for each component
GainType off_gen(int *sol_blue, int *sol_red, int *offspring,
                 int *label_list)
{
    int i, i_l, i_h, k, aux, aux2, *select_cand, select_rest,
        *offspring_p2, *sol_blue_index, *sol_red_index, v_aux;
    GainType blue_fitness_rest, red_fitness_rest, fitness;

    // Computing the Fitness of each subtour in each true component
    for (i = 0; i < n_cand; i++) {
        blue[i].fitness = 0;
        red[i].fitness = 0;
    }
    blue_fitness_rest = 0; // fitness of the parts that are not true components
                           // in the blue path
    red_fitness_rest = 0;  // fitness of the parts that are not true components
                           // in the red path
    for (i = 0; i < n; i++) {
        if (i < n - 1)
            i_h = i + 1;
        else
            i_h = 0;
        aux = sol_blue[i];
        aux2 = sol_blue[i_h];
        if (test[id[aux]] > 0 && id[aux] == id[aux2]) {
            if (label_list[aux] != label_list[aux2])
                blue[id[aux]].fitness = blue[id[aux]].fitness +
                    weight(label_list[aux], label_list[aux2]);
        } else {
            if (label_list[aux] != label_list[aux2])
                blue_fitness_rest = blue_fitness_rest +
                    weight(label_list[aux], label_list[aux2]);
        }
        aux = sol_red[i];
        aux2 = sol_red[i_h];
        if (test[id[aux]] > 0 && id[aux] == id[aux2]) {
            if (label_list[aux] != label_list[aux2])
                red[id[aux]].fitness = red[id[aux]].fitness +
                    weight(label_list[aux], label_list[aux2]);
        } else if (label_list[aux] != label_list[aux2])
            red_fitness_rest = red_fitness_rest +
                weight(label_list[aux], label_list[aux2]);
    }

    // Selecting the components
    if (blue_fitness_rest > red_fitness_rest)
        select_rest = 1; // 1 for red and 0 for blue
    else
        select_rest = 0;
    select_cand = new_int(n);

    for (i = 0; i < n_cand; i++) {
        if (test[i] < 1)
            select_cand[i] = select_rest;
        else {
            if (blue[i].fitness > red[i].fitness)
                select_cand[i] = 1;
            else
                select_cand[i] = 0;
        }
    }

    // Generating the offspring     
    // index for the solutions
    sol_blue_index = new_int(n);
    sol_red_index = new_int(n);
    for (i = 0; i < n; i++) {
        sol_blue_index[sol_blue[i]] = i;
        sol_red_index[sol_red[i]] = i;
    }
    // Offspring: Graph
    Graph *G_offspring = new_Graph(n); // graph for offspring
    for (i = 0; i < n; i++) {
        if (select_cand[id[i]] == 0) {
            if (sol_blue_index[i] == 0) {
                i_l = sol_blue[n - 1];
                i_h = sol_blue[sol_blue_index[i] + 1];
            } else if (sol_blue_index[i] == (n - 1)) {
                i_l = sol_blue[sol_blue_index[i] - 1];
                i_h = sol_blue[0];
            } else {
                i_l = sol_blue[sol_blue_index[i] - 1];
                i_h = sol_blue[sol_blue_index[i] + 1];
            }
        } else {
            if (sol_red_index[i] == 0) {
                i_l = sol_red[n - 1];
                i_h = sol_red[sol_red_index[i] + 1];
            } else if (sol_red_index[i] == (n - 1)) {
                i_l = sol_red[sol_red_index[i] - 1];
                i_h = sol_red[0];
            } else {
                i_l = sol_red[sol_red_index[i] - 1];
                i_h = sol_red[sol_red_index[i] + 1];
            }
        }
        insertEdge(G_offspring, i, i_l);
        insertEdge(G_offspring, i, i_h);
    }

    // Offspring: vector
    offspring_p2 = new_int(n);
    i = 0;
    offspring_p2[i] = 0;
    v_aux = 0;
    Adj *a = G_offspring->firstAdj[v_aux]; // first edge of the vertex 
    Adj *b = a->nextAdj; // next edge of the vertex 
    v_aux = a->vertex;
    i++;
    offspring_p2[i] = v_aux;

    while (v_aux > 0) {
        a = G_offspring->firstAdj[v_aux]; // first edge of the vertex
        b = a->nextAdj; // next edge of the vertex
        assert(a->vertex != b->vertex);
        if (a->vertex == offspring_p2[i - 1])
            v_aux = b->vertex;
        else
            v_aux = a->vertex;
        i++;
        if (v_aux > 0)
            offspring_p2[i] = v_aux;
    }

    freeGraph(G_offspring);

    // Fitness of the offspring
    fitness = 0;
    for (i = 0; i < n_cand; i++) {
        if (test[i] == 1) {
            if (select_cand[i] == 0)
                fitness = fitness + blue[i].fitness;
            else
                fitness = fitness + red[i].fitness;
        }
    }
    if (select_rest == 0)
        fitness = fitness + blue_fitness_rest;
    else
        fitness = fitness + red_fitness_rest;

    // removing the ghost nodes
    k = 0;
    for (i = 0; i < n; i++)
        if (offspring_p2[i] < n_cities)
            offspring[k++] = offspring_p2[i];

    free(sol_red_index);
    free(sol_blue_index);
    free(select_cand);
    free(offspring_p2);
    return fitness;
}

/******************************************************************************\
*			 		 Dynamic allocation	and deallocatio       *
\******************************************************************************/

int *alloc_vectori(int lines)
{
    int *vector;

    vector = (int *) malloc(lines * sizeof(int));
    if (!vector) {
        printf("Allocation Error\n");
        exit(EXIT_FAILURE);
    }
    return vector;
}

int **alloc_matrixi(int lines, int collumns)
{
    int i, **Matrix;

    Matrix = (int **) malloc(lines * sizeof(int *));
    for (i = 0; i < lines; i++)
        Matrix[i] = (int *) malloc(collumns * sizeof(int));
    if (!Matrix) {
        printf("Allocation Error\n");
        exit(EXIT_FAILURE);
    }
    return Matrix;
}

void dealloc_matrixi(int **Matrix, int lines)
{
    int i;
    for (i = 0; i < lines; i++)
        free(Matrix[i]);
    free(Matrix);
}

/******************************************************************************\
*                                  Graph                                       *
\******************************************************************************/

Graph *new_Graph(int n)
{
    Graph *g = (Graph *) malloc(sizeof(Graph));
    g->numVertices = n;
    g->firstAdj = (Adj **) calloc(n, sizeof(Adj *));
    g->lastAdj = (Adj **) calloc(n, sizeof(Adj *));
    return g;
}

void insertEdge(Graph * g, int v1, int v2)
{
    Adj *a = (Adj *) malloc(sizeof(Adj));
    a->vertex = v2;
    a->nextAdj = 0;
    if (!g->firstAdj[v1])
        g->firstAdj[v1] = g->lastAdj[v1] = a;
    else
        g->lastAdj[v1] = g->lastAdj[v1]->nextAdj = a;
}

void freeGraph(Graph * g)
{
    int v;
    for (v = 0; v < g->numVertices; v++) {
        Adj *a = g->firstAdj[v];
        while (a) {
            Adj *tmp = a;
            a = a->nextAdj;
            free(tmp);
        }
    }
    free(g->firstAdj);
    free(g->lastAdj);
    free(g);
}

const int white = 0, grey = 1, black = 2;

void visitaDfsCC(Graph * g, int u, int *color,
                 int *vector_comp, int components)
{
    vector_comp[u] = components;
    color[u] = grey;
    Adj *a = g->firstAdj[u];
    while (a) {
        int v = a->vertex;
        if (color[v] == white)
            visitaDfsCC(g, v, color, vector_comp, components);
        a = a->nextAdj;
    }
    color[u] = black;
}

void compCon(Graph * g, int *vector_comp)
{
    int components = 0, u, n = g->numVertices;
    int *color = (int *) malloc(n * sizeof(int));

    for (u = 0; u < n; u++)
        color[u] = white;
    for (u = 0; u < n; u++) {
        if (color[u] == white) {
            visitaDfsCC(g, u, color, vector_comp, components);
            components++;
        }
    }
}
