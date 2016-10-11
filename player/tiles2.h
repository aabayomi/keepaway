#ifndef _TILES2_H_
#define _TILES2_H_

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include "SoccerTypes.h"

#define MAX_NUM_VARS 64        // Maximum number of variables in a grid-tiling
#define MaxLONGINT 2147483647

class collision_table {
public:
  collision_table();

  ~collision_table();

  long data[RL_MEMORY_SIZE];
  const int safe;
  long calls;
  long clearhits;
  long collisions;

  void reset();

  int usage();

  void save(int);

  void restore(int);
};

void GetTiles(
    int tiles[],               // provided array contains returned tiles (tile indices)
    int num_tilings,           // number of tile indices to be returned in tiles
    int memory_size,           // total number of possible tiles
    float floats[],            // array of floating point variables
    int num_floats,            // number of floating point variables
    int ints[],          // array of integer variables
    int num_ints);             // number of integer variables

void GetTiles(
    int tiles[],               // provided array contains returned tiles (tile indices)
    int num_tilings,           // number of tile indices to be returned in tiles
    collision_table *ctable,   // total number of possible tiles
    float floats[],            // array of floating point variables
    int num_floats,            // number of floating point variables
    int ints[],          // array of integer variables
    int num_ints);             // number of integer variables

void GetTilesWrap(
    int tiles[],               // provided array contains returned tiles (tile indices)
    int num_tilings,           // number of tile indices to be returned in tiles
    int memory_size,           // total number of possible tiles
    float floats[],            // array of floating point variables
    int num_floats,            // number of floating point variables
    int wrap_widths[],         // array of widths (length and units as in floats)
    int ints[],          // array of integer variables
    int num_ints);             // number of integer variables

void GetTilesWrap(
    int tiles[],               // provided array contains returned tiles (tile indices)
    int num_tilings,           // number of tile indices to be returned in tiles
    collision_table *ctable,   // total number of possible tiles
    float floats[],            // array of floating point variables
    int num_floats,            // number of floating point variables
    int wrap_widths[],         // array of widths (length and units as in floats)
    int ints[],          // array of integer variables
    int num_ints);             // number of integer variables

// no ints
void GetTilesWrap(int tiles[], int num_tilings, int memory_size, float floats[],
                  int num_floats, int wrap_widths[]);

long hash_UNH(int *ints, int num_ints, long m, int increment);

long hash_safe(int *ints, int num_ints, collision_table *ctable);

// no ints
void GetTiles(int tiles[], int nt, int memory, float floats[], int nf);

void GetTiles(int tiles[], int nt, collision_table *ct, float floats[], int nf);


// one int
void GetTiles(int tiles[], int nt, int memory, float floats[], int nf, int h1);

void GetTiles(int tiles[], int nt, collision_table *ct, float floats[], int nf, int h1);

// two ints
void GetTiles(int tiles[], int nt, int memory, float floats[], int nf, int h1, int h2);

void GetTiles(int tiles[], int nt, collision_table *ct, float floats[], int nf, int h1, int h2);

// three ints
void GetTiles(int tiles[], int nt, int memory, float floats[], int nf, int h1, int h2, int h3);

void GetTiles(int tiles[], int nt, collision_table *ct, float floats[], int nf, int h1, int h2, int h3);

// one float, no ints
void GetTiles1(int tiles[], int nt, int memory, float f1);

void GetTiles1(int tiles[], int nt, collision_table *ct, float f1);

// one float, one int
void GetTiles1(int tiles[], int nt, int memory, float f1, int h1);

void GetTiles1(int tiles[], int nt, collision_table *ct, float f1, int h1);

// one float, two ints
void GetTiles1(int tiles[], int nt, int memory, float f1, int h1, int h2);

void GetTiles1(int tiles[], int nt, collision_table *ct, float f1, int h1, int h2);

// one float, three ints
void GetTiles1(int tiles[], int nt, int memory, float f1, int h1, int h2, int h3);

void GetTiles1(int tiles[], int nt, collision_table *ct, float f1, int h1, int h2, int h3);

// one float, four ints
void GetTiles1(int tiles[], int nt, int memory, float f1, int h1, int h2, int h3, int h4);

void GetTiles1(int tiles[], int nt, collision_table *ct, float f1, int h1, int h2, int h3, int h4);

// one float, five ints
void GetTiles1(int tiles[], int nt, int memory, float f1, int h1, int h2, int h3, int h4, int h5);

void GetTiles1(int tiles[], int nt, collision_table *ct, float f1, int h1, int h2, int h3, int h4, int h5);


// two floats, no ints
void GetTiles2(int tiles[], int nt, int memory, float f1, float f2);

void GetTiles2(int tiles[], int nt, collision_table *ct, float f1, float f2);

// two floats, one int
void GetTiles2(int tiles[], int nt, int memory, float f1, float f2, int h1);

void GetTiles2(int tiles[], int nt, collision_table *ct, float f1, float f2, int h1);

// two floats, two ints
void GetTiles2(int tiles[], int nt, int memory, float f1, float f2, int h1, int h2);

void GetTiles2(int tiles[], int nt, collision_table *ct, float f1, float f2, int h1, int h2);

// two floats, three ints
void GetTiles2(int tiles[], int nt, int memory, float f1, float f2, int h1, int h2, int h3);

void GetTiles2(int tiles[], int nt, collision_table *ct, float f1, float f2, int h1, int h2, int h3);

#endif

