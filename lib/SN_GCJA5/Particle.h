#include "Wire.h"
#include "SN_GCJA5.h"

#define getPM1_0COMMAND 1
#define getPM2_5COMMAND 2
#define getPM10COMMAND  3

#define getPC1_0COMMAND 4
#define getPC2_5COMMAND 5
#define getPC5_0COMMAND 6
#define getPC7_5COMMAND 7
#define getPC10COMMAND 8

float Particle_return(int command);