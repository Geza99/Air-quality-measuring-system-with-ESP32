#include "Particle.h"
#include "SN_GCJA5.h"
#include "Wire.h"

SFE_PARTICLE_SENSOR szp;

float Particle_return(int command){

Wire.begin();
szp.begin(Wire);
switch(command)
{
    case 1: return szp.getPM1_0();break;
    case 2: return szp.getPM2_5();break;
    case 3: return szp.getPM10();break;
    case 4: return szp.getPC1_0();break;
    case 5: return szp.getPC2_5();break;
    case 6: return szp.getPC5_0();break;
    case 7: return szp.getPC7_5();break;
    case 8: return szp.getPC10();break;
}
}
