#ifndef VECTOR_H
#define VECTOR_H

#include <Fusion/FusionMath.h>

typedef union vec3 {
    struct {
        float x;
        float y;
        float z;
    };
    FusionVector fusionVec;
} vec3;

#endif
