// Force the generic clang vector backend with 32-byte vectors.
#define EIGEN_VECTORIZE_GENERIC 1
#define EIGEN_GENERIC_VECTOR_SIZE_BYTES 32
#include "packetmath.cpp"
