#ifndef AUXILIARY_H
#define AUXILIARY_H

#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET

#include <ecl/linear_algebra.hpp>
#include <sstream>
#include <fstream>
#include <string>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <cstdarg>
#include <cstring>
#include <cstddef>
#include <iostream>
#include <vector>
#include <Eigen/Sparse>

/* Some useful namespaces */
using namespace ecl::linear_algebra;
using namespace std;

/* Maximum matrix buffer size, larger matrix = problem */
#define MAXBUFSIZE  ((int) 1e6)

/*
* readMatrix:
* Import a matrix from a txt file.
* It needs to run once and need to be extremely efficient.
* Adapted from the following link:
* http://stackoverflow.com/questions/14199798/very-generic-argmax-function-in-c-wanted
* Needs a full path to the txt file to work, which is kind of annoying
*/
//template <typename MyMatType>
MatrixXf readMatrix(const char *filename);
SparseMatrix<float> readSparseMatrix(const char *filename);
void print(const int &n, ...);

#endif /*AUXILIARY_H*/
