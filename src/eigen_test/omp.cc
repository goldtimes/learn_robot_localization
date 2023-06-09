#include "omp.h"

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
//  g++ omp.cc -o omp -fopenmp
int main(int argc, char **) {
  int width = 1280;
  int height = 1280;
  float *imageBuffer = new float[3 * width * height];
  omp_set_num_threads(4);
#pragma omp parallel
  {
    int tid = omp_get_thread_num();
    for (int i = 0; i < width * height; i++) {
      imageBuffer[i] = 0;
      imageBuffer[width * height + i] = 255;
      imageBuffer[width * height * 2 + i] = 0;
      std::cout << "tid:" << tid << std::endl;
    }
  }
  return 0;
}