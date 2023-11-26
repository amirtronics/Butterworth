#pragma once

#ifndef FILTER_H
#define FILTER_H

#include <vector>


namespace filter
{
  struct DataIndex
  {
      int t;
      int x_anafi, y_anafi, z_anafi;
      int x_bebop, y_bebop, z_bebop;

      int x_rel, y_rel, z_rel;
      int x_est, y_est, z_est;

  };

  class ButterworthFilter
   {
    public:
      int n; // Filter Order
      std::vector<float> a_, b_, x, y; // Filter Coefficients

      void init(int filt_order, float *a, float *b);
      float filter(float sample);
      std::vector<float> shift_array(float x0, std::vector<float> vector_);

    private:

   };
}

#endif
