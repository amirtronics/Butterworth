// Author:
/*
 * Node: butter.cpp
 * Author: Amir Hossein Ebrahimnezhad
 * Date: Nov 21/22
*/

#include "Filter.h"

namespace filter 
{
    void ButterworthFilter::init(int filt_order, float *a, float *b)
    {   
        n = filt_order;

        for(int  j = 0; j < n; j++)
        {
            a_.push_back(a[j]);
            b_.push_back(b[j]);

            x.push_back(0.0);
            y.push_back(0.0);
        }
    }


    float ButterworthFilter::filter(float sample)
    {
        x = shift_array(sample, x);
        y = shift_array(0.0, y);

        float sum = 0.0;

        for (int  j = 1; j < n; j++)
        {
            sum += (b_[j]*x[j] - a_[j]*y[j]);
        }

        sum += b_[0]*x[0];

        y[0] = sum;

        return y[0];

    }

    std::vector<float> ButterworthFilter::shift_array(float x0, std::vector<float> vector_)
    {
        for(int  j = n-1; j > 0; j--)
        {
            vector_[j] = vector_[j-1];
        }
        vector_[0] = x0;

        return vector_;
    }
    
}


