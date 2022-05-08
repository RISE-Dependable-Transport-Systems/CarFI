#pragma once

#include "Fault.h"

class FaultBlending : public FFault
{
private:
    /* data */
    double alpha;
    unsigned int beta;

public:
    FaultBlending(double _alpha, unsigned int _beta, FString s) : FFault{s}, alpha(_alpha), beta(_beta) {}
    carla::Buffer apply(carla::Buffer &&buf)
    {
//TODO: Read image in carla


        for (auto p = buf.begin(); p < buf.end(); p++)
        {

            *p = (*p * alpha) + beta;
          
        }
        return std::move(buf);

    }
};
