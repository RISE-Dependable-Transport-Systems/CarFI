#pragma once

#include "Fault.cpp"

class FaultBlending : public FFault
{
private:
    /* data */
    double alpha;
    unsigned int beta;

public:
    FaultBlending(double _alpha, unsigned int _beta, FString s) : FFault{s}, alpha(_alpha), beta(_beta) {}

    void apply(TArray<FColor> &img)
    {
//TODO: Read image in carla
        for (auto p : img)
        {
            p.R = (p.R * alpha) + beta;
            p.G = (p.G * alpha) + beta;
            p.B = (p.B * alpha) + beta;
        }
    }
};
