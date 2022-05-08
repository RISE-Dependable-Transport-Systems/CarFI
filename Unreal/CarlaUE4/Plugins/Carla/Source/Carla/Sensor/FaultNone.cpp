#pragma once

#include "Fault.h"

class FaultNone : public FFault
{
private:


public:
    FaultNone() { }
        carla::Buffer apply(carla::Buffer &&buf)
    {
return std::move(buf);
       
    }
};
