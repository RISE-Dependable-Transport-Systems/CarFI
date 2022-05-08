#ifndef FAULT_H_
#define FAULT_H_

#include <compiler/disable-ue4-macros.h>
#include <carla/Buffer.h>
#include <carla/sensor/SensorRegistry.h>
#include <compiler/enable-ue4-macros.h>
//#include "Carla.h"
#include <map>

class FFault
{
protected:
    std::map<FString, FString> param;

public:
    virtual carla::Buffer apply(carla::Buffer &&) = 0;
    virtual ~FFault() {};
    FFault(FString s = FString(TEXT("")))
    {
        parseParameters(s);
    }

private:
    struct cmpFString;

    void parseParameters(FString s);
};

#endif // FAULT_H_
