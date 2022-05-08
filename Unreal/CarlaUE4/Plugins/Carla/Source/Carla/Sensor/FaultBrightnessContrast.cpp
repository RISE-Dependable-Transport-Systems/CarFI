#pragma once

#include "Fault.h"

class FaultBrightnessAndContrast : public FFault
{
private:
    /* data */
    float alpha;
    int beta;

public:
    FaultBrightnessAndContrast(FString s) : FFault{s}, alpha(FCString::Atof(*param[FString(TEXT("alpha"))])), beta(FCString::Atoi(*param[FString(TEXT("beta"))]))
    {
        GEngine->AddOnScreenDebugMessage(4, 10.f, FColor::Red, FString::SanitizeFloat(alpha));
        GEngine->AddOnScreenDebugMessage(2, 10.f, FColor::Red, FString::FromInt(beta));
    }

    carla::Buffer apply(carla::Buffer &&buf)

    {

       

        for (auto ptr = buf.begin(); ptr < buf.end(); ptr++)
        {
            *ptr = (*ptr * alpha) + beta;
           
        }
        return std::move(buf);

      
    }
};
