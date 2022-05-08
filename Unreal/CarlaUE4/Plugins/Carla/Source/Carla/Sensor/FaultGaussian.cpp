#ifndef FAULTGAUSSIAN_H_
#define FAULTGAUSSIAN_H_

#include "Fault.h"

class FaultGaussian : public FFault
{
private:
    /* data */
    unsigned seed;
    std::normal_distribution<float> distribution;

public:
    FaultGaussian(FString s) : FFault{s}, seed(FCString::Atoi(*param[FString(TEXT("seed"))])), distribution(FCString::Atof(*param[FString(TEXT("distMin"))]), FCString::Atof(*param[FString(TEXT("distMax"))])) {}

    carla::Buffer apply(carla::Buffer &&buf)
    {
        std::default_random_engine generator(seed);

        GEngine->AddOnScreenDebugMessage(4, 10.f, FColor::Red, param[FString(TEXT("seed"))]);
        GEngine->AddOnScreenDebugMessage(2, 10.f, FColor::Red, param[FString(TEXT("distMin"))]);

        GEngine->AddOnScreenDebugMessage(8, 10.f, FColor::Red, param[FString(TEXT("distMax"))]);

        for (auto ptr = buf.begin(); ptr < buf.end(); ptr++)
        {
            float Noise = distribution(generator);
            *ptr = *ptr * Noise;
        }
        return std::move(buf);
    }
};

#endif // FAULTGAUSSIAN_H_
