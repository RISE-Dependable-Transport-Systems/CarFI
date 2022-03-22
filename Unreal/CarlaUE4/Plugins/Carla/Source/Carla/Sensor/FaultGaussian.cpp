#ifndef FAULTGAUSSIAN_H_
#define FAULTGAUSSIAN_H_

#include "Fault.cpp"

class FaultGaussian : public FFault
{
private:
    /* data */
    unsigned seed;
    std::normal_distribution<float> distribution;

public:
    FaultGaussian(FString s) : FFault{s}, seed(FCString::Atoi(*param[FString(TEXT("seed"))])), distribution(FCString::Atof(*param[FString(TEXT("distMin"))]), FCString::Atof(*param[FString(TEXT("distMax"))])) {}

    void apply(TArray<FColor> &img)
    {
        std::default_random_engine generator(seed);

        GEngine->AddOnScreenDebugMessage(4, 10.f, FColor::Red, param[FString(TEXT("seed"))]);
        GEngine->AddOnScreenDebugMessage(2, 10.f, FColor::Red, param[FString(TEXT("distMin"))]);

        GEngine->AddOnScreenDebugMessage(8, 10.f, FColor::Red, param[FString(TEXT("distMax"))]);

        for (int32 Index = 0; Index != img.Num(); Index++)
        {
            FColor Pixel = img[Index];
            float Noise = distribution(generator);
            uint8_t NR = Pixel.R * Noise;
            uint8_t NG = Pixel.G * Noise;
            uint8_t NB = Pixel.B * Noise;
            img[Index] = FColor(NR, NG, NB, Pixel.A);
        }
    }
};

#endif // FAULTGAUSSIAN_H_
