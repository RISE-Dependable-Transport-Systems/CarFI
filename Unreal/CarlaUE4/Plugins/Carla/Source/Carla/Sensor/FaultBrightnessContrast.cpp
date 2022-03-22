#pragma once

#include "Fault.cpp"

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

    void apply(TArray<FColor> &img)
    {

        for (int32 Index = 0; Index != img.Num(); Index++)
        {
            FColor Pixel = img[Index];
            uint8_t NR = (Pixel.R * alpha) + beta;
            uint8_t NG = (Pixel.G * alpha) + beta;
            uint8_t NB = (Pixel.B * alpha) + beta;
            img[Index] = FColor(NR, NG, NB, Pixel.A);
        }
    }
};
