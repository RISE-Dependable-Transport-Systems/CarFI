// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla/Sensor/FaultySceneCaptureCamera.h"
#include "Carla.h"
#include "Runtime/RenderCore/Public/RenderingThread.h"
#include "stdint.h"
#include "FaultGaussian.cpp"
#include "FaultBrightnessContrast.cpp"
using namespace std::chrono;
/*
TODO: parse and use the parameters in the sensors
TODO: event paramerts use time in ns and use GETEpisode().getGameTime() to trigger at given time periods.
TODO: refactor code to be included into set method for persistance.
*/
FActorDefinition AFaultySceneCaptureCamera::GetSensorDefinition()
{
  constexpr bool bEnableModifyingPostProcessEffects = true;

  FActorVariation faultyParameters;
  faultyParameters.Id = TEXT("faulty_parameters");
  faultyParameters.Type = EActorAttributeType::String;
  faultyParameters.RecommendedValues = {TEXT("newline seperated Parameter:value")};
  faultyParameters.bRestrictToRecommended = false;

  FActorVariation faultyEventParameters;
  faultyEventParameters.Id = TEXT("faulty_parameters");
  faultyEventParameters.Type = EActorAttributeType::String;
  faultyEventParameters.RecommendedValues = {TEXT("newline seperated StartTime:TimeInNS")};
  faultyEventParameters.bRestrictToRecommended = false;

  FActorVariation faultType;
  faultType.Id = TEXT("fault_type");
  faultType.Type = EActorAttributeType::String;
  faultType.RecommendedValues = {TEXT("None")};
  faultType.bRestrictToRecommended = false;

  auto Definition = UActorBlueprintFunctionLibrary::MakeCameraDefinition(
      TEXT("frgb"), bEnableModifyingPostProcessEffects);
  Definition.Variations.Append({faultyParameters, faultType, faultyEventParameters});
  return Definition;
}

void AFaultySceneCaptureCamera::Set(const FActorDescription &Description)
{
  Super::Set(Description);
  faultyParameters =
      UActorBlueprintFunctionLibrary::RetrieveActorAttributeToString(
          "faulty_parameters", Description.Variations, FString(TEXT("None:None")));

  faultyEventParameters =
      UActorBlueprintFunctionLibrary::RetrieveActorAttributeToString(
          "faulty_event_parameters", Description.Variations, FString(TEXT("None:None")));
  faultType =
      UActorBlueprintFunctionLibrary::RetrieveActorAttributeToString(
          "fault_type", Description.Variations, FString(TEXT("None")));

  /* Thoughts: This fuction is run on every tick. Creating a fault object every time might not be desierable.
It might be a better idea to move the instantiation of the faults
into the SET method and then use the instantiated object with the apply function here.*/
  if (faultType.Equals(FString(TEXT("Gaussian"), ESearchCase::IgnoreCase)))
  {
    GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Blue, faultyParameters);
    // std::cout<< "got faulty parameters " << faultyParameters << std::endl;
    fault = std::make_shared<FaultGaussian>(faultyParameters);
    /*FaultGaussian(this->Seed
      //std::chrono::system_clock::now().time_since_epoch().count(),
       0.0, 1.0);*/
  }
  if (faultType.Equals(FString(TEXT("BrightnessAndContrast"), ESearchCase::IgnoreCase)))
  {
    // alpha value [1.0-3.0] TODO: move this as a parameter
    //  beta value [0-100]
    GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Blue, faultyParameters);

    fault = std::make_shared<FaultBrightnessAndContrast>(faultyParameters); // 1.0, 20);
  }
  else if (faultType.Equals(FString(TEXT("None"), ESearchCase::IgnoreCase)))
  {
    fault = std::make_shared<FaultNone>();
  }
  else
  {
    GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Red, FString(TEXT("Got invalid  fault type") + faultType));
  }
}
AFaultySceneCaptureCamera::AFaultySceneCaptureCamera(
    const FObjectInitializer &ObjectInitializer)
    : Super(ObjectInitializer)
{
  AddPostProcessingMaterial(TEXT("Material'/Carla/PostProcessingMaterials/"
                                 "PhysicLensDistortion.PhysicLensDistortion'"));
}

void AFaultySceneCaptureCamera::PostPhysTick(UWorld *World, ELevelTick TickType,
                                             float DeltaSeconds)
{



  

  FPixelReader::SendFaultyPixelsInRenderThread(*this,fault);

}
