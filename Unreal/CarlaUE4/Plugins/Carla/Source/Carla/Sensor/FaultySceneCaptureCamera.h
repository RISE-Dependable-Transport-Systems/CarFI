// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "Carla/Actor/ActorDefinition.h"
#include "Carla/Sensor/PixelReader.h"
#include "Carla/Sensor/ShaderBasedSensor.h"

#include "FaultySceneCaptureCamera.generated.h"

/// A sensor that captures images from the scene.
UCLASS()
class CARLA_API AFaultySceneCaptureCamera : public AShaderBasedSensor
{
  GENERATED_BODY()

public:
  static FActorDefinition GetSensorDefinition();
  void Set(const FActorDescription &ActorDescription) override;
  AFaultySceneCaptureCamera(const FObjectInitializer &ObjectInitializer);

protected:
  void PostPhysTick(UWorld *World, ELevelTick TickType,
                    float DeltaSeconds) override;

private:
  float faultyPixelPercentage;
  FString faultType;
  FString faultyParameters;
  FString faultyEventParameters;
  std::shared_ptr<FFault> fault;
  TUniquePtr<TImagePixelData<FColor>> pixelData;
};
