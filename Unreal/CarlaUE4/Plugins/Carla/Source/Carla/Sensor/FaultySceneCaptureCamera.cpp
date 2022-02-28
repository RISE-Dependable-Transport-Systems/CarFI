// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla.h"
#include "Carla/Sensor/FaultySceneCaptureCamera.h"
#include "stdint.h"
#include "Runtime/RenderCore/Public/RenderingThread.h"

FActorDefinition AFaultySceneCaptureCamera::GetSensorDefinition()
{
  constexpr bool bEnableModifyingPostProcessEffects = true;
  return UActorBlueprintFunctionLibrary::MakeCameraDefinition(
      TEXT("frgb"),
      bEnableModifyingPostProcessEffects);
}

AFaultySceneCaptureCamera::AFaultySceneCaptureCamera(const FObjectInitializer &ObjectInitializer)
  : Super(ObjectInitializer)
{
  AddPostProcessingMaterial(
      TEXT("Material'/Carla/PostProcessingMaterials/PhysicLensDistortion.PhysicLensDistortion'"));
  
}

void AFaultySceneCaptureCamera::PostPhysTick(UWorld *World, ELevelTick TickType, float DeltaSeconds)
{
  TRACE_CPUPROFILER_EVENT_SCOPE(ASceneCaptureCamera::PostPhysTick);
  check(CaptureRenderTarget != nullptr);
  if (!HasActorBegunPlay() || IsPendingKill())
  {
    return;
  }

  /// Immediate enqueues render commands of the scene at the current time.
  EnqueueRenderSceneImmediate();
  WaitForRenderThreadToFinsih();

  //Super (ASceneCaptureSensor) Capture the Scene in a (UTextureRenderTarget2D) CaptureRenderTarge from the CaptureComponent2D
  /** Read the image **/
  TArray<FColor> RawImage;
  this->ReadPixels(RawImage);  
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);
  std::normal_distribution<float> distribution (0.0,1.0);
 
  for (int32 Index = 0; Index != RawImage.Num(); ++Index)
  {
     FColor Pixel = RawImage[Index];
     float Noise = distribution(generator);
     uint8_t NR = Pixel.R * Noise;
     uint8_t NG = Pixel.G * Noise;
     uint8_t NB = Pixel.B * Noise;
     RawImage[Index] = FColor(NR, NG, NB, Pixel.A);
  } 
  
  FPixelReader::SendFaultyPixelsInRenderThread(*this, RawImage, false);
}
