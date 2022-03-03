// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla/Sensor/FaultySceneCaptureCamera.h"
#include "Carla.h"
#include "Runtime/RenderCore/Public/RenderingThread.h"
#include "stdint.h"

FActorDefinition AFaultySceneCaptureCamera::GetSensorDefinition() {
  constexpr bool bEnableModifyingPostProcessEffects = true;

  FActorVariation perFaultyPixels;
  perFaultyPixels.Id = TEXT("percentage_faulty_pixels");
  perFaultyPixels.Type = EActorAttributeType::Float;
  perFaultyPixels.RecommendedValues = {TEXT("0")};
  perFaultyPixels.bRestrictToRecommended = false;

  auto Definition = UActorBlueprintFunctionLibrary::MakeCameraDefinition(
      TEXT("frgb"), bEnableModifyingPostProcessEffects);
  Definition.Variations.Append({perFaultyPixels});
  return Definition;
}

void AFaultySceneCaptureCamera::Set(const FActorDescription &Description) {
  Super::Set(Description);
  faultyPixelPercentage =
      UActorBlueprintFunctionLibrary::RetrieveActorAttributeToFloat(
          "percentage_faulty_pixels", Description.Variations, 1.0f);
}
AFaultySceneCaptureCamera::AFaultySceneCaptureCamera(
    const FObjectInitializer &ObjectInitializer)
    : Super(ObjectInitializer) {
  AddPostProcessingMaterial(TEXT("Material'/Carla/PostProcessingMaterials/"
                                 "PhysicLensDistortion.PhysicLensDistortion'"));
}

void AFaultySceneCaptureCamera::PostPhysTick(UWorld *World, ELevelTick TickType,
                                             float DeltaSeconds) {
  TRACE_CPUPROFILER_EVENT_SCOPE(ASceneCaptureCamera::PostPhysTick);
  check(CaptureRenderTarget != nullptr);
  if (!HasActorBegunPlay() || IsPendingKill()) {
    return;
  }

  /// Immediate enqueues render commands of the scene at the current time.
  EnqueueRenderSceneImmediate();
  WaitForRenderThreadToFinsih();

  // Super (ASceneCaptureSensor) Capture the Scene in a (UTextureRenderTarget2D)
  // CaptureRenderTarge from the CaptureComponent2D
  /** Read the image **/
  TArray<FColor> RawImage;
  this->ReadPixels(RawImage);
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);

  int nbrOfPixelsToAffect = faultyPixelPercentage * RawImage.Num() / 100;
  // std::normal_distribution<int> distribution(0, RawImage.Num() -
  // nbrOfPixelsToAffect);
  int startIndex = 10; // distribution(generator);
  for (int32 Index = startIndex; Index != startIndex + nbrOfPixelsToAffect;
       ++Index) {
    FColor Pixel = RawImage[Index];
    uint8_t NR = Pixel.R * 0;
    uint8_t NG = Pixel.G * 0;
    uint8_t NB = Pixel.B * 0;
    RawImage[Index] = FColor(NR, NG, NB, Pixel.A);
  }

  FPixelReader::SendFaultyPixelsInRenderThread(*this, RawImage, false);
}
