// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "AircraftCamera.generated.h"

UCLASS()
class CAMERASIM_API AAircraftCamera : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AAircraftCamera();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	TArray<FColor> PixelData;
	bool GetCameraPixelData();
	USceneCaptureComponent2D *Camera;
	UTextureRenderTarget2D *RenderTarget;
	UTexture2D *Texture2D;



public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

};
