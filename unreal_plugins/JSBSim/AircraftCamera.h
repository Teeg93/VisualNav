// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "sys/ipc.h"
#include "sys/shm.h"
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
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	void SendImageData(TArray<FColor> &texture_pixels);


	TArray<FColor> PixelData;
	bool GetCameraPixelData();

	UPROPERTY(VisibleAnywhere, Category="Components", meta = (AllowPrivateAccess = true))
	USceneCaptureComponent2D *Camera;

	UTextureRenderTarget2D *RenderTarget;
	UTexture2D *Texture2D;

	int32_t image_size_x;
	int32_t image_size_y;

	key_t key;
	int shmid;
	uint8_t* data;


public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

};
