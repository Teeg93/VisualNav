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

	void SendImageData(TArray<FColor> &texture_pixels, int size);


	TArray<FColor> PixelData;
	bool GetCameraPixelData();
	USceneCaptureComponent2D *Camera;
	UTextureRenderTarget2D *RenderTarget;
	UTexture2D *Texture2D;

	int image_size_x;
	int image_size_y;

	int CameraImageWidth;
	int CameraImageHeight;

	key_t key;
	int shmid;
	uint8_t* data;
	struct ImgData
	{
		unsigned long msg_id=0;
		uint8_t data[512*512*3];
	}ImgDataMsg;
	const int ImgDataSz = 512 * 512 * 3;


public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

};
