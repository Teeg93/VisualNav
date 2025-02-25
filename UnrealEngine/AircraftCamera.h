// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "sys/ipc.h"
#include "sys/shm.h"
#include "JSBSim.h"
#include "CesiumGeoreference.h"
#include "CesiumGlobeAnchorComponent.h"
//#include "Camera/CameraComponent.h"

#include "AircraftCamera.generated.h"


UCLASS()
class CAMERASIM_API AAircraftCamera : public APawn
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AAircraftCamera();

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Networking") int port;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Time of day") int time_offset_hours;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Time of day") int time_offset_minutes;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Time of day") int time_offset_seconds;

protected:	

	// Members for JSBSim
	ACesiumGeoreference * georeference;
	AActor * player_camera_manager;
	JSBSim * jsbsim;

	double origin_lat;
	double origin_lon;
	double origin_alt;
	double previous_timestamp;


	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;


	// Transfer image data via shared memory
	void SendImageData(TArray<FColor> &texture_pixels);

	TArray<FColor> PixelData;
	bool GetCameraPixelData();

	UPROPERTY(VisibleAnywhere, Category="Components", meta = (AllowPrivateAccess = true))
	USceneCaptureComponent2D *Camera;

	UTextureRenderTarget2D *RenderTarget;
	UTexture2D *Texture2D;

	UCesiumGlobeAnchorComponent *cesium_globe_anchor;

	int32_t image_size_x;
	int32_t image_size_y;

	key_t key;
	int shmid;
	uint8_t* data;


public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

};
