// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "EngineMinimal.h"

#include "JSBSim.h"
#include "CesiumGeoreference.h"

#include "Components/SceneCaptureComponent2D.h"

#include "JSBSimInterface.generated.h"

// #define CSV_OUTPUT "/tmp/unreal_camera_sim.csv"

UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class UJSBSimInterface : public USceneComponent
{
	GENERATED_BODY()

public:	
	// Sets default values for this component's properties
	UJSBSimInterface();

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Networking") int port;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Time of day") int time_offset_hours;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Time of day") int time_offset_minutes;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Time of day") int time_offset_seconds;

	UPROPERTY(VisibleAnywhere, Category="Components", meta = (AllowPrivateAccess = true))
	class USceneCaptureComponent2D* aircraft_camera;

protected:
	// Called when the game starts
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	ACesiumGeoreference * georeference;
	JSBSim * jsbsim;

	double origin_lat;
	double origin_lon;
	double origin_alt;

	double previous_timestamp;


private:
	// Camera 
	TArray<FColor> aircraft_camera_pixel_data;

	bool GetCameraPixelData();
	UTextureRenderTarget2D* aircraft_camera_render_target;
	int32 aircraft_camera_image_width;
	int32 aircraft_camera_image_height;
	int32 aircraft_camera_image_size;


public:	
	// Called every frame
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
};
