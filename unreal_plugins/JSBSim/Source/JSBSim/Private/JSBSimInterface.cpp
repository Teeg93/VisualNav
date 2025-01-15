// Fill out your copyright notice in the Description page of Project Settings.


#include "JSBSimInterface.h"
#include "Math/MathFwd.h"
#include "Kismet/GameplayStatics.h"
#include "Geo.h"
#include <iostream>
#include <fstream>
#include <iomanip>
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/TextureRenderTarget2D.h"

#include "CesiumGeoreference.h"

// Sets default values for this component's properties
UJSBSimInterface::UJSBSimInterface()
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = true;

	port = 8000;
	time_offset_hours = 0;
	time_offset_minutes = 0;
	time_offset_seconds = 0;

	previous_timestamp = 0;

	aircraft_camera = CreateDefaultSubobject<USceneCaptureComponent2D>(TEXT("aircraft_camera"));
	aircraft_camera->SetupAttachment(this);
	aircraft_camera->RegisterComponent();

	aircraft_camera_image_width = 0;
	aircraft_camera_image_height = 0;
	aircraft_camera_image_size = 0;

	#ifdef CSV_OUTPUT
		std::ofstream f;
		f.open(CSV_OUTPUT);
		f << "timestamp, lat, lon, x, y\n";
		f.close();

	#endif
	// ...
}



bool UJSBSimInterface::GetCameraPixelData(){
	FTextureRenderTargetResource* RenderTargetResource = aircraft_camera_render_target->GameThread_GetRenderTargetResource();
	bool bReadSuccess = RenderTargetResource->ReadPixels(aircraft_camera_pixel_data);
	return bReadSuccess;
}


// Called when the game starts
void UJSBSimInterface::BeginPlay()
{
	Super::BeginPlay();
	jsbsim = new JSBSim(port, 0);

	TArray<AActor*> geos;
	//UGameplayStatics::GetAllActorsWithTag(GetWorld(), TEXT("DEFAULT_GEOREFERENCE"), geos);
	UGameplayStatics::GetAllActorsOfClass(GetWorld(), ACesiumGeoreference::StaticClass(), geos);

	//auto actor = this->GetOwner();

	aircraft_camera->TextureTarget = aircraft_camera_render_target;

	//aircraft_camera_image_width = aircraft_camera_render_target->SizeX;
	//aircraft_camera_image_height = aircraft_camera_render_target->SizeY;
	//aircraft_camera_image_size = aircraft_camera_image_width * aircraft_camera_image_height;

	aircraft_camera_pixel_data.AddUninitialized(1920*1080);

	if (geos.Num() > 0){
		this->georeference = (ACesiumGeoreference *)geos[0];
		GEngine->AddOnScreenDebugMessage(-1, 10.0f, FColor::Green, FString::Printf(TEXT("Georeference found")));

		this->origin_lat = georeference->GetOriginLatitude();
		this->origin_lon = georeference->GetOriginLongitude();
		this->origin_alt = georeference->GetOriginHeight();

		GEngine->AddOnScreenDebugMessage(-1, 10.0f, FColor::Blue, FString::Printf(TEXT("%.2f, %.2f, %.2f"), origin_lat, origin_lon, origin_alt ));


	}
	else {
		GEngine->AddOnScreenDebugMessage(-1, 10.0f, FColor::Red, FString::Printf(TEXT("Warning: Could not find GeoReference")));
	}


	// ...
	
}


// Called every frame
void UJSBSimInterface::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	auto actor = this->GetOwner();
	FVector loc = actor->GetActorLocation();

	int ret = jsbsim->recv();
	if (ret >= 0 && jsbsim->timestamp > previous_timestamp){
		double pitch = jsbsim->pitch;
		double roll = jsbsim->roll;
		double yaw = jsbsim->yaw;
		double lat = jsbsim->lat;
		double lon = jsbsim->lon;
		double alt = jsbsim->alt;
		double timestamp = jsbsim->timestamp;
		previous_timestamp = timestamp;


		FVector2D pos = get_xy_offset_from_origin(origin_lat, origin_lon, lat, lon);

		FVector3d location = {pos.X * 100.0, -pos.Y * 100.0, alt * 100.0};
		FRotator rotation = {-roll, yaw, pitch };

		#ifdef CSV_OUTPUT
			std::ofstream f;
			f.open(CSV_OUTPUT, std::ios_base::app); 
			
			//timestamp, lat, lon, x, y\n
			f << std::fixed << std::setprecision(9) << timestamp << "," << lat << "," << lon << "," << location[0] << "," << location[1] << "\n";
			f.close();
		#endif

		actor->SetActorLocation(location, false);
		actor->SetActorRotation(rotation);

	}
	else {
		//GEngine->AddOnScreenDebugMessage(-1, 10.0f, FColor::Blue, FString::Printf(TEXT("Failed to read")));
	}

	//if(GetCameraPixelData()){
		//GEngine->AddOnScreenDebugMessage(-1, 10.0f, FColor::Blue, FString::Printf(TEXT("Got Camera Pixel Data")));
	//}
	//else {
		//GEngine->AddOnScreenDebugMessage(-1, 10.0f, FColor::Red, FString::Printf(TEXT("Failed to get Camera Pixel Data")));
	//}

	//GEngine->AddOnScreenDebugMessage(-1, 10.0f, FColor::Blue, FString::Printf(TEXT("%.2f, %.2f, %.2f"), loc[0], loc[1], loc[2]));
	// ...
}


void UJSBSimInterface::EndPlay(const EEndPlayReason::Type EndPlayReason){
	Super::EndPlay(EndPlayReason);

	jsbsim->close();
}