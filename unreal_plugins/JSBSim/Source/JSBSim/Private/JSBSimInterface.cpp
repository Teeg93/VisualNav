// Fill out your copyright notice in the Description page of Project Settings.


#include "JSBSimInterface.h"
#include "Math/MathFwd.h"
#include "Kismet/GameplayStatics.h"
#include "Geo.h"

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

	previous_x_loc = 0;
	previous_y_loc = 0;

	// ...
}


// Called when the game starts
void UJSBSimInterface::BeginPlay()
{
	Super::BeginPlay();
	jsbsim = new JSBSim(port, 0);

	TArray<AActor*> geos;
	//UGameplayStatics::GetAllActorsWithTag(GetWorld(), TEXT("DEFAULT_GEOREFERENCE"), geos);
	UGameplayStatics::GetAllActorsOfClass(GetWorld(), ACesiumGeoreference::StaticClass(), geos);


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

		/*
		bool high_motion_flag = false;

		if (abs(pos.X - previous_x_loc) > 2 ){
			high_motion_flag = true;
			GEngine->AddOnScreenDebugMessage(-1, 10.0f, FColor::Red, FString::Printf(TEXT("Previous x loc: %.5f,  Current x loc: %.5f"),  previous_x_loc, pos.X ));
			GEngine->AddOnScreenDebugMessage(-1, 10.0f, FColor::Red, FString::Printf(TEXT("Lat: %.8f,  Lon: %.8f"),  lat, lon ));
		}

		if (abs(pos.Y - previous_y_loc) > 2 ){
			high_motion_flag = true;
			GEngine->AddOnScreenDebugMessage(-1, 10.0f, FColor::Red, FString::Printf(TEXT("Previous y loc: %.5f,  Current y loc: %.5f"),  previous_y_loc, pos.Y ));
			GEngine->AddOnScreenDebugMessage(-1, 10.0f, FColor::Red, FString::Printf(TEXT("Lat: %.8f,  Lon: %.8f"),  lat, lon ));
		}
		*/


		previous_x_loc = pos.X;
		previous_y_loc = pos.Y;

		//GEngine->AddOnScreenDebugMessage(-1, 10.0f, FColor::Blue, FString::Printf(TEXT("%.5f, %.5f, %.5f"), pos.X, pos.Y, alt ));
		//GEngine->AddOnScreenDebugMessage(-1, 10.0f, FColor::Blue, FString::Printf(TEXT("%.7f, %.7f"), lat, lon));

		//if (!high_motion_flag){
			actor->SetActorLocation(location, false);
			actor->SetActorRotation(rotation);
		//}

	}
	else {
		GEngine->AddOnScreenDebugMessage(-1, 10.0f, FColor::Blue, FString::Printf(TEXT("Failed to read")));
	}

	//GEngine->AddOnScreenDebugMessage(-1, 10.0f, FColor::Blue, FString::Printf(TEXT("%.2f, %.2f, %.2f"), loc[0], loc[1], loc[2]));
	// ...
}


void UJSBSimInterface::EndPlay(const EEndPlayReason::Type EndPlayReason){
	Super::EndPlay(EndPlayReason);

	jsbsim->close();
}