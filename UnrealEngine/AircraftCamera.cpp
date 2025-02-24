// Fill out your copyright notice in the Description page of Project Settings.

#include "AircraftCamera.h"

#include "CameraSim.h"
#include "Runtime/Engine/Classes/Engine/TextureRenderTarget2D.h"
#include "Components/SphereComponent.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Kismet/GameplayStatics.h"
#include "PixelFormat.h"
#include <iostream>
#include <fstream>
#include <iomanip>

#include "CesiumGeoreference.h"
#include "CesiumCameraManager.h"

//#include "Camera/CameraComponent.h"


#include "Geo.h"

// Sets default values
AAircraftCamera::AAircraftCamera()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	RootComponent = CreateDefaultSubobject<USphereComponent>(TEXT("Root"));

	Camera = CreateDefaultSubobject<USceneCaptureComponent2D>(TEXT("Camera"));
	Camera->SetupAttachment(RootComponent);

	//Camera->RegisterComponent();

	ConstructorHelpers::FObjectFinder<UTextureRenderTarget2D> RenderTargetAsset(TEXT("/Script/Engine.TextureRenderTarget2D'/Game/StarterContent/Textures/CameraRenderTarget.CameraRenderTarget'"));

	RenderTarget = DuplicateObject(RenderTargetAsset.Object, NULL);

	image_size_x = RenderTargetAsset.Object->GetSurfaceWidth();
	image_size_y = RenderTargetAsset.Object->GetSurfaceHeight();

	//RenderTarget->InitCustomFormat(image_size_x, image_size_y, RenderTargetAsset.Object->GetFormat(), false);
	RenderTarget->InitAutoFormat(image_size_x, image_size_y);

	Camera->TextureTarget = RenderTarget;

	// Limit to 20Hz
	PrimaryActorTick.TickInterval = 0.03333f;

	port=8000;
	#ifdef CSV_OUTPUT
		std::ofstream f;
		f.open(CSV_OUTPUT);
		f << "timestamp, lat, lon, x, y\n";
		f.close();

	#endif
}


// Called when the game starts or when spawned
void AAircraftCamera::BeginPlay()
{
	Super::BeginPlay();	

	// Setup JSBSim interface //
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

	TArray<AActor*> player_cameras;
	//UGameplayStatics::GetAllActorsWithTag(GetWorld(), TEXT("DEFAULT_GEOREFERENCE"), geos);
	UGameplayStatics::GetAllActorsOfClass(GetWorld(), APlayerCameraManager::StaticClass(), player_cameras);
	if (geos.Num() > 0){
		this->player_camera_manager = player_cameras[0];
		GEngine->AddOnScreenDebugMessage(-1, 10.0f, FColor::Green, FString::Printf(TEXT("Camera Found!")));
	}
	else {
		GEngine->AddOnScreenDebugMessage(-1, 10.0f, FColor::Red, FString::Printf(TEXT("Warning: Could not find Cesium Camera Manager")));
	}



	// Setup Shared Memory //

	key = ftok("/usr/share/ue5camsim.data", 10);

	// The size is the image + an 8 byte u_long for message indexing
	//shmid = shmget(key, image_size_x * image_size_y * 3 + 16, 0666|IPC_CREAT);
	shmid = shmget(key, 1920 * 1080 * 3 + 16, 0666|IPC_CREAT);

	if (shmid == -1){
		{
			UE_LOG(LogTemp, Warning, TEXT("1 ERROR CREATING SHARED MEMORY SEGMENT"));
		}
	}
	else {
		data = (uint8_t*)shmat(shmid, NULL, 0);
		if (data == (void*)-1){
			UE_LOG(LogTemp, Warning, TEXT("2 ERROR CREATING SHARED MEMORY SEGMENT"));
		}
	}

	Camera->TextureTarget = RenderTarget;
	Texture2D = RenderTarget->ConstructTexture2D(this, FString("Tex2D"), EObjectFlags::RF_NoFlags);

	PixelData.AddUninitialized(image_size_x * image_size_y);

}

void AAircraftCamera::SendImageData(TArray<FColor> &texture_pixels){

	static u_long msg_id = 0;
	if(shmid != -1 && data != (void*)-1)
	{
		/* Pack the message ID as the first 8 bytes (64 bit unsigned integer)*/
		msg_id++;
		u_long *q = (u_long*)data;
		*q = msg_id;
		q++;

		/* Pack the x and y image size as 32 bit integers */
		int32_t *p = (int32_t*)q;
		*p = (int32_t)image_size_x;
		p++;
		*p = (int32_t)image_size_y;
		p++;

		/* Back to uint8_t format */
		uint8_t* d = (uint8_t*)p;

		std::cout << "Length of array: " << texture_pixels.Num() << std::endl;

		for (u_long i=0; i<texture_pixels.Num(); i++){
			*d = texture_pixels[i].R;
			d++;
			*d = texture_pixels[i].G;
			d++;
			*d = texture_pixels[i].B;
			d++;
		}
	}
}


bool AAircraftCamera::GetCameraPixelData(){
	FTextureRenderTargetResource *RenderTargetResource = RenderTarget->GameThread_GetRenderTargetResource();
	bool bReadSuccess = RenderTargetResource->ReadPixels(PixelData);
	return bReadSuccess;
}

// Called every frame
void AAircraftCamera::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if(GetCameraPixelData()) {
		//GEngine->AddOnScreenDebugMessage(-1, 10.0f, FColor::Green, PixelData[100].ToString() );
		SendImageData(PixelData);
	}
	else {
		GEngine->AddOnScreenDebugMessage(-1, 10.0f, FColor::Red, FString::Printf(TEXT("Could not get pixel data")));
	}


	auto actor = this;

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


		//FVector2D pos = get_xy_offset_from_origin(origin_lat, origin_lon, lat, lon);
		FVector3d pos = vector_offset_from_origin(origin_lat, origin_lon, origin_alt, lat, lon, alt);
		//GEngine->AddOnScreenDebugMessage(-1, 10.0f, FColor::Blue, FString::Printf(TEXT("Origin Lat: %.6f, Origin Lon %.6f, Origin Alt %.6f"), origin_lat, origin_lon, origin_alt));
		//GEngine->AddOnScreenDebugMessage(-1, 10.0f, FColor::Blue, FString::Printf(TEXT("POSITION: %.8f, %.8f, %.8f"), pos[0], pos[1], pos[2]));

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
		GEngine->AddOnScreenDebugMessage(-1, 10.0f, FColor::Blue, FString::Printf(TEXT("Failed to read JSBSim Data")));
	}

}

void AAircraftCamera::EndPlay(const EEndPlayReason::Type EndPlayReason){
	Super::EndPlay(EndPlayReason);

	// Shutdown shared memory
	shmdt(data);
	jsbsim->close();
}