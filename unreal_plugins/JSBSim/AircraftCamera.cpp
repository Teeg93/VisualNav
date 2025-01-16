// Fill out your copyright notice in the Description page of Project Settings.

#include "AircraftCamera.h"

#include "CameraSim.h"
#include "Runtime/Engine/Classes/Engine/TextureRenderTarget2D.h"
#include "Components/SphereComponent.h"
#include "Components/SceneCaptureComponent2D.h"
#include <iostream>

// Sets default values
AAircraftCamera::AAircraftCamera()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	RootComponent = CreateDefaultSubobject<USphereComponent>(TEXT("Root"));
	Camera = CreateDefaultSubobject<USceneCaptureComponent2D>(TEXT("Camera"));

	ConstructorHelpers::FObjectFinder<UTextureRenderTarget2D> RenderTargetAsset(TEXT("/Script/Engine.TextureRenderTarget2D'/Game/StarterContent/Textures/CameraRenderTarget.CameraRenderTarget'"));

	RenderTarget = DuplicateObject(RenderTargetAsset.Object, NULL);
	RenderTarget->InitAutoFormat(256, 256);
	Camera->TextureTarget = RenderTarget;

}


// Called when the game starts or when spawned
void AAircraftCamera::BeginPlay()
{
	Super::BeginPlay();
	key = ftok("/usr/share/ue5camsim.data", 3);

	std::cout << "Size of ImgDataMsg: " << sizeof(ImgDataMsg) << std::endl;
	shmid = shmget(key, sizeof(ImgDataMsg), 0666|IPC_CREAT);

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

	int X = RenderTarget->GetSurfaceHeight();
	int Y = RenderTarget->GetSurfaceWidth();

	Texture2D = RenderTarget->ConstructTexture2D(this, FString("Tex2D"), EObjectFlags::RF_NoFlags);

	int xx = Texture2D->GetSizeX();
	int yy = Texture2D->GetSizeY();

	image_size_x = xx;
	image_size_y = yy;

	int size=X*Y;
	PixelData.AddUninitialized(size);


	CameraImageWidth = RenderTarget->SizeX;
	CameraImageHeight = RenderTarget->SizeY;
	
}

void AAircraftCamera::SendImageData(TArray<FColor> &texture_pixels){

	static u_long msg_id = 0;
	if(shmid != -1 && data != (void*)-1)
	{
		msg_id++;
		u_long *q = (u_long*)data;
		*q = msg_id;
		q++;
		uint8_t* d = (uint8_t*)q;

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
		GEngine->AddOnScreenDebugMessage(-1, 10.0f, FColor::Green, PixelData[100].ToString() );
		SendImageData(PixelData);
	}
	else {
		GEngine->AddOnScreenDebugMessage(-1, 10.0f, FColor::Red, FString::Printf(TEXT("Could not get pixel data")));
	}

}

void AAircraftCamera::EndPlay(const EEndPlayReason::Type EndPlayReason){
	Super::EndPlay(EndPlayReason);

	// Shutdown shared memory
	shmdt(data);
}