// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "EngineMinimal.h"

#include "JSBSim.h"
#include "CesiumGeoreference.h"

#include "JSBSimInterface.generated.h"


UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class UJSBSimInterface : public UActorComponent
{
	GENERATED_BODY()

public:	
	// Sets default values for this component's properties
	UJSBSimInterface();

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Networking") int port;

protected:
	// Called when the game starts
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	ACesiumGeoreference * georeference;
	JSBSim * jsbsim;

	double origin_lat;
	double origin_lon;
	double origin_alt;

public:	
	// Called every frame
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
};
