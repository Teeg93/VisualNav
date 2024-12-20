// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "Modules/ModuleManager.h"
#include <netinet/in.h>
#include <ctime>
#include <vector>
#include <string>
#include <chrono>

#include "CoreMinimal.h"

#define BUFFERSIZE 1024

class JSBSim
{
public:
	JSBSim(int port, double initial_timestamp);
	~JSBSim();

	int recv();
	bool is_alive();
	void close();

	double timestamp;
	double roll;
	double pitch;
	double yaw;
	double lat;
	double lon;
	double alt;


private:

	double _initial_timestamp;
	double _timestamp_offset = 0;

	int unpack_buffer();
	std::vector<std::string> split(std::string *s, char delim);
	double str_to_double(std::string *s);

	//auto initialization_time = std::chrono::high_resolution_clock::now(); 
	std::chrono::time_point<std::chrono::high_resolution_clock> initialization_time = std::chrono::high_resolution_clock::now(); 

	bool first_iter = true;


	std::time_t last_recv_time;

	int socket_fd;
	struct sockaddr_in server_addr;
	struct sockaddr_in client_addr;
	char buffer[BUFFERSIZE];


};
