// Fill out your copyright notice in the Description page of Project Settings.

#include "JSBSim.h"
#include <iostream>
#include <algorithm>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <vector>
#include <string>
#include <cstring>
#include <ctime>
#include <iostream>
#include <chrono>
#include "Geo.h"


JSBSim::JSBSim(int port, double initial_timestamp){

    //timeval socket_timeout;
    //socket_timeout.tv_sec = 0;
    //socket_timeout.tv_usec = 10;

    if ( (socket_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0){
        std::cerr << "Failed to open JSBSim Socket!" << std::endl;
        exit(-1);
    }

    int _buffer_size = 1;
    if (setsockopt(socket_fd, SOL_SOCKET, SO_RCVBUF, &_buffer_size, sizeof(int)) < 0){
        std::cerr << "Socket option error!" << std::endl;
    }

    //if (setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, &socket_timeout, sizeof(socket_timeout)) < 0){
        //std::cerr << "Socket option error!" << std::endl;
    //}

    memset(&server_addr, 0, sizeof(server_addr));
    memset(&client_addr, 0, sizeof(client_addr));

    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(port);

    auto result =  bind(socket_fd, (const struct sockaddr*)&server_addr, sizeof(server_addr));
    if (result < 0)
    {
        std::cerr << "Failed to bind JSBSim Socket!" << std::endl;
        std::cout << "Result: " << result << std::endl;
    }

    _initial_timestamp = initial_timestamp;

    std::cout << std::fixed << "Initial Timestamp: " << _initial_timestamp << std::endl;
    timestamp = _initial_timestamp;
    roll = 0;
    pitch = 0;
    yaw = 0;
    lat = 0;
    lon = 0;
}


std::vector<std::string> JSBSim::split(std::string *s, char delim) {
    std::vector<std::string> elems;

    std::string elem;
    for (char c : *s){
        if (c==delim){
            if (elem.empty()){
                continue;
            }
            else{
                std::remove_if(elem.begin(), elem.end(), isspace);
                elems.push_back(elem);
                elem = "";
            }
        }
        else {
            elem = elem + c;
        }
    }
    if (! elem.empty()){
        std::remove_if(elem.begin(), elem.end(), isspace);
        elems.push_back(elem);
    }

    //std::cout << "Number of elements: " << elems.size() << std::endl;
    return elems;
}



int JSBSim::unpack_buffer(){
    std::string s(buffer);
    std::vector<std::string> vals = split(&s, ',');

    if (vals.size() != 10){
        return -1;
    }




    timestamp = atof(vals[0].c_str());

    if (first_iter){
        auto t_now = std::chrono::high_resolution_clock::now();
        auto t_since_startup = std::chrono::duration_cast<std::chrono::microseconds>(t_now-initialization_time);

        long since_startup = t_since_startup.count();
        double d_since_startup = since_startup / 1000000;


        _timestamp_offset = _initial_timestamp - timestamp + d_since_startup;
        first_iter = false;
    }

    // About 2 second offset between JSBSim and SITL...ugly I know, but no other clean way to do this
    timestamp = timestamp + _timestamp_offset;

    //timestamp = _initial_timestamp + atof(vals[0].c_str());

    alt = atof(vals[1].c_str()) * FT_TO_M;
    roll = atof(vals[2].c_str());
    pitch = atof(vals[3].c_str());
    yaw = atof(vals[4].c_str());
    lon = atof(vals[8].c_str());
    lat = atof(vals[9].c_str());

    // std::cout << "Timestamp: " << timestamp << ", Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << ", Lat: " << lat << ", Lon: " << lon << std::endl;
    // std::cout << std::fixed << "Timestamp: " << timestamp << std::endl;
    return 0;
}

int JSBSim::recv(){
    socklen_t len;
    int n = 1;
    len = sizeof(client_addr);

    n = recvfrom(socket_fd, (char *)buffer, BUFFERSIZE, MSG_WAITALL, (struct sockaddr *)&client_addr, &len);
    buffer[n] = '\0';   // Add string termination

    //std::cout << "Message received: " << buffer << std::endl;
    int ret = unpack_buffer();

    if (ret >= 0){
        last_recv_time = std::time(0);
    }
    return ret;
}


void JSBSim::close(){
    ::close(socket_fd);
}

JSBSim::~JSBSim()
{
}


