#pragma once

#include <iostream>

#ifdef _WIN32
#include "WS2tcpip.h"
#pragma comment(lib,"ws2_32.lib")
#include <direct.h>
#else
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <errno.h>
#include <memory.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif

#define BUFFER_SIZE			1024

class Utils
{
public:

	static int CreateTrafficProviderSocket();
	static bool SendDataBySocket(int clientSocket, const std::string& data);
};

