#include "pch.h"
#include "Utils.h"

int Utils::CreateTrafficProviderSocket()
{
	std::string ip = "127.0.0.1";
	std::string port = "21568";
	int clientSocket;

	try
	{

#ifdef _WIN32
		WSADATA wsaData;
		WSAStartup(0x0202, &wsaData);
#else
#endif
		struct sockaddr_in clientConnection;
#ifdef _WIN32
#else
		bzero(&clientConnection, sizeof(clientConnection));
#endif

		struct in_addr ipAddress;
		inet_pton(AF_INET, ip.c_str(), (void *)&ipAddress);
		clientConnection.sin_family = AF_INET;
		clientConnection.sin_port = htons(atoi(port.c_str()));
		clientConnection.sin_addr.s_addr = ipAddress.s_addr;
		clientSocket = (int)socket(AF_INET, SOCK_STREAM, IPPROTO_HOPOPTS);

		int error = connect(clientSocket, (struct sockaddr*)&clientConnection, sizeof(clientConnection));
#ifdef _WIN32
		if (SOCKET_ERROR == error)
		{
			closesocket(clientSocket);
			WSACleanup();
#else
		if (-1 == error)
		{
			close(clientSocket);
#endif
			return false;
		}
	}
	catch (...)
	{
		return false;
	}

	return clientSocket;
}

bool Utils::SendDataBySocket(int clientSocket, const std::string& data)
{
	try
	{
		send(clientSocket, data.c_str(), (int)(data.length()), 0);
	}
	catch (...)
	{
		return false;
	}

	return true;
}
