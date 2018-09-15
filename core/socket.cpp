#include "socket.h"

void init_socket(SOCKET &sclient, sockaddr_in &sin, int &len, int port, string &addr, int self_port)
{
	WORD socketVersion = MAKEWORD(2, 2);
	WSADATA wsaData;
	if (WSAStartup(socketVersion, &wsaData) != 0)
	{
		perror("init socket error");

		exit(-1);
	}

	sclient = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

	sin.sin_family = AF_INET;
	sin.sin_port = htons(port);
	sin.sin_addr.S_un.S_addr = inet_addr(addr.c_str());

	len = sizeof(sin);

	sockaddr_in bind_Addr;
	bind_Addr.sin_family = AF_INET;
	bind_Addr.sin_port = htons(self_port);
	bind_Addr.sin_addr.S_un.S_addr = INADDR_ANY;

	if (bind(sclient, (sockaddr *)&bind_Addr, sizeof(bind_Addr)) == SOCKET_ERROR)
	{
		printf("bind error !\n");
		closesocket(sclient);
		
		exit(0);
	}
}

void des_socket(SOCKET sclient)
{
	closesocket(sclient);
	WSACleanup();
}