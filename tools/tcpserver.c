/* Server code in C */
 
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/sendfile.h>
 
int main(int argc, char **argv)
{
  struct sockaddr_in stSockAddr;
  int SocketFD = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
  int out_fd;
  ssize_t sent;

  if (argc != 4) {
    printf("Usage: %s <port> <file> <count> \n", argv[0]);
    printf("\n");
    printf("\t<port> -- TCP port number to listen on\n");
    printf("\t<file> -- output file to send data to\n");
    printf("\t<count> -- amount of bytes to receive/send\n");
    printf("\n");
    exit(255);
  }
 
  if(-1 == SocketFD)
    {
      perror("[server] Can not create socket");
      exit(EXIT_FAILURE);
    }
 
  memset(&stSockAddr, 0, sizeof(struct sockaddr_in));
 
  stSockAddr.sin_family = AF_INET;
  stSockAddr.sin_port = atoi(argv[1]);
  stSockAddr.sin_addr.s_addr = INADDR_ANY;
 
  if(-1 == bind(SocketFD,(const struct sockaddr *)&stSockAddr, sizeof(struct sockaddr_in)))
    {
      perror("[server] Error: bind failed");
      close(SocketFD);
      exit(EXIT_FAILURE);
    }
 
  if(-1 == listen(SocketFD, 10))
    {
      perror("[server] Error: listen failed");
      close(SocketFD);
      exit(EXIT_FAILURE);
    }
 
  for(;;)
    {
      int ConnectFD = accept(SocketFD, NULL, NULL);
      
      if(0 > ConnectFD)
	{
	  perror("[server] Error: accept failed");
	  close(SocketFD);
	  exit(EXIT_FAILURE);
	}
      
      if ((out_fd = open(argv[2], O_WRONLY | O_CREAT)) <= 0) {
	printf("[server] Can not open %s\n", argv[2]);
      }

      if (out_fd > 0) {
	if ((sent = sendfile(out_fd, ConnectFD, 0, atoi(argv[3]))) < 0) {
	  perror("[server] Error: unable to sendfile");
	}
      }

      if (sent >= 0) {
	printf("[server] Sent %d bytes successfully\n", sent);
      }

      if (out_fd > 0) {
	close(out_fd);
      }
      
      shutdown(ConnectFD, SHUT_RDWR);
      
      close(ConnectFD);
    }

  close(SocketFD);
  return 0;
}
