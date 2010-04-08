/* Client code in C */
 
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
 
int main(int argc, char **argv)
{
  struct sockaddr_in stSockAddr;
  int Res;
  int SocketFD = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
  int in_fd;
  ssize_t sent;

  if (argc != 5) {
    printf("Usage: %s <addr> <port> <file> <count> \n", argv[0]);
    printf("\n");
    printf("\t<addr> -- remote address to connect to\n");
    printf("\t<port> -- TCP port number to connect to\n");
    printf("\t<file> -- input file to send data from\n");
    printf("\t<count> -- amount of bytes to send\n");
    printf("\n");
    exit(255);
  }

  if (-1 == SocketFD)
    {
      perror("Cannot create socket");
      exit(EXIT_FAILURE);
    }
 
  memset(&stSockAddr, 0, sizeof(struct sockaddr_in));
 
  stSockAddr.sin_family = AF_INET;
  stSockAddr.sin_port = atoi(argv[2]);
  Res = inet_pton(AF_INET, argv[1], &stSockAddr.sin_addr);
 
  if (0 > Res)
    {
      perror("Error: unable to open an Interet family socket");
      close(SocketFD);
      exit(EXIT_FAILURE);
    }
  else if (0 == Res)
    {
      perror("Error: first parameter does not contain a valid IP-address");
      close(SocketFD);
      exit(EXIT_FAILURE);
    }
 
  if (-1 == connect(SocketFD, (const struct sockaddr *)&stSockAddr, sizeof(struct sockaddr_in)))
    {
      perror("Error: connection failed");
      close(SocketFD);
      exit(EXIT_FAILURE);
    }
 
  if ((in_fd = open(argv[3], O_RDONLY)) <= 0) {
	printf("Can not open %s\n", argv[3]);
      }

      if (in_fd > 0) {
	if ((sent = sendfile(SocketFD, in_fd, 0, atoi(argv[4]))) < 0) {
	  perror("Error: unable to sendfile");
	}
      }

      if (sent >= 0) {
	printf("Sent %d bytes successfully\n", sent);
      }

      if (in_fd > 0) {
	close(in_fd);
      }

  shutdown(SocketFD, SHUT_RDWR);
 
  close(SocketFD);
  return 0;
}
