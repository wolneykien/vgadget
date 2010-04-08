/* Server code in C */

#define _GNU_SOURCE
 
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

#define TRANSFER_LENGTH 4096

int main(int argc, char **argv)
{
  struct sockaddr_in stSockAddr;
  int SocketFD = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
  int out_fd;
  ssize_t sent;
  unsigned long total_sent;
  int pipe_fds[2];

  pipe_fds[0] = 0;
  pipe_fds[1] = 0;

  if (argc != 3) {
    printf("Usage: %s <port> <file> \n", argv[0]);
    printf("\n");
    printf("\t<port> -- TCP port number to listen on\n");
    printf("\t<file> -- output file to send data to\n");
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
      
      if ((out_fd = open(argv[2], O_WRONLY | O_CREAT | O_TRUNC)) <= 0) {
	printf("[server] Can not open %s\n", argv[2]);
      }

      if (out_fd > 0) {
	if (pipe(pipe_fds) < 0) {
	  perror("[server] Can not create a pipe\n");
	}
      }

      total_sent = 0;
      do {
	if (pipe_fds[1] > 0) {
	  if ((sent = splice(ConnectFD,
			     NULL,
			     pipe_fds[1],
			     NULL,
			     TRANSFER_LENGTH,
			     SPLICE_F_MOVE)) < 0) {
	    perror("[server] Error: unable to splice data from the socket");
	  }
	}

	if (sent > 0 && pipe_fds[0] > 0) {
	  if ((sent = splice(pipe_fds[0],
			     NULL,
			     out_fd,
			     NULL,
			     sent,
			     SPLICE_F_MOVE)) < 0) {
	    perror("[server] Error: unable to splice data to the file");
	  }
	}
	if (sent > 0) {
	  total_sent += sent;
	}
      } while (sent > 0);

      if (total_sent > 0) {
	printf("[server] Transfer %ld bytes successfully\n", total_sent);
      }

      if (pipe_fds[1] != 0) {
	close(pipe_fds[1]);
      }
      if (pipe_fds[0] != 0) {
	close(pipe_fds[0]);
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
