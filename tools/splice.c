/* Server code in C */

#define _GNU_SOURCE
 
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define TRANSFER_LENGTH 4096

int main(int argc, char **argv)
{
  int in_fd, out_fd;
  ssize_t sent;
  unsigned long total, total_sent;
  int pipe_fds[2];

  pipe_fds[0] = 0;
  pipe_fds[1] = 0;

  if (argc != 4) {
    printf("Usage: %s <from-file> <to-file> <count>\n", argv[0]);
    printf("\n");
    printf("\t<from-file> -- input file to receive data from\n");
    printf("\t<to-file> -- output file to send data to\n");
    printf("\t<count> -- number of bytes to transfer\n");
    printf("\n");
    exit(255);
  }
 
  if ((in_fd = open(argv[1], O_RDONLY)) <= 0) {
    printf("[server] Can not open %s for reading\n", argv[1]);
  }

  if ((out_fd = open(argv[2], O_WRONLY | O_CREAT | O_TRUNC)) <= 0) {
    printf("[server] Can not open %s for writing\n", argv[2]);
  }

  if (in_fd > 0 && out_fd > 0) {
    if (pipe(pipe_fds) < 0) {
      perror("[server] Can not create a pipe\n");
    }
  }

  total = atol(argv[3]);
  total_sent = 0;
  do {
    if (pipe_fds[1] > 0) {
      if ((sent = splice(in_fd,
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
  } while (sent > 0 && total_sent < total);

  if (total_sent > 0) {
    printf("[server] Transfer %ld bytes successfully\n", total_sent);
  }

  if (pipe_fds[1] != 0) {
    close(pipe_fds[1]);
  }
  if (pipe_fds[0] != 0) {
    close(pipe_fds[0]);
  }

  if (in_fd > 0) {
    close(in_fd);
  }
  if (out_fd > 0) {
    close(out_fd);
  }

  return 0;
}
