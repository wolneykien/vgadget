/* Server code in C */

#define _GNU_SOURCE
 
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "syscalls.h"

#define TRANSFER_LENGTH 4096

int main(int argc, char **argv)
{
  int in_fd, out_fd;
  ssize_t sent, first_sent;
  unsigned long total, total_sent, all_sent;
  int pipe_fds[2];
  int cycles;
  int maxcycles;

  struct timeval start_t;
  struct timeval end_t;

  pipe_fds[0] = 0;
  pipe_fds[1] = 0;

  if (argc < 4) {
    printf("Usage: %s <from-file> <to-file> <count> [<cycles>]\n", argv[0]);
    printf("\n");
    printf("\t<from-file> -- input file to receive data from\n");
    printf("\t<to-file> -- output file to send data to\n");
    printf("\t<count> -- number of bytes to transfer\n");
    printf("\t<cycles> -- number of times to send the file\n");
    printf("\n");
    exit(255);
  }

  if ((out_fd = open(argv[2], O_WRONLY | O_CREAT | O_TRUNC)) <= 0) {
    printf("Can not open %s for writing\n", argv[2]);
  }

  if (argc = 5) {
    maxcycles = atoi(argv[4]);
  } else {
    maxcycles = 1;
  }

  cycles = 0;
  all_sent = 0;
  total = atol(argv[3]);
  first_sent = 0;
  
  if (out_fd > 0) {
    if (pipe(pipe_fds) < 0) {
      perror("Can not create a pipe\n");
    }
  }

  while (cycles < maxcycles) {
    if ((in_fd = open(argv[1], O_RDONLY)) <= 0) {
      printf("Can not open %s for reading\n", argv[1]);
    }

    total_sent = 0;
    do {
      unsigned long len = total - total_sent;
      if (len > TRANSFER_LENGTH) {
	len = TRANSFER_LENGTH;
      }
      if (pipe_fds[1] > 0 && in_fd > 0) {
	if ((sent = splice(in_fd,
			   NULL,
			   pipe_fds[1],
			   NULL,
			   len,
			   SPLICE_F_MOVE)) < 0) {
	  perror("Error: unable to splice data from the file");
	}
      }

      if (sent > 0 && pipe_fds[0] > 0) {
	if ((sent = splice(pipe_fds[0],
			   NULL,
			   out_fd,
			   NULL,
			   sent,
			   SPLICE_F_MOVE)) < 0) {
	  perror("Error: unable to splice data to the file");
	}
      }
      if (sent > 0) {
	if (first_sent == 0) {
	  first_sent = sent;
	  gettimeofday(&start_t, NULL);
	}
	total_sent += sent;
      }
    } while (sent > 0 && total_sent < total);

    all_sent += total_sent;
    cycles++;
    if (in_fd > 0) {
      close(in_fd);
    }
  }

  gettimeofday(&end_t, NULL);

  if (all_sent > 0) {
    printf("Transfer %ld bytes successfully\n", all_sent);
    if (first_sent > 0 && all_sent > first_sent) {
      double period_s =
	(end_t.tv_sec + ((double) end_t.tv_usec / 1000000)) -		\
	(start_t.tv_sec + ((double) start_t.tv_usec / 1000000));
      double speed = ((double) (all_sent - first_sent)) / period_s;
      if (speed > (1024 * 1024 * 1024)) {
	printf("Transfer speed is %.2f GB/s\n",
	       speed / (1024 * 1024 * 1024));
      } else if (speed > (1024 * 1024)) {
	printf("Transfer speed is %.2f MB/s\n", speed / (1024 * 1024));
      } else if (speed > 1024) {
	printf("Transfer speed is %.2f kB/s\n", speed / 1024);
      } else {
	printf("Transfer speed is %.2f B/s\n", speed);
      }
    } else {
      printf("Not enouph data to calculate the transfer speed\n");
    }
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

  return 0;
}
