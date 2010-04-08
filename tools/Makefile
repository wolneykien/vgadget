
all: tcpserver

tcpserver: tcpserver.c
	$(CROSS_COMPILE)gcc -o tcpserver tcpserver.c
clean:
	[ ! -f tcpserver ] || rm tcpserver

install:
	install -D -m0755 tcpserver $(PREFIX)/bin/tcpserver
