
all: tcpserver tcpclient

tcpserver: tcpserver.c
	$(CROSS_COMPILE)gcc -o tcpserver tcpserver.c

tcpclient: tcpclient.c
	$(CROSS_COMPILE)gcc -o tcpclient tcpclient.c

clean:
	[ ! -f tcpserver ] || rm tcpserver
	[ ! -f tcpclient ] || rm tcpclient

install:
	install -D -m0755 tcpserver $(PREFIX)/bin/tcpserver
	install -D -m0755 tcpclient $(PREFIX)/bin/tcpclient
