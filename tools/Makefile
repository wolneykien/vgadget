
all: tcpserver tcpclient splice

tcpserver: tcpserver.c
	$(CROSS_COMPILE)gcc -o tcpserver tcpserver.c

tcpclient: tcpclient.c
	$(CROSS_COMPILE)gcc -o tcpclient tcpclient.c

splice: splice.c
	$(CROSS_COMPILE)gcc -o splice splice.c

clean:
	[ ! -f tcpserver ] || rm tcpserver
	[ ! -f tcpclient ] || rm tcpclient
	[ ! -f splice ] || rm splice

install:
	install -D -m0755 tcpserver $(PREFIX)/bin/tcpserver
	install -D -m0755 tcpclient $(PREFIX)/bin/tcpclient
	install -D -m0755 splice $(PREFIX)/bin/splice
