INCLUDE_ASM_DIR=/usr/include

all: tcpserver tcpclient splice

tcpserver: tcpserver.c
	$(CROSS_COMPILE)gcc -o tcpserver -I$(INCLUDE_ASM_DIR) tcpserver.c

tcpclient: tcpclient.c
	$(CROSS_COMPILE)gcc -o tcpclient -I$(INCLUDE_ASM_DIR) tcpclient.c

splice: splice.c
	$(CROSS_COMPILE)gcc -o splice -I$(INCLUDE_ASM_DIR) splice.c

clean:
	[ ! -f tcpserver ] || rm tcpserver
	[ ! -f tcpclient ] || rm tcpclient
	[ ! -f splice ] || rm splice

install: all
	install -D -m0755 tcpserver $(PREFIX)/bin/tcpserver
	install -D -m0755 tcpclient $(PREFIX)/bin/tcpclient
	install -D -m0755 splice $(PREFIX)/bin/splice
