default: lichuan_a4

prefix = /usr/local
exec_prefix = $(prefix)
bindir = $(exec_prefix)/bin
datarootdir = $(prefix)/share
mandir = $(datarootdir)/man
man1dir = $(mandir)/man1

CC = gcc
CFLAGS = -Wall -Wextra -Wpedantic -g -O
ALL_CFLAGS = -O2 -D_FORTITY_SOURCE=2 -DRTAPI -I/usr/include/linuxcnc -I/usr/include/modbus $(CFLAGS)
LDLIBS := -lmodbus -llinuxcnchal -lpthread -lm -lglib-2.0
LDFLAGS := -Wl,-z,now -Wl,-z,relro

lichuan_a4: lichuan_a4.o
	$(CC) -o $@ $^ $(LDFLAGS) $(LDLIBS)

%.o: %.c
	$(CC) $(ALL_CFLAGS) -o $@ -c $<

.PHONY: install clean distclean uninstall

install: lichuan_a4
	install -d -m 755 $(DESTDIR)$(bindir)
#	install -d -m 755 $(DESTDIR)$(man1dir)
	install lichuan_a4 $(DESTDIR)$(bindir)/
#	install -m 644 lichuan_a4.1 $(DESTDIR)$(man1dir)/

clean:
	rm -f lichuan_a4
	rm -f lichuan_a4.o
	rm -f tags

distclean: clean

uninstall:
	-rm -f $(DESTDIR)$(bindir)/lichuan_a4
	-rm -f $(DESTDIR)$(man1dir)/lichuan_a4.1

TAGS: lichuan_a4.c
	ctags $^

