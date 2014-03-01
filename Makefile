DYN_VERS_MAJ=1
DYN_VERS_MIN=0

VERSION=$(DYN_VERS_MAJ).$(DYN_VERS_MIN)
DESTDIR=/usr
PREFIX=/local

STATIC=liblego.a
DYNAMIC=liblego.so.$(VERSION)

DEBUG	= -g -O2
CC	= gcc
INCLUDE	= -I.
CFLAGS	= $(DEBUG) -Wall $(INCLUDE) -Winline -pipe -fpic

MAKE = make

KNRM = \e[0m"
KRED = "\e[31m


# Should not alter anything below this line
###############################################################################

SRC =  lego_motor.c lego_analog.c lego.c

OBJ	= 	$(SRC:.c=.o)

all:	$(DYNAMIC)

static:	$(STATIC)

$(STATIC):	$(OBJ)
	@/bin/echo -e $(KRED)[Link (Static)]$(KNRM)
	@ar rcs $(STATIC) $(OBJ)
	@ranlib $(STATIC)


$(DYNAMIC):	$(OBJ)
	@/bin/echo -e $(KRED)[Link (Dynamic)]$(KNRM)
	@$(CC) -shared -Wl,-soname,liblego.so.1 -o liblego.so.1.0 -lpthread $(OBJ)

.c.o:
	@/bin/echo -e $(KRED)[Compile]$(KNRM) $<
	@$(CC) -c $(CFLAGS) $< -o $@


test:	
	@/bin/echo -e $(KRED)[Making Tests]$(KNRM)
	@$(MAKE) -C tests
	@$(MAKE) -C tests MAKEFLAGS= install
	@$(MAKE) -C tests MAKEFLAGS= clean

test_uninstall:
	@$(MAKE) -C tests MAKEFLAGS= uninstall

.PHONEY: clean
clean:
	@/bin/echo -e $(KRED)[Cleaning]$(KNRM)
	@rm -f $(OBJ) *~ core tags Makefile.bak liblego.*

.PHONEY: tags
tags: $(SRC)
	@/bin/echo -e $(KRED)[ctags]$(KNRM)
	@ctags $(SRC)

.PHONEY: install
install: $(DYNAMIC)
	@/bin/echo -e $(KRED)[Install]$(KNRM)
	@install -m 0755 -d $(DESTDIR)$(PREFIX)/lib
	@install -m 0755 -d $(DESTDIR)$(PREFIX)/include
	@install -m 0644 lego.h $(DESTDIR)$(PREFIX)/include
	@install -m 0644 lego_motor.h $(DESTDIR)$(PREFIX)/include
	@install -m 0644 lego_analog.h $(DESTDIR)$(PREFIX)/include
	@install -m 0644 lego_shared.h $(DESTDIR)$(PREFIX)/include
	@install -m 0755 liblego.so.$(VERSION)	$(DESTDIR)$(PREFIX)/lib
	@ln -sf $(DESTDIR)$(PREFIX)/lib/liblego.so.$(VERSION) $(DESTDIR)/lib/liblego.so
	@ln -sf $(DESTDIR)$(PREFIX)/lib/liblego.so.$(VERSION) $(DESTDIR)/lib/liblego.so.1
	@ldconfig

.PHONEY: install-static
install-static: $(STATIC)
	@/bin/echo -e $(KRED)[Install Static]$(KNRM)
	@install -m 0755 liblego.a $(DESTDIR)$(PREFIX)/lib

.PHONEY: uninstall
uninstall:
	@/bin/echo -e $(KRED)[UnInstall]$(KNRM)
	@rm -f $(DESTDIR)$(PREFIX)/include/lego_motor.h
	@rm -f $(DESTDIR)$(PREFIX)/include/lego_shared.h
	@rm -f $(DESTDIR)$(PREFIX)/include/lego_analog.h
	@rm -f $(DESTDIR)$(PREFIX)/include/lego.h	
	@rm -f $(DESTDIR)$(PREFIX)/lib/liblego.*
	@$(MAKE) -C tests MAKEFLAGS= uninstall
	@ldconfig


lego_motor.o: lego_shared.h lego_motor.h
lego_analog.o: lego_shared.h lego_analog.h
lego.o: lego.h
