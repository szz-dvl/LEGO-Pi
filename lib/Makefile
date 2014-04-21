DYN_VERS_MAJ=1
DYN_VERS_MIN=0

VERSION=$(DYN_VERS_MAJ).$(DYN_VERS_MIN)
DESTDIR=/usr
PREFIX=/local
LEGO_DIR=/lego

STATIC=liblegoi2c.a
DYNAMIC=liblegoi2c.so.$(VERSION)

DEBUG	= -g -O2
CC	= gcc
INCLUDE	= -I.
CFLAGS	= $(DEBUG) -Wall $(INCLUDE) -Winline -pipe -fpic

MAKE = make

KNRM = \e[0m"
KRED = "\e[31m


# Should not alter anything below this line
###############################################################################

SRC =  lego_i2c.c

OBJ	= 	$(SRC:.c=.o)

all:	$(DYNAMIC)

static:	$(STATIC)

$(STATIC):	$(OBJ)
	@/bin/echo -e $(KRED)[I2C Link (Static)]$(KNRM)
	@ar rcs $(STATIC) $(OBJ)
	@ranlib $(STATIC)


$(DYNAMIC):	$(OBJ)
	@/bin/echo -e $(KRED)[I2C Link (Dynamic)]$(KNRM)
	@$(CC) -shared -Wl,-soname,liblegoi2c.so.1 -o liblegoi2c.so.1.0 $(OBJ)

.c.o:
	@/bin/echo -e $(KRED)[Compile I2C]$(KNRM) $<
	@$(CC) -c $(CFLAGS) $< -o $@

.PHONEY: clean
clean:
	@/bin/echo -e $(KRED)[Cleaning I2C]$(KNRM)
	@rm -f $(OBJ) *~ core tags Makefile.bak liblegoi2c.*

.PHONEY: tags
tags: $(SRC)
	@/bin/echo -e $(KRED)[ctags]$(KNRM)
	@ctags $(SRC)

.PHONEY: install
install: $(DYNAMIC)
	@/bin/echo -e $(KRED)[Install I2C]$(KNRM)
	@install -m 0755 -d $(DESTDIR)$(PREFIX)/lib
	@install -m 0755 -d $(DESTDIR)$(PREFIX)/include
	@install -m 0755 -d $(DESTDIR)$(PREFIX)/include$(LEGO_DIR)
	@install -m 0644 lego_i2c.h $(DESTDIR)$(PREFIX)/include$(LEGO_DIR)
	@install -m 0755 liblegoi2c.so.$(VERSION) $(DESTDIR)$(PREFIX)/lib
	@ln -sf $(DESTDIR)$(PREFIX)/lib/liblegoi2c.so.$(VERSION) $(DESTDIR)/lib/liblegoi2c.so
	@ln -sf $(DESTDIR)$(PREFIX)/lib/liblegoi2c.so.$(VERSION) $(DESTDIR)/lib/liblegoi2c.so.1
	@ldconfig

.PHONEY: install-static
install-static: $(STATIC)
	@/bin/echo -e $(KRED)[Install Static I2C]$(KNRM)
	@install -m 0755 liblegoi2c.a $(DESTDIR)$(PREFIX)/lib

.PHONEY: uninstall
uninstall:
	@/bin/echo -e $(KRED)[UnInstall I2C]$(KNRM)
	@rm -f $(DESTDIR)$(PREFIX)/include$(LEGO_DIR)/lego_i2c.h
	@rm -f $(DESTDIR)$(PREFIX)/lib/liblegoi2c.*
	@ldconfig


lego_i2c.o: lego_i2c.h

