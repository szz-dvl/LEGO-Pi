#;
############################################################################
# This file is part of LEGO-Pi.						   
#									   
# Copyright (Copyplease) szz-dvl.					   
# 									   	
#									   	
# License								   	
#                                                                          
# This program is free software: you can redistribute it and/or modify     
# it under the terms of the GNU Affero General Public License as published 
# by the Free Software Foundation, either version 3 of the License, or     
# (at your option) any later version.                                      
#  									   
# This program is distributed in the hope that it will be useful,          
# but WITHOUT ANY WARRANTY; without even the implied warranty of           
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the             
# GNU Affero General Public License for more details at                    
# <http://www.gnu.org/licenses/agpl-3.0-standalone.html>                   
############################################################################

DYN_VERS_MAJ=1
DYN_VERS_MIN=0

VERSION=$(DYN_VERS_MAJ).$(DYN_VERS_MIN)
DESTDIR=/usr
PREFIX=/local
LEGO_DIR=/lego

STATIC=liblego.a
DYNAMIC=liblego.so.$(VERSION)

DEBUG	= -g -O2
CC	= gcc
INCLUDE	= -I.
CFLAGS	= $(DEBUG) -Wall $(INCLUDE) -Winline -pipe -fpic

LIBS = -lpthread -llegoi2c -lrt

MAKE = make

KNRM = \e[0m"
KRED = "\e[31m


# Should not alter anything below this line
###############################################################################

SRC =  lego_motor.c lego_analog.c lego_digital.c lego.c

OBJ	= 	$(SRC:.c=.o)

all:	$(DYNAMIC)

static:	$(STATIC)

$(STATIC):	$(OBJ)
	@/bin/echo -e $(KRED)[Link (Static)]$(KNRM)
	@ar rcs $(STATIC) $(OBJ)
	@ranlib $(STATIC)


$(DYNAMIC):	$(OBJ)
	@/bin/echo -e $(KRED)[Link (Dynamic)]$(KNRM)
	@$(CC) -shared -Wl,-soname,liblego.so.1 -o liblego.so.1.0 $(LIBS) $(OBJ)

.c.o:
	@/bin/echo -e $(KRED)[Compile]$(KNRM) $<
	@$(CC) -c $(CFLAGS) $< -o $@


i2c:	
	@/bin/echo -e $(KRED)[Making I2C lib]$(KNRM)
	@$(MAKE) -C lib 
	@$(MAKE) -C lib MAKEFLAGS= install
	@$(MAKE) -C lib MAKEFLAGS= clean

test:	
	@/bin/echo -e $(KRED)[Making Tests]$(KNRM)
	@$(MAKE) -C tests
	@$(MAKE) -C tests MAKEFLAGS= clean
	@$(MAKE) -C lib/tests
	@$(MAKE) -C lib/tests MAKEFLAGS= clean

test_uninstall:
	@$(MAKE) -C tests MAKEFLAGS= uninstall
	@$(MAKE) -C lib/tests MAKEFLAGS= uninstall

i2c_uninstall:
	@$(MAKE) -C lib MAKEFLAGS= uninstall

.PHONEY: clean
clean:
	@/bin/echo -e $(KRED)[Cleaning]$(KNRM)
	@rm -f $(OBJ) *~ core tags Makefile.bak liblego.*
	@$(MAKE) -C tests MAKEFLAGS= clean
	@$(MAKE) -C lib MAKEFLAGS= clean
	@$(MAKE) -C lib/tests MAKEFLAGS= clean


.PHONEY: tags
tags: $(SRC)
	@/bin/echo -e $(KRED)[ctags]$(KNRM)
	@ctags $(SRC)

.PHONEY: install
install: $(DYNAMIC)
	@/bin/echo -e $(KRED)[Install]$(KNRM)
	@install -m 0755 -d $(DESTDIR)$(PREFIX)/lib
	@install -m 0755 -d $(DESTDIR)$(PREFIX)/include
	@install -m 0755 -d $(DESTDIR)$(PREFIX)/include$(LEGO_DIR)
	@install -m 0644 lego.h $(DESTDIR)$(PREFIX)/include
	@install -m 0644 lego_motor.h $(DESTDIR)$(PREFIX)/include$(LEGO_DIR)
	@install -m 0644 lego_analog.h $(DESTDIR)$(PREFIX)/include$(LEGO_DIR)
	@install -m 0644 lego_digital.h $(DESTDIR)$(PREFIX)/include$(LEGO_DIR)
	@install -m 0644 lego_shared.h $(DESTDIR)$(PREFIX)/include$(LEGO_DIR)
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
	@rm -f $(DESTDIR)$(PREFIX)/include$(LEGO_DIR)/lego_motor.h
	@rm -f $(DESTDIR)$(PREFIX)/include$(LEGO_DIR)/lego_shared.h
	@rm -f $(DESTDIR)$(PREFIX)/include$(LEGO_DIR)/lego_analog.h
	@rm -f $(DESTDIR)$(PREFIX)/include$(LEGO_DIR)/lego_digital.h
	@rm -f $(DESTDIR)$(PREFIX)/include/lego.h	
	@rm -f $(DESTDIR)$(PREFIX)/lib/liblego.*
	@rm -f $(DESTDIR)/lib/liblego.*
	@$(MAKE) -C tests MAKEFLAGS= uninstall
	@$(MAKE) -C lib MAKEFLAGS= uninstall
	@$(MAKE) -C lib/tests MAKEFLAGS= uninstall
	@rm -rf $(DESTDIR)$(PREFIX)/include$(LEGO_DIR)
	@ldconfig


lego_motor.o: lego_shared.h lego_motor.h
lego_analog.o: lego_shared.h lego_analog.h
lego_digital.o: lego_shared.h lego_digital.h
lego.o: lego.h
