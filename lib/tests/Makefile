############################################################################
# This file is part of LEGO-Pi.						   #
#									   #
# Copyright (Copyplease) szz-dvl.					   #
# 									   #	
#									   #	
# License								   #	
#                                                                          #
# This program is free software: you can redistribute it and/or modify     #
# it under the terms of the GNU Affero General Public License as published #
# by the Free Software Foundation, either version 3 of the License, or     #
# (at your option) any later version.                                      #
#  									   #
# This program is distributed in the hope that it will be useful,          #
# but WITHOUT ANY WARRANTY; without even the implied warranty of           #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the             #
# GNU Affero General Public License for more details at                    #
# <http://www.gnu.org/licenses/agpl-3.0-standalone.html>                   #
############################################################################

DESTDIR=./
HEADDIR=../

DEBUG	= -g -O2
CC	= gcc
INCLUDE	= -I.
CFLAGS = $(DEBUG) -Wall


KNRM = \e[0m"
KRED = "\e[31m

# Should not alter anything below this line
###############################################################################

EXE = i2c_test

OBJ = i2c_test.o

LIBS = -lrt -llegoi2c

all:	$(EXE)

.c.o:
	@$(CC) -c $(CFLAGS) $< -o $@

$(EXE):	$(OBJ)
	@/bin/echo -e $(KRED)[Making I2C test]$(KNRM) $@ 
	@$(CC) $< -o $@ $(LIBS) 

.PHONEY: clean
clean:
	@/bin/echo -e $(KRED)[Cleaning I2C test]$(KNRM)
	@rm -f $(OBJ) *~ core tags Makefile.bak

.PHONEY: uninstall
uninstall:
	@/bin/echo -e $(KRED)[UnInstall I2C test]$(KNRM)
	@rm -f $(DESTDIR)i2c_test

i2c_test.o: $(HEADDIR)lego_i2c.h
