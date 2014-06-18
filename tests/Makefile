#;
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

DEBUG	= -g -O2
CC	= gcc
INCLUDE	= -I.
CFLAGS = $(DEBUG) -Wall
LIBS = -llego -lgsl -lgslcblas -lwiringPi
#-lwiringPi -lrt -llego -lgsl -lgslcblas


KNRM = \e[0m"
KRED = "\e[31m

# Should not alter anything below this line
###############################################################################

EXE = dg_test ag_test mt_test

all: $(EXE)


ag_test:
	@/bin/echo -e $(KRED)[Making test]$(KNRM) $@
	@$(CC) $(CFLAGS) analog_test.c $(LIBS) -o $@

mt_test:
	@/bin/echo -e $(KRED)[Making test]$(KNRM) $@
	@$(CC) $(CFLAGS) motor_test.c $(LIBS) -o $@

dg_test:
	@/bin/echo -e $(KRED)[Making test]$(KNRM) $@
	@$(CC) $(CFLAGS) digital_test.c $(LIBS) -o $@


.PHONEY: clean
clean:
	@/bin/echo -e $(KRED)[Cleaning Tests]$(KNRM)
	@rm -f *.o *~ core tags Makefile.bak

.PHONEY: uninstall
uninstall:
	@/bin/echo -e $(KRED)[UnInstall Tests]$(KNRM)
	@rm -f $(DESTDIR)/mt_test
	@rm -f $(DESTDIR)/dg_test
	@rm -f $(DESTDIR)/ag_test

