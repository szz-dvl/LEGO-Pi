DESTDIR=./

DEBUG	= -g -O2
CC	= gcc
INCLUDE	= -I.
CFLAGS = $(DEBUG) -Wall
LIBS = -lwiringPi -lrt -llego -lgsl -lgslcblas


KNRM = \e[0m"
KRED = "\e[31m

# Should not alter anything below this line
###############################################################################

EXE = interpol digitst legotst

all: $(EXE)


legotst:
	@/bin/echo -e $(KRED)[Making test]$(KNRM) $@
	@$(CC) $(CFLAGS) lego_test.c $(LIBS) -o lego_test

interpol:
	@/bin/echo -e $(KRED)[Making test]$(KNRM) $@
	@$(CC) $(CFLAGS) interpolation.c $(LIBS) -o interpol

digitst:
	@/bin/echo -e $(KRED)[Making test]$(KNRM) $@
	@$(CC) $(CFLAGS) digital_test.c $(LIBS) -o digi_test

.PHONEY: clean
clean:
	@/bin/echo -e $(KRED)[Cleaning Tests]$(KNRM)
	@rm -f *.o *~ core tags Makefile.bak

.PHONEY: uninstall
uninstall:
	@/bin/echo -e $(KRED)[UnInstall Tests]$(KNRM)
	@rm -f $(DESTDIR)/lego_test
	@rm -f $(DESTDIR)/interpol
	@rm -f $(DESTDIR)/digi_test



