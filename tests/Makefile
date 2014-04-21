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

EXE = interpol digitst analogtst motortst

all: $(EXE)


analogtst:
	@/bin/echo -e $(KRED)[Making test]$(KNRM) $@
	@$(CC) $(CFLAGS) analog_test.c $(LIBS) -o ag_test

motortst:
	@/bin/echo -e $(KRED)[Making test]$(KNRM) $@
	@$(CC) $(CFLAGS) motor_test.c $(LIBS) -o mt_test

interpol:
	@/bin/echo -e $(KRED)[Making test]$(KNRM) $@
	@$(CC) $(CFLAGS) interpolation.c $(LIBS) -o interpol

digitst:
	@/bin/echo -e $(KRED)[Making test]$(KNRM) $@
	@$(CC) $(CFLAGS) digital_test.c $(LIBS) -o dg_test

.PHONEY: clean
clean:
	@/bin/echo -e $(KRED)[Cleaning Tests]$(KNRM)
	@rm -f *.o *~ core tags Makefile.bak

.PHONEY: uninstall
uninstall:
	@/bin/echo -e $(KRED)[UnInstall Tests]$(KNRM)
	@rm -f $(DESTDIR)/mt_test
	@rm -f $(DESTDIR)/interpol
	@rm -f $(DESTDIR)/dg_test
	@rm -f $(DESTDIR)/ag_test


