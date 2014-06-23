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

EXE = motion laber follow square

all: $(EXE)


motion:
	@/bin/echo -e $(KRED)[Making demo]$(KNRM) $@
	@$(CC) $(CFLAGS) motion.c $(LIBS) -o $@

laber:
	@/bin/echo -e $(KRED)[Making demo]$(KNRM) $@
	@$(CC) $(CFLAGS) laber.c $(LIBS) -o $@

follow:
	@/bin/echo -e $(KRED)[Making demo]$(KNRM) $@
	@$(CC) $(CFLAGS) follow.c $(LIBS) -o $@

square:
	@/bin/echo -e $(KRED)[Making demo]$(KNRM) $@
	@$(CC) $(CFLAGS) square.c $(LIBS) -o $@

.PHONEY: clean
clean:
	@/bin/echo -e $(KRED)[Cleaning demo tests]$(KNRM)
	@rm -f *.o *~ core tags Makefile.bak

.PHONEY: uninstall
uninstall:
	@/bin/echo -e $(KRED)[UnInstall demo tests]$(KNRM)
	@rm -f $(DESTDIR)/laber
	@rm -f $(DESTDIR)/motion
	@rm -f $(DESTDIR)/follow
	@rm -f $(DESTDIR)/square
