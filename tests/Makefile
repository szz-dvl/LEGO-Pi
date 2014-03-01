DESTDIR=../


DEBUG	= -g -O2
CC	= gcc
INCLUDE	= -I.
CFLAGS = $(DEBUG) -Wall
LIBS = -lwiringPi -lrt -llego -lgsl -lgslcblas
TARGET = test install

KNRM = \e[0m"
KRED = "\e[31m

# Should not alter anything below this line
###############################################################################

EXE = lego_test interpolation

SRC = lego_test.c interpolation.c

OBJ = $(SRC:.c=.o)

all:	$(EXE)

%.o: %.c 
	 @$(CC) $(CFLAGS) -c $< -o $@

$(EXE): $(OBJ)
	@/bin/echo -e $(KRED)[Making test]$(KNRM) $@ 
	@$(CC) $< -o $@ $(LIBS)

.PHONEY: clean
clean:
	@/bin/echo -e $(KRED)[Cleaning Tests]$(KNRM)
	@rm -f $(OBJ) *~ core tags Makefile.bak lego_test interpolation

.PHONEY: install
install: $(TEST)
	@/bin/echo -e $(KRED)[Install Tests]$(KNRM)
	@install -m 0755 lego_test $(DESTDIR)
	@install -m 0755 interpolation $(DESTDIR)

.PHONEY: uninstall
uninstall:
	@/bin/echo -e $(KRED)[UnInstall Tests]$(KNRM)
	@rm -f $(DESTDIR)/lego_test
	@rm -f $(DESTDIR)/interpolation

lego_test.o: $(DESTDIR)lego.h 

