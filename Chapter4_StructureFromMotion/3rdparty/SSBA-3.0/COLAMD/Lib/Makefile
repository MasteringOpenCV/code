#-------------------------------------------------------------------------------
# COLAMD Makefile
#-------------------------------------------------------------------------------

default: libcolamd.a

include ../../SuiteSparse_config/SuiteSparse_config.mk

I = -I../Include -I../../SuiteSparse_config

INC = ../Include/colamd.h ../../SuiteSparse_config/SuiteSparse_config.h

SRC = ../Source/colamd.c ../Source/colamd_global.c

# creates libcolamd.a, a C-callable COLAMD library
libcolamd.a:  $(SRC) $(INC)
	$(CC) $(CF) $(I) -c ../Source/colamd_global.c
	$(CC) $(CF) $(I) -c ../Source/colamd.c
	$(CC) $(CF) $(I) -c ../Source/colamd.c -DDLONG -o colamd_l.o
	$(ARCHIVE)  libcolamd.a colamd.o colamd_l.o colamd_global.o
	- $(RANLIB) libcolamd.a

ccode: libcolamd.a

library: libcolamd.a

clean:
	- $(RM) $(CLEAN)

purge: distclean

distclean: clean
	- $(RM) libcolamd.a
