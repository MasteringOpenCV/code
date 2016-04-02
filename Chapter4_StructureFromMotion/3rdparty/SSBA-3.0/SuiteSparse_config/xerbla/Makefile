# Makefile for null-output xerbla

default: ccode

include ../SuiteSparse_config.mk

ccode: libcerbla.a

fortran: libxerbla.a 

all: libxerbla.a libcerbla.a

# Fortran version:
libxerbla.a: xerbla.f
	$(F77) $(F77FLAGS) -c xerbla.f
	$(ARCHIVE)  libxerbla.a xerbla.o
	- $(RANLIB) libxerbla.a
	- $(RM) xerbla.o

# C version:
libcerbla.a: xerbla.c xerbla.h
	$(CC) $(CF) -c xerbla.c
	$(ARCHIVE)  libcerbla.a xerbla.o
	- $(RANLIB) libcerbla.a
	- $(RM) xerbla.o

distclean: purge

purge: clean
	- $(RM) *.o *.a

clean:
	- $(RM) $(CLEAN)
