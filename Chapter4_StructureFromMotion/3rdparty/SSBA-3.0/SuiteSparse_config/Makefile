#-------------------------------------------------------------------------------
# SuiteSparse_config Makefile
#-------------------------------------------------------------------------------

VERSION = 4.0.0

default: ccode

include SuiteSparse_config.mk

ccode: libsuitesparseconfig.a

all: libsuitesparseconfig.a

library: libsuitesparseconfig.a

libsuitesparseconfig.a: SuiteSparse_config.c SuiteSparse_config.h
	$(CC) $(CF) -c SuiteSparse_config.c
	$(ARCHIVE) libsuitesparseconfig.a SuiteSparse_config.o
	$(RANLIB) libsuitesparseconfig.a
	- $(RM) SuiteSparse_config.o

distclean: purge

purge: clean
	- $(RM) *.o *.a

clean:
	- $(RM) -r $(CLEAN)

# install SuiteSparse_config
install:
	$(CP) libsuitesparseconfig.a $(INSTALL_LIB)/libsuitesparseconfig.$(VERSION).a
	( cd $(INSTALL_LIB) ; ln -sf libsuitesparseconfig.$(VERSION).a libsuitesparseconfig.a )
	$(CP) SuiteSparse_config.h $(INSTALL_INCLUDE)
	chmod 644 $(INSTALL_LIB)/libsuitesparseconfig*.a
	chmod 644 $(INSTALL_INCLUDE)/SuiteSparse_config.h

# uninstall SuiteSparse_config
uninstall:
	$(RM) $(INSTALL_LIB)/libsuitesparseconfig*.a
	$(RM) $(INSTALL_INCLUDE)/SuiteSparse_config.h

