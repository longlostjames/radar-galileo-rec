# Makefile for Radar Time Series Package (RTS)
# Chris Walden 30/04/2010

# This is the base path for the package
ROOTPATH=.

# This is where the bits and pieces are kept
SRCDIR = $(ROOTPATH)/src
LIBDIR = $(ROOTPATH)/lib
BINDIR = $(ROOTPATH)/bin
INCDIR = $(ROOTPATH)/include

CC     = gcc
CFLAGS = -Wall -O3 -ffast-math -I$(INCDIR) -I$(ROOTPATH)/../RSP/include \
			-I$(ROOTPATH)/../include -D_FILE_OFFSET_BITS=64
LIBS   = -lm

# The master header file
INC = $(INCDIR)/RTS.h

# Top level rule
all : $(LIBDIR)/librts.a

# The main library
$(LIBDIR)/librts.a : $(BINDIR)/RTS.o
	ar r $@ $(BINDIR)/RTS.o

$(BINDIR)/RTS.o : $(SRCDIR)/RTS.c $(INC)
	$(CC) $(CFLAGS) -o $@ -c $(SRCDIR)/RTS.c

clean :
	$(RM) $(BINDIR)/*.[doa]
	$(RM) $(LIBDIR)/*.[doa]
