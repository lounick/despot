# -----------------------
# Compiler/linker options
# -----------------------

CXX = g++
LDFLAGS = -O3 -Wall -Wno-sign-compare $(GPROF)

# -----------
# Directories
# -----------

DESPOTHOME = ../../..
SRCDIR = src
INCDIR = $(DESPOTHOME)/include
LIBDIR = $(DESPOTHOME)/build
INCLUDE = -I $(INCDIR) -I $(SRCDIR)

# -----
# Files
# -----

CM_SOURCES = $(shell find -L $(SRCDIR)/base $(SRCDIR)/cooperative_minefield -name '*.cpp')
CM_BIN = cooperative_minefield


# -------
# Targets
# -------

.PHONY: all clean

all: CM_BIN

CM_BIN:
	@cd $(DESPOTHOME) && make
	$(CXX) $(LDFLAGS) $(CM_SOURCES) $(INCLUDE) $(LIBDIR)/*.o -o $(CM_BIN)


CM_BIN_LIGHT:
	@cd $(DESPOTHOME) && make
	$(CXX) $(LDFLAGS) $(CM_SOURCES) $(INCLUDE) -L $(LIBDIR) -ldespot -o $(CM_BIN)


clean:
	rm -f $(RS_BIN) $(FVRS_BIN)
