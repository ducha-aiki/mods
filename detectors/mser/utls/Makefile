include ../config

INCLUDE_PATHS = -I.

LIB = libutls.a 
C_FILES = timeutls.c
CPP_FILES = matrix.cpp
OBJ_FILES = $(C_FILES:.c=.o) $(CPP_FILES:.cpp=.o)

all: $(LIB)

$(LIB): $(OBJ_FILES)
	$(AR) crs $(LIB) $(OBJ_FILES)

install: all
	$(INSTALL) -d $(LIBS_LIB_DIR)
	$(CP) $(LIB) $(LIBS_LIB_DIR)/$(LIB)

GARBAGE = $(OBJ_FILES) $(LIB) core *~

.PHONY:clean
clean:
	$(RM) $(GARBAGE)
