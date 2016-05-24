include ../config

INCLUDE_PATHS = -I. -I../LL -I../utls

GM_LIBS = -lLL
LIB = libOptionGM.a 

C_FILES = optionDouble.c optionList.c option.c optionInt.c optionMisc.c

OBJ_FILES = $(C_FILES:.c=.o)

all: $(LIB)

$(LIB): $(OBJ_FILES)
	$(AR) crs $(LIB) $(OBJ_FILES)

GARBAGE = $(OBJ_FILES) $(LIB) core *~

.PHONY:clean
clean:
	$(RM) $(GARBAGE)
