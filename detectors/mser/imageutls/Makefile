include ../../../config

INCLUDE_PATHS = -I. -I.. $(IMAGE_LIBS_CFLAGS)

LIB = libimageutls.a 
CPP_FILES = imageutls.cpp
OBJ_FILES = $(C_FILES:.c=.o) $(CPP_FILES:.cpp=.o)

all: $(LIB)

$(LIB): $(OBJ_FILES)
	$(AR) crs $(LIB) $(OBJ_FILES)

install: all
	$(INSTALL) -d $(LIBS_LIB_DIR)
	$(CP) $(LIB) $(LIBS_LIB_DIR)/$(LIB)
	$(INSTALL) -d $(LIBS_INCLUDES)/imageutls
	$(CP) *.h $(LIBS_INCLUDES)/imageutls

GARBAGE = $(OBJ_FILES) $(LIB) core *~

.PHONY:clean
clean:
	$(RM) $(GARBAGE)
