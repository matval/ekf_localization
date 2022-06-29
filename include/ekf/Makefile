CPP_FILES := $(wildcard *.cpp)
TMPVAR := $(CPP_FILES)
VAR = $(filter-out $(CPP_FILTER), $(TMPVAR))
OBJ_DIR=../obj
OBJ_FILES := $(addprefix $(OBJ_DIR)/,$(notdir $(VAR:.cpp=.o)))
CFLAGS= -c -w -pthread -lrt -O2 -std=c++11

default: $(OBJ_FILES)

$(OBJ_DIR)/%.o: %.cpp
	$(CC) $(CFLAGS) $(INCLUDES) -c -o $@ $<
