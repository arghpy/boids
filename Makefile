CFLAGS         := -Wall -Wextra -Wpedantic

INCLUDE_DIR    := ./thirdparty

RAYLIB         := $(INCLUDE_DIR)/raylib-5.5_linux_amd64
RAYLIB_INCLUDE := $(RAYLIB)/include
RAYLIB_LIB     := $(RAYLIB)/lib
RAYLIB_RPATH   := -Wl,-rpath=$(RAYLIB_LIB)
RAYLIB_LINK    := raylib

UTILS_INCLUDE := $(INCLUDE_DIR)/utils/c

INCLUDES      := -I$(RAYLIB_INCLUDE) -I$(UTILS_INCLUDE)
LIBS          := -L$(RAYLIB_LIB) -l$(RAYLIB_LINK) -lm $(RAYLIB_RPATH)

TARGET        := boids

# default action. Builds target
all: $(TARGET)

$(TARGET): $(TARGET).c
	@echo "Compiling $^..."
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $^ $(LIBS)

clean:
	@echo "Cleaning..."
	rm --force $(TARGET)

# Builds if needed
run: $(TARGET)
	./$(TARGET)

bear:
	@echo "Creating compile_commands.json..."
	bear -- $(CC) $(CFLAGS) $(INCLUDES) -o $(TARGET) $(TARGET).c $(LIBS)

.PHONY: all clean run bear
