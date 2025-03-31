# Compiler
CC = gcc

# Compiler flags: adding -D_POSIX_C_SOURCE=199309L ensures that CLOCK_MONOTONIC is defined.
CFLAGS = -Wall -Wextra -std=c11 -I. -D_POSIX_C_SOURCE=199309L

# Linker flags (if needed, e.g., -lrt for older systems)
LDFLAGS = -lrt

# Source files and corresponding object files
SOURCES = read_uid.c pn532.c uart.c
OBJECTS = $(SOURCES:.c=.o)

# Final executable target
TARGET = read_uid

# Default rule
all: $(TARGET)

# Link object files to create the final executable
$(TARGET): $(OBJECTS)
	$(CC) $(LDFLAGS) -o $(TARGET) $(OBJECTS)

# Compile C files to object files
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

# Clean up build files
clean:
	rm -f $(OBJECTS) $(TARGET)
