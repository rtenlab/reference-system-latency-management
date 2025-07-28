# Makefile for dl-mutex-bench

# Compiler and flags
CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2

# Source file
SRC = dl-mutex-bench.cpp

# Output executable
EXEC = dl-mutex-bench

# Libraries (added -ldl for dynamic linker functions used by NVTX)
LIBS = -lpthread -lnvToolsExt -ldl

# NVTX include and library paths (customizable)
NVTX_INCLUDE_DIR = /usr/local/cuda/targets/sbsa-linux/include
NVTX_LIB_DIR = /usr/local/cuda/targets/sbsa-linux/lib /usr/local/cuda/lib64

# Add include paths
INCLUDES = -I$(NVTX_INCLUDE_DIR)

# Default target
all: $(EXEC)

# Build rule
$(EXEC): $(SRC)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $^ $(LIBS) $(addprefix -L, $(NVTX_LIB_DIR))

# Clean rule
clean:
	rm -f $(EXEC)

# Phony targets
.PHONY: all clean