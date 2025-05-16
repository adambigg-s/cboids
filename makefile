CC = clang
SRC_DIR = src
OBJ_DIR = obj
BIN_DIR = bin
SRC = $(wildcard $(SRC_DIR)/*.cpp)
OBJ = $(patsubst $(SRC_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(SRC))
OUT = $(BIN_DIR)/boids.exe
CFLAGS = -Wall

.PHONY: all
all: $(OUT)

$(OUT): $(OBJ) | $(BIN_DIR)
	$(CC) $(OBJ) -o $@ $(CFLAGS)

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp | $(OBJ_DIR)
	$(CC) -c $< -o $@ $(CFLAGS)

$(BIN_DIR):
	if not exist $(BIN_DIR) mkdir $(BIN_DIR)

$(OBJ_DIR):
	if not exist $(OBJ_DIR) mkdir $(OBJ_DIR)

.PHONY: clean
clean:
	if exist $(OBJ_DIR) rmdir /s /q $(OBJ_DIR)
	if exist $(BIN_DIR) rmdir /s /q $(BIN_DIR)

.PHONY: run
run: all
	$(OUT)
