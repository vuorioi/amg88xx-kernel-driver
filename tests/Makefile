BIN = userspace
OBJ = userspace.o
CC = gcc

all: $(BIN)

%.o: %.c
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@ -Wno-deprecated-declarations

$(BIN): $(OBJ)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

clean:
	rm -f *.o $(BIN)
