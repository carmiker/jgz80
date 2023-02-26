bin = z80_tests
src = z80.c z80_tests.c
obj = z80.o z80_tests.o
CFLAGS = -g -Wall -Wextra -O2 -std=c11 -pedantic
LDFLAGS =

.PHONY: all clean

all: $(bin)

$(bin): $(obj)
	$(CC) -o $@ $^ $(LDFLAGS)

clean:
	-rm $(bin) $(obj)
