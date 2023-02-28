CC ?= cc
bin := z80_tests
src := z80.c z80_tests.c
obj := z80.o z80_tests.o
MAINT_FLAGS := -std=c11 -Wall -Wextra -pedantic
CFLAGS ?= -g -O2

.PHONY: all clean

all: $(bin)

%.o: %.c
	$(CC) $(CFLAGS) $(MAINT_FLAGS) $(CPPFLAGS) -c $< -o $@

$(bin): $(obj)
	$(CC) -o $@ $^ $(LDFLAGS)

clean:
	-rm $(bin) $(obj)
