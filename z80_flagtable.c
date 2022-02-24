#include "z80.c"
#define GET_BIT(n, val) (((val) >> (n)) & 1)

#include <stdio.h>

int main() {
	int result = 0, f;
	for (; result < 256; ++result) {
		f = 0 |
		flag_val(sf, result >> 7) |
		flag_val(zf, result == 0) |
		flag_val(pf, parity(result)) |
		flag_val(xf, GET_BIT(3, result)) |
		flag_val(yf, GET_BIT(5, result));
		if(result % 16 == 0)
			printf("\n    ");
		else
			printf(" ");
		printf("0x%02x,", f);

	}
	printf("\n");
}
