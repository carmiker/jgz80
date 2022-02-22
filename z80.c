#include "z80.h"

// MARK: timings
static const uint8_t cyc_00[256] = {4, 10, 7, 6, 4, 4, 7, 4, 4, 11, 7, 6, 4, 4,
    7, 4, 8, 10, 7, 6, 4, 4, 7, 4, 12, 11, 7, 6, 4, 4, 7, 4, 7, 10, 16, 6, 4, 4,
    7, 4, 7, 11, 16, 6, 4, 4, 7, 4, 7, 10, 13, 6, 11, 11, 10, 4, 7, 11, 13, 6,
    4, 4, 7, 4, 4, 4, 4, 4, 4, 4, 7, 4, 4, 4, 4, 4, 4, 4, 7, 4, 4, 4, 4, 4, 4,
    4, 7, 4, 4, 4, 4, 4, 4, 4, 7, 4, 4, 4, 4, 4, 4, 4, 7, 4, 4, 4, 4, 4, 4, 4,
    7, 4, 7, 7, 7, 7, 7, 7, 4, 7, 4, 4, 4, 4, 4, 4, 7, 4, 4, 4, 4, 4, 4, 4, 7,
    4, 4, 4, 4, 4, 4, 4, 7, 4, 4, 4, 4, 4, 4, 4, 7, 4, 4, 4, 4, 4, 4, 4, 7, 4,
    4, 4, 4, 4, 4, 4, 7, 4, 4, 4, 4, 4, 4, 4, 7, 4, 4, 4, 4, 4, 4, 4, 7, 4, 4,
    4, 4, 4, 4, 4, 7, 4, 5, 10, 10, 10, 10, 11, 7, 11, 5, 10, 10, 0, 10, 17, 7,
    11, 5, 10, 10, 11, 10, 11, 7, 11, 5, 4, 10, 11, 10, 0, 7, 11, 5, 10, 10, 19,
    10, 11, 7, 11, 5, 4, 10, 4, 10, 0, 7, 11, 5, 10, 10, 4, 10, 11, 7, 11, 5, 6,
    10, 4, 10, 0, 7, 11};

static const uint8_t cyc_ed[256] = {8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
    8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
    8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 12,
    12, 15, 20, 8, 14, 8, 9, 12, 12, 15, 20, 8, 14, 8, 9, 12, 12, 15, 20, 8, 14,
    8, 9, 12, 12, 15, 20, 8, 14, 8, 9, 12, 12, 15, 20, 8, 14, 8, 18, 12, 12, 15,
    20, 8, 14, 8, 18, 12, 12, 15, 20, 8, 14, 8, 8, 12, 12, 15, 20, 8, 14, 8, 8,
    8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
    8, 8, 8, 8, 8, 8, 8, 16, 16, 16, 16, 8, 8, 8, 8, 16, 16, 16, 16, 8, 8, 8, 8,
    16, 16, 16, 16, 8, 8, 8, 8, 16, 16, 16, 16, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
    8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
    8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
    8, 8, 8, 8, 8, 8, 8};

static const uint8_t cyc_ddfd[256] = {4, 4, 4, 4, 4, 4, 4, 4, 4, 15, 4, 4, 4, 4,
    4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 15, 4, 4, 4, 4, 4, 4, 4, 14, 20, 10, 8, 8,
    11, 4, 4, 15, 20, 10, 8, 8, 11, 4, 4, 4, 4, 4, 23, 23, 19, 4, 4, 15, 4, 4,
    4, 4, 4, 4, 4, 4, 4, 4, 8, 8, 19, 4, 4, 4, 4, 4, 8, 8, 19, 4, 4, 4, 4, 4, 8,
    8, 19, 4, 4, 4, 4, 4, 8, 8, 19, 4, 8, 8, 8, 8, 8, 8, 19, 8, 8, 8, 8, 8, 8,
    8, 19, 8, 19, 19, 19, 19, 19, 19, 4, 19, 4, 4, 4, 4, 8, 8, 19, 4, 4, 4, 4,
    4, 8, 8, 19, 4, 4, 4, 4, 4, 8, 8, 19, 4, 4, 4, 4, 4, 8, 8, 19, 4, 4, 4, 4,
    4, 8, 8, 19, 4, 4, 4, 4, 4, 8, 8, 19, 4, 4, 4, 4, 4, 8, 8, 19, 4, 4, 4, 4,
    4, 8, 8, 19, 4, 4, 4, 4, 4, 8, 8, 19, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 0,
    4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 14, 4, 23, 4,
    15, 4, 4, 4, 8, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 10, 4, 4, 4, 4,
    4, 4};

enum z80_flagbit {
    cf = 0,
    nf = 1,
    pf = 2,
    xf = 3,
    hf = 4,
    yf = 5,
    zf = 6,
    sf = 7
};


// MARK: helpers

// get bit "n" of number "val"
#define GET_BIT(n, val) (((val) >> (n)) & 1)

static inline uint8_t flag_val(enum z80_flagbit bit, bool cond) {
	return (!!cond) << bit;
}

static inline bool flag_get(z80* const z, enum z80_flagbit bit) {
	return !!(z->f & (1 << bit));
}

static inline void flag_set(z80* const z, enum z80_flagbit bit, bool val) {
	z->f &= ~(1<<bit);
	z->f |= (!!val) << bit;
}

static inline uint8_t rb(z80* const z, uint16_t addr) {
  return z->read_byte(z->userdata, addr);
}

static inline void wb(z80* const z, uint16_t addr, uint8_t val) {
  z->write_byte(z->userdata, addr, val);
}

static inline uint16_t rw(z80* const z, uint16_t addr) {
  return (z->read_byte(z->userdata, addr + 1) << 8) |
         z->read_byte(z->userdata, addr);
}

static inline void ww(z80* const z, uint16_t addr, uint16_t val) {
  z->write_byte(z->userdata, addr, val & 0xFF);
  z->write_byte(z->userdata, addr + 1, val >> 8);
}

static inline void pushw(z80* const z, uint16_t val) {
  z->sp -= 2;
  ww(z, z->sp, val);
}

static inline uint16_t popw(z80* const z) {
  z->sp += 2;
  return rw(z, z->sp - 2);
}

static inline uint8_t nextb(z80* const z) {
  return rb(z, z->pc++);
}

static inline uint16_t nextw(z80* const z) {
  z->pc += 2;
  return rw(z, z->pc - 2);
}

// increments R, keeping the highest byte intact
static inline void inc_r(z80* const z) {
  z->r = (z->r & 0x80) | ((z->r + 1) & 0x7f);
}

// returns if there was a carry between bit "bit_no" and "bit_no - 1" when
// executing "a + b + cy"
static inline bool carry(int bit_no, uint16_t a, uint16_t b, bool cy) {
  int32_t result = a + b + cy;
  int32_t carry = result ^ a ^ b;
  return carry & (1 << bit_no);
}

// returns the parity of byte: 0 if number of 1 bits in `val` is odd, else 1
static inline bool parity(uint8_t val) {
  uint8_t nb_one_bits = 0;
  for (int i = 0; i < 8; i++) {
    nb_one_bits += ((val >> i) & 1);
  }

  return (nb_one_bits & 1) == 0;
}

static unsigned exec_opcode(z80* const z, uint8_t opcode);
static unsigned exec_opcode_cb(z80* const z, uint8_t opcode);
static unsigned exec_opcode_dcb(
    z80* const z, const uint8_t opcode, const uint16_t addr);
static unsigned exec_opcode_ed(z80* const z, uint8_t opcode);
static unsigned exec_opcode_ddfd(z80* const z, uint8_t opcode, uint16_t* const iz);

// MARK: opcodes
// jumps to an address
static inline void jump(z80* const z, uint16_t addr) {
  z->pc = addr;
  z->mem_ptr = addr;
}

// jumps to next word in memory if condition is true
static inline void cond_jump(z80* const z, bool condition) {
  const uint16_t addr = nextw(z);
  if (condition) {
    jump(z, addr);
  }
  z->mem_ptr = addr;
}

// calls to next word in memory
static inline void call(z80* const z, uint16_t addr) {
  pushw(z, z->pc);
  z->pc = addr;
  z->mem_ptr = addr;
}

// calls to next word in memory if condition is true
static inline unsigned cond_call(z80* const z, bool condition) {
  const uint16_t addr = nextw(z);
  unsigned cyc = 0;
  if (condition) {
    call(z, addr);
    cyc = 7;
  }
  z->mem_ptr = addr;
  return cyc;
}

// returns from subroutine
static inline void ret(z80* const z) {
  z->pc = popw(z);
  z->mem_ptr = z->pc;
}

// returns from subroutine if condition is true
static inline unsigned cond_ret(z80* const z, bool condition) {
  if (condition) {
    ret(z);
    return 6;
  }
  return 0;
}

static inline void jr(z80* const z, int8_t displacement) {
  z->pc += displacement;
  z->mem_ptr = z->pc;
}

static inline unsigned cond_jr(z80* const z, bool condition) {
  const int8_t b = nextb(z);
  if (condition) {
    jr(z, b);
    return 5;
  }
  return 0;
}

// ADD Byte: adds two bytes together
static inline uint8_t addb(z80* const z, uint8_t a, uint8_t b, bool cy) {
  const uint8_t result = a + b + cy;
  z->f = 0 |
    flag_val(sf, result >> 7) |
    flag_val(zf, result == 0) |
    flag_val(hf, carry(4, a, b, cy)) |
    flag_val(pf, carry(7, a, b, cy) != carry(8, a, b, cy)) |
    flag_val(cf, carry(8, a, b, cy)) |
    flag_val(nf, 0) |
    flag_val(xf, GET_BIT(3, result)) |
    flag_val(yf, GET_BIT(5, result));
  return result;
}

// SUBstract Byte: substracts two bytes (with optional carry)
static inline uint8_t subb(z80* const z, uint8_t a, uint8_t b, bool cy) {
  uint8_t val = addb(z, a, ~b, !cy);
  flag_set(z, cf, !flag_get(z, cf));
  flag_set(z, hf, !flag_get(z, hf));
  flag_set(z, nf, 1);
  return val;
}

// ADD Word: adds two words together
static inline uint16_t addw(z80* const z, uint16_t a, uint16_t b, bool cy) {
  uint8_t lsb = addb(z, a, b, cy);
  uint8_t msb = addb(z, a >> 8, b >> 8, flag_get(z, cf));

  uint16_t result = (msb << 8) | lsb;
  flag_set(z, zf, result == 0);
  z->mem_ptr = a + 1;
  return result;
}

// SUBstract Word: substracts two words (with optional carry)
static inline uint16_t subw(z80* const z, uint16_t a, uint16_t b, bool cy) {
  uint8_t lsb = subb(z, a, b, cy);
  uint8_t msb = subb(z, a >> 8, b >> 8, flag_get(z, cf));

  uint16_t result = (msb << 8) | lsb;
  flag_set(z, zf, result == 0);
  z->mem_ptr = a + 1;
  return result;
}

// adds a word to HL
static inline void addhl(z80* const z, uint16_t val) {
  bool sfc = flag_get(z, sf);
  bool zfc = flag_get(z, zf);
  bool pfc = flag_get(z, pf);
  uint16_t result = addw(z, z->hl, val, 0);
  z->hl = result;
  flag_set(z, sf, sfc);
  flag_set(z, zf, zfc);
  flag_set(z, pf, pfc);
}

// adds a word to IX or IY
static inline void addiz(z80* const z, uint16_t* reg, uint16_t val) {
  bool sfc = flag_get(z, sf);
  bool zfc = flag_get(z, zf);
  bool pfc = flag_get(z, pf);
  uint16_t result = addw(z, *reg, val, 0);
  *reg = result;
  flag_set(z, sf, sfc);
  flag_set(z, zf, zfc);
  flag_set(z, pf, pfc);
}

// adds a word (+ carry) to HL
static inline void adchl(z80* const z, uint16_t val) {
  uint16_t result = addw(z, z->hl, val, flag_get(z, cf));
  flag_set(z, sf, result >> 15);
  flag_set(z, zf, result == 0);
  z->hl = result;
}

// substracts a word (+ carry) to HL
static inline void sbchl(z80* const z, uint16_t val) {
  const uint16_t result = subw(z, z->hl, val, flag_get(z, cf));
  flag_set(z, sf, result >> 15);
  flag_set(z, zf, result == 0);
  z->hl = result;
}

// increments a byte value
static inline uint8_t inc(z80* const z, uint8_t a) {
  bool cfc = flag_get(z, cf);
  uint8_t result = addb(z, a, 1, 0);
  flag_set(z, cf, cfc);
  return result;
}

// decrements a byte value
static inline uint8_t dec(z80* const z, uint8_t a) {
  bool cfc = flag_get(z, cf);
  uint8_t result = subb(z, a, 1, 0);
  flag_set(z, cf, cfc);
  return result;
}

// MARK: bitwise

// executes a logic "and" between register A and a byte, then stores the
// result in register A
static inline void land(z80* const z, uint8_t val) {
  const uint8_t result = z->a & val;
  z->f = 0 |
    flag_val(sf, result >> 7) |
    flag_val(zf, result == 0) |
    flag_val(hf,  1) |
    flag_val(pf, parity(result)) |
    flag_val(nf, 0) |
    flag_val(cf, 0) |
    flag_val(xf, GET_BIT(3, result)) |
    flag_val(yf, GET_BIT(5, result));
  z->a = result;
}

// executes a logic "xor" between register A and a byte, then stores the
// result in register A
static inline void lxor(z80* const z, const uint8_t val) {
  const uint8_t result = z->a ^ val;
  z->f = 0 |
    flag_val(sf, result >> 7) |
    flag_val(zf, result == 0) |
    flag_val(hf, 0) |
    flag_val(pf, parity(result)) |
    flag_val(nf, 0) |
    flag_val(cf, 0) |
    flag_val(xf, GET_BIT(3, result)) |
    flag_val(yf, GET_BIT(5, result));
  z->a = result;
}

// executes a logic "or" between register A and a byte, then stores the
// result in register A
static inline void lor(z80* const z, const uint8_t val) {
  const uint8_t result = z->a | val;
  z->f = 0 |
    flag_val(sf, result >> 7) |
    flag_val(zf, result == 0) |
    flag_val(hf, 0) |
    flag_val(pf, parity(result)) |
    flag_val(nf, 0) |
    flag_val(cf, 0) |
    flag_val(xf, GET_BIT(3, result)) |
    flag_val(yf, GET_BIT(5, result));
  z->a = result;
}

// compares a value with register A
static inline void cp(z80* const z, const uint8_t val) {
  subb(z, z->a, val, 0);

  // the only difference between cp and sub is that
  // the xf/yf are taken from the value to be substracted,
  // not the result
  flag_set(z, yf, GET_BIT(5, val));
  flag_set(z, xf, GET_BIT(3, val));
}

// 0xCB opcodes
// rotate left with carry
static inline uint8_t cb_rlc(z80* const z, uint8_t val) {
  const bool old = val >> 7;
  val = (val << 1) | old;
  z->f = 0 |
    flag_val(sf, val >> 7) |
    flag_val(zf, val == 0) |
    flag_val(pf, parity(val)) |
    flag_val(nf, 0) |
    flag_val(hf, 0) |
    flag_val(cf, old) |
    flag_val(xf, GET_BIT(3, val)) |
    flag_val(yf, GET_BIT(5, val));
  return val;
}

// rotate right with carry
static inline uint8_t cb_rrc(z80* const z, uint8_t val) {
  const bool old = val & 1;
  val = (val >> 1) | (old << 7);
  z->f = 0 |
    flag_val(sf, val >> 7) |
    flag_val(zf, val == 0) |
    flag_val(nf, 0) |
    flag_val(hf, 0) |
    flag_val(cf, old) |
    flag_val(pf, parity(val)) |
    flag_val(xf, GET_BIT(3, val)) |
    flag_val(yf, GET_BIT(5, val));
  return val;
}

// rotate left (simple)
static inline uint8_t cb_rl(z80* const z, uint8_t val) {
  const bool cfc = flag_get(z, cf);
  const bool cfn = val >> 7;
  val = (val << 1) | cfc;
  z->f = 0 |
    flag_val(cf, cfn) |
    flag_val(sf, val >> 7) |
    flag_val(zf, val == 0) |
    flag_val(nf, 0) |
    flag_val(hf, 0) |
    flag_val(pf, parity(val)) |
    flag_val(xf, GET_BIT(3, val)) |
    flag_val(yf, GET_BIT(5, val));
  return val;
}

// rotate right (simple)
static inline uint8_t cb_rr(z80* const z, uint8_t val) {
  const bool c = flag_get(z, cf);
  const bool cfn = val & 1;
  val = (val >> 1) | (c << 7);
  z->f = 0 |
    flag_val(cf, cfn) |
    flag_val(sf, val >> 7) |
    flag_val(zf, val == 0) |
    flag_val(nf, 0) |
    flag_val(hf, 0) |
    flag_val(pf, parity(val)) |
    flag_val(xf, GET_BIT(3, val)) |
    flag_val(yf, GET_BIT(5, val));
  return val;
}

// shift left preserving sign
static inline uint8_t cb_sla(z80* const z, uint8_t val) {
  const bool cfn = val >> 7;
  val <<= 1;
  z->f = 0 |
    flag_val(cf, cfn) |
    flag_val(sf, val >> 7) |
    flag_val(zf, val == 0) |
    flag_val(nf, 0) |
    flag_val(hf, 0) |
    flag_val(pf, parity(val)) |
    flag_val(xf, GET_BIT(3, val)) |
    flag_val(yf, GET_BIT(5, val));
  return val;
}

// SLL (exactly like SLA, but sets the first bit to 1)
static inline uint8_t cb_sll(z80* const z, uint8_t val) {
  const bool cfn = val >> 7;
  val <<= 1;
  val |= 1;
  z->f = 0 |
    flag_val(cf, cfn) |
    flag_val(sf, val >> 7) |
    flag_val(zf, val == 0) |
    flag_val(nf, 0) |
    flag_val(hf, 0) |
    flag_val(pf, parity(val)) |
    flag_val(xf, GET_BIT(3, val)) |
    flag_val(yf, GET_BIT(5, val));
  return val;
}

// shift right preserving sign
static inline uint8_t cb_sra(z80* const z, uint8_t val) {
  const bool cfn = val & 1;
  val = (val >> 1) | (val & 0x80); // 0b10000000
  z->f = 0 |
    flag_val(cf, cfn) |
    flag_val(sf, val >> 7) |
    flag_val(zf, val == 0) |
    flag_val(nf, 0) |
    flag_val(hf, 0) |
    flag_val(pf, parity(val)) |
    flag_val(xf, GET_BIT(3, val)) |
    flag_val(yf, GET_BIT(5, val));
  return val;
}

// shift register right
static inline uint8_t cb_srl(z80* const z, uint8_t val) {
  const bool cfn = val & 1;
  val >>= 1;
  z->f = 0 |
    flag_val(cf, cfn) |
    flag_val(sf, val >> 7) |
    flag_val(zf, val == 0) |
    flag_val(nf, 0) |
    flag_val(hf, 0) |
    flag_val(pf, parity(val)) |
    flag_val(xf, GET_BIT(3, val)) |
    flag_val(yf, GET_BIT(5, val));
  return val;
}

// tests bit "n" from a byte
static inline uint8_t cb_bit(z80* const z, uint8_t val, uint8_t n) {
  const uint8_t result = val & (1 << n);
  z->f = 0 |
    flag_val(cf, flag_get(z, cf)) | /* original code didn't set this one, so use old value */
    flag_val(sf, result >> 7) |
    flag_val(zf, result == 0) |
    flag_val(yf, GET_BIT(5, val)) |
    flag_val(hf, 1) |
    flag_val(xf, GET_BIT(3, val)) |
    flag_val(pf, result == 0) |
    flag_val(nf, 0);
  return result;
}

static inline void ldi(z80* const z) {
  const uint16_t de = z->de;
  const uint16_t hl = z->hl;
  const uint8_t val = rb(z, hl);

  wb(z, de, val);

  ++z->hl;
  ++z->de;
  --z->bc;

  // see https://wikiti.brandonw.net/index.php?title=Z80_Instruction_Set
  // for the calculation of xf/yf on LDI
  const uint8_t result = val + z->a;
  flag_set(z, xf, GET_BIT(3, result));
  flag_set(z, yf, GET_BIT(1, result));

  flag_set(z, nf, 0);
  flag_set(z, hf, 0);
  flag_set(z, pf, z->bc > 0);
}

static inline void ldd(z80* const z) {
  ldi(z);
  // same as ldi but HL and DE are decremented instead of incremented
  z->hl -= 2;
  z->de -= 2;
}

static inline void cpi(z80* const z) {
  bool cfc = flag_get(z, cf);
  const uint8_t result = subb(z, z->a, rb(z, z->hl), 0);
  ++z->hl;
  --z->bc;
  bool hfc = flag_get(z, hf);
  flag_set(z, xf, GET_BIT(3, result - hfc));
  flag_set(z, yf, GET_BIT(1, result - hfc));
  flag_set(z, pf, z->bc != 0);
  flag_set(z, cf, cfc);
  z->mem_ptr += 1;
}

static inline void cpd(z80* const z) {
  cpi(z);
  // same as cpi but HL is decremented instead of incremented
  z->hl -= 2;
  z->mem_ptr -= 2;
}

static void in_r_c(z80* const z, uint8_t* r) {
  *r = z->port_in(z, z->c);
  flag_set(z, zf, *r == 0);
  flag_set(z, sf, *r >> 7);
  flag_set(z, pf, parity(*r));
  flag_set(z, nf, 0);
  flag_set(z, hf, 0);
}

static void ini(z80* const z) {
  uint8_t val = z->port_in(z, z->c);
  wb(z, z->hl, val);
  ++z->hl;
  z->b -= 1;
  flag_set(z, zf, z->b == 0);
  flag_set(z, nf, 1);
  z->mem_ptr = z->bc + 1;
}

static void ind(z80* const z) {
  ini(z);
  z->hl -= 2;
  z->mem_ptr = z->bc - 2;
}

static void outi(z80* const z) {
  z->port_out(z, z->c, rb(z, z->hl));
  ++z->hl;
  z->b -= 1;
  flag_set(z, zf, z->b == 0);
  flag_set(z, nf, 1);
  z->mem_ptr = z->bc + 1;
}

static void outd(z80* const z) {
  outi(z);
  z->hl -= 2;
  z->mem_ptr = z->bc - 2;
}

static void daa(z80* const z) {
  // "When this instruction is executed, the A register is BCD corrected
  // using the  contents of the flags. The exact process is the following:
  // if the least significant four bits of A contain a non-BCD digit
  // (i. e. it is greater than 9) or the H flag is set, then $06 is
  // added to the register. Then the four most significant bits are
  // checked. If this more significant digit also happens to be greater
  // than 9 or the C flag is set, then $60 is added."
  // > http://z80-heaven.wikidot.com/instructions-set:daa
  uint8_t correction = 0;

  if ((z->a & 0x0F) > 0x09 || flag_get(z, hf)) {
    correction += 0x06;
  }

  if (z->a > 0x99 || flag_get(z, cf)) {
    correction += 0x60;
    flag_set(z, cf, 1);
  }

  const bool substraction = flag_get(z, nf);
  if (substraction) {
    flag_set(z, hf, flag_get(z, hf) && (z->a & 0x0F) < 0x06);
    z->a -= correction;
  } else {
    flag_set(z, hf, (z->a & 0x0F) > 0x09);
    z->a += correction;
  }

  flag_set(z, sf, z->a >> 7);
  flag_set(z, zf, z->a == 0);
  flag_set(z, pf, parity(z->a));
  flag_set(z, xf, GET_BIT(3, z->a));
  flag_set(z, yf, GET_BIT(5, z->a));
}

static inline uint16_t displace(
    z80* const z, uint16_t base_addr, int8_t displacement) {
  const uint16_t addr = base_addr + displacement;
  z->mem_ptr = addr;
  return addr;
}

static inline unsigned process_interrupts(z80* const z) {
  unsigned cyc = 0;
  // "When an EI instruction is executed, any pending interrupt request
  // is not accepted until after the instruction following EI is executed."
  if (z->iff_delay > 0) {
    z->iff_delay -= 1;
    if (z->iff_delay == 0) {
      z->iff1 = 1;
      z->iff2 = 1;
    }
    return cyc;
  }

  if (z->nmi_pending) {
    z->nmi_pending = 0;
    z->halted = 0;
    z->iff1 = 0;
    inc_r(z);

    cyc += 11;
    call(z, 0x66);
    return cyc;
  }

  if (z->int_pending && z->iff1) {
    //z->int_pending = 0; // Commented when function to clear the line was added
    z->halted = 0;
    z->iff1 = 0;
    z->iff2 = 0;
    inc_r(z);

    switch (z->interrupt_mode) {
    case 0:
      cyc += 11;
      cyc += exec_opcode(z, z->int_data);
      break;

    case 1:
      cyc += 13;
      call(z, 0x38);
      break;

    case 2:
      cyc += 19;
      call(z, rw(z, (z->i << 8) | z->int_data));
      break;

    default:
      //fprintf(stderr, "unsupported interrupt mode %d\n", z->interrupt_mode);
      break;
    }

    return cyc;
  }
  return cyc;
}

// MARK: interface
// initialises a z80 struct. Note that read_byte, write_byte, port_in, port_out
// and userdata must be manually set by the user afterwards.
void z80_init(z80* const z) {
  z->read_byte = NULL;
  z->write_byte = NULL;
  z->port_in = NULL;
  z->port_out = NULL;
  z->userdata = NULL;

  z->pc = 0;
  z->sp = 0xFFFF;
  z->ix = 0;
  z->iy = 0;
  z->mem_ptr = 0;

  // af and sp are set to 0xFFFF after reset,
  // and the other values are undefined (z80-documented)
  z->af = 0xFFFF;
  z->bc = 0;
  z->de = 0;
  z->hl = 0;

  z->a_f_ = 0;
  z->b_c_ = 0;
  z->d_e_ = 0;
  z->h_l_ = 0;

  z->i = 0;
  z->r = 0;

  z->iff_delay = 0;
  z->interrupt_mode = 0;
  z->iff1 = 0;
  z->iff2 = 0;
  z->halted = 0;
  z->int_pending = 0;
  z->nmi_pending = 0;
  z->int_data = 0;
}

static unsigned z80_step_s(z80* const z) {
  unsigned cyc = 0;
  if (z->halted) {
    cyc += exec_opcode(z, 0x00);
  } else {
    const uint8_t opcode = nextb(z);
    cyc += exec_opcode(z, opcode);
  }

  cyc += process_interrupts(z);
  return cyc;
}

// executes the next instruction in memory + handles interrupts
unsigned z80_step(z80* const z) {
  return z80_step_s(z);
}

// executes the next instructions in memory + handles interrupts,
// until the cycle count is >= the requested amount.
unsigned z80_step_n(z80* const z, unsigned cycles) {
  unsigned cyc = 0;
  while (cyc < cycles) {
    cyc += z80_step_s(z);
  }
  return cyc;
}

// outputs to stdout a debug trace of the emulator
void z80_debug_output(z80* const z) {
  if (z) { }
  /*printf("PC: %04X, AF: %04X, BC: %04X, DE: %04X, HL: %04X, SP: %04X, "
         "IX: %04X, IY: %04X, I: %02X, R: %02X",
      z->pc, (z->a << 8) | z->f, z->bc, z->de, z->hl, z->sp,
      z->ix, z->iy, z->i, z->r);

  printf("\t(%02X %02X %02X %02X), cyc: %lu\n", rb(z, z->pc), rb(z, z->pc + 1),
      rb(z, z->pc + 2), rb(z, z->pc + 3), z->cyc);*/
}

// function to call when an NMI is to be serviced
void z80_gen_nmi(z80* const z) {
  z->nmi_pending = 1;
}

// function to call when an INT is to be serviced
void z80_gen_int(z80* const z, uint8_t data) {
  z->int_pending = 1;
  z->int_data = data;
}

void z80_clr_int(z80* const z) {
    z->int_pending = 0;
}

// executes a non-prefixed opcode
unsigned exec_opcode(z80* const z, uint8_t opcode) {
  unsigned cyc = cyc_00[opcode];
  inc_r(z);

  switch (opcode) {
  case 0x7F: z->a = z->a; break; // ld a,a
  case 0x78: z->a = z->b; break; // ld a,b
  case 0x79: z->a = z->c; break; // ld a,c
  case 0x7A: z->a = z->d; break; // ld a,d
  case 0x7B: z->a = z->e; break; // ld a,e
  case 0x7C: z->a = z->h; break; // ld a,h
  case 0x7D: z->a = z->l; break; // ld a,l

  case 0x47: z->b = z->a; break; // ld b,a
  case 0x40: z->b = z->b; break; // ld b,b
  case 0x41: z->b = z->c; break; // ld b,c
  case 0x42: z->b = z->d; break; // ld b,d
  case 0x43: z->b = z->e; break; // ld b,e
  case 0x44: z->b = z->h; break; // ld b,h
  case 0x45: z->b = z->l; break; // ld b,l

  case 0x4F: z->c = z->a; break; // ld c,a
  case 0x48: z->c = z->b; break; // ld c,b
  case 0x49: z->c = z->c; break; // ld c,c
  case 0x4A: z->c = z->d; break; // ld c,d
  case 0x4B: z->c = z->e; break; // ld c,e
  case 0x4C: z->c = z->h; break; // ld c,h
  case 0x4D: z->c = z->l; break; // ld c,l

  case 0x57: z->d = z->a; break; // ld d,a
  case 0x50: z->d = z->b; break; // ld d,b
  case 0x51: z->d = z->c; break; // ld d,c
  case 0x52: z->d = z->d; break; // ld d,d
  case 0x53: z->d = z->e; break; // ld d,e
  case 0x54: z->d = z->h; break; // ld d,h
  case 0x55: z->d = z->l; break; // ld d,l

  case 0x5F: z->e = z->a; break; // ld e,a
  case 0x58: z->e = z->b; break; // ld e,b
  case 0x59: z->e = z->c; break; // ld e,c
  case 0x5A: z->e = z->d; break; // ld e,d
  case 0x5B: z->e = z->e; break; // ld e,e
  case 0x5C: z->e = z->h; break; // ld e,h
  case 0x5D: z->e = z->l; break; // ld e,l

  case 0x67: z->h = z->a; break; // ld h,a
  case 0x60: z->h = z->b; break; // ld h,b
  case 0x61: z->h = z->c; break; // ld h,c
  case 0x62: z->h = z->d; break; // ld h,d
  case 0x63: z->h = z->e; break; // ld h,e
  case 0x64: z->h = z->h; break; // ld h,h
  case 0x65: z->h = z->l; break; // ld h,l

  case 0x6F: z->l = z->a; break; // ld l,a
  case 0x68: z->l = z->b; break; // ld l,b
  case 0x69: z->l = z->c; break; // ld l,c
  case 0x6A: z->l = z->d; break; // ld l,d
  case 0x6B: z->l = z->e; break; // ld l,e
  case 0x6C: z->l = z->h; break; // ld l,h
  case 0x6D: z->l = z->l; break; // ld l,l

  case 0x7E: z->a = rb(z, z->hl); break; // ld a,(hl)
  case 0x46: z->b = rb(z, z->hl); break; // ld b,(hl)
  case 0x4E: z->c = rb(z, z->hl); break; // ld c,(hl)
  case 0x56: z->d = rb(z, z->hl); break; // ld d,(hl)
  case 0x5E: z->e = rb(z, z->hl); break; // ld e,(hl)
  case 0x66: z->h = rb(z, z->hl); break; // ld h,(hl)
  case 0x6E: z->l = rb(z, z->hl); break; // ld l,(hl)

  case 0x77: wb(z, z->hl, z->a); break; // ld (hl),a
  case 0x70: wb(z, z->hl, z->b); break; // ld (hl),b
  case 0x71: wb(z, z->hl, z->c); break; // ld (hl),c
  case 0x72: wb(z, z->hl, z->d); break; // ld (hl),d
  case 0x73: wb(z, z->hl, z->e); break; // ld (hl),e
  case 0x74: wb(z, z->hl, z->h); break; // ld (hl),h
  case 0x75: wb(z, z->hl, z->l); break; // ld (hl),l

  case 0x3E: z->a = nextb(z); break; // ld a,*
  case 0x06: z->b = nextb(z); break; // ld b,*
  case 0x0E: z->c = nextb(z); break; // ld c,*
  case 0x16: z->d = nextb(z); break; // ld d,*
  case 0x1E: z->e = nextb(z); break; // ld e,*
  case 0x26: z->h = nextb(z); break; // ld h,*
  case 0x2E: z->l = nextb(z); break; // ld l,*
  case 0x36: wb(z, z->hl, nextb(z)); break; // ld (hl),*

  case 0x0A:
    z->a = rb(z, z->bc);
    z->mem_ptr = z->bc + 1;
    break; // ld a,(bc)
  case 0x1A:
    z->a = rb(z, z->de);
    z->mem_ptr = z->de + 1;
    break; // ld a,(de)
  case 0x3A: {
    const uint16_t addr = nextw(z);
    z->a = rb(z, addr);
    z->mem_ptr = addr + 1;
  } break; // ld a,(**)

  case 0x02:
    wb(z, z->bc, z->a);
    z->mem_ptr = (z->a << 8) | ((z->bc + 1) & 0xFF);
    break; // ld (bc),a

  case 0x12:
    wb(z, z->de, z->a);
    z->mem_ptr = (z->a << 8) | ((z->de + 1) & 0xFF);
    break; // ld (de),a

  case 0x32: {
    const uint16_t addr = nextw(z);
    wb(z, addr, z->a);
    z->mem_ptr = (z->a << 8) | ((addr + 1) & 0xFF);
  } break; // ld (**),a

  case 0x01: z->bc = nextw(z); break; // ld bc,**
  case 0x11: z->de = nextw(z); break; // ld de,**
  case 0x21: z->hl = nextw(z); break; // ld hl,**
  case 0x31: z->sp = nextw(z); break; // ld sp,**

  case 0x2A: {
    const uint16_t addr = nextw(z);
    z->hl = rw(z, addr);
    z->mem_ptr = addr + 1;
  } break; // ld hl,(**)

  case 0x22: {
    const uint16_t addr = nextw(z);
    ww(z, addr, z->hl);
    z->mem_ptr = addr + 1;
  } break; // ld (**),hl

  case 0xF9: z->sp = z->hl; break; // ld sp,hl

  case 0xEB: {
    const uint16_t de = z->de;
    z->de = z->hl;
    z->hl = de;
  } break; // ex de,hl

  case 0xE3: {
    const uint16_t val = rw(z, z->sp);
    ww(z, z->sp, z->hl);
    z->hl = val;
    z->mem_ptr = val;
  } break; // ex (sp),hl

  case 0x87: z->a = addb(z, z->a, z->a, 0); break; // add a,a
  case 0x80: z->a = addb(z, z->a, z->b, 0); break; // add a,b
  case 0x81: z->a = addb(z, z->a, z->c, 0); break; // add a,c
  case 0x82: z->a = addb(z, z->a, z->d, 0); break; // add a,d
  case 0x83: z->a = addb(z, z->a, z->e, 0); break; // add a,e
  case 0x84: z->a = addb(z, z->a, z->h, 0); break; // add a,h
  case 0x85: z->a = addb(z, z->a, z->l, 0); break; // add a,l
  case 0x86: z->a = addb(z, z->a, rb(z, z->hl), 0); break; // add a,(hl)
  case 0xC6: z->a = addb(z, z->a, nextb(z), 0); break; // add a,*

  case 0x8F: z->a = addb(z, z->a, z->a, flag_get(z, cf)); break; // adc a,a
  case 0x88: z->a = addb(z, z->a, z->b, flag_get(z, cf)); break; // adc a,b
  case 0x89: z->a = addb(z, z->a, z->c, flag_get(z, cf)); break; // adc a,c
  case 0x8A: z->a = addb(z, z->a, z->d, flag_get(z, cf)); break; // adc a,d
  case 0x8B: z->a = addb(z, z->a, z->e, flag_get(z, cf)); break; // adc a,e
  case 0x8C: z->a = addb(z, z->a, z->h, flag_get(z, cf)); break; // adc a,h
  case 0x8D: z->a = addb(z, z->a, z->l, flag_get(z, cf)); break; // adc a,l
  case 0x8E: z->a = addb(z, z->a, rb(z, z->hl), flag_get(z, cf)); break; // adc a,(hl)
  case 0xCE: z->a = addb(z, z->a, nextb(z), flag_get(z, cf)); break; // adc a,*

  case 0x97: z->a = subb(z, z->a, z->a, 0); break; // sub a,a
  case 0x90: z->a = subb(z, z->a, z->b, 0); break; // sub a,b
  case 0x91: z->a = subb(z, z->a, z->c, 0); break; // sub a,c
  case 0x92: z->a = subb(z, z->a, z->d, 0); break; // sub a,d
  case 0x93: z->a = subb(z, z->a, z->e, 0); break; // sub a,e
  case 0x94: z->a = subb(z, z->a, z->h, 0); break; // sub a,h
  case 0x95: z->a = subb(z, z->a, z->l, 0); break; // sub a,l
  case 0x96: z->a = subb(z, z->a, rb(z, z->hl), 0); break; // sub a,(hl)
  case 0xD6: z->a = subb(z, z->a, nextb(z), 0); break; // sub a,*

  case 0x9F: z->a = subb(z, z->a, z->a, flag_get(z, cf)); break; // sbc a,a
  case 0x98: z->a = subb(z, z->a, z->b, flag_get(z, cf)); break; // sbc a,b
  case 0x99: z->a = subb(z, z->a, z->c, flag_get(z, cf)); break; // sbc a,c
  case 0x9A: z->a = subb(z, z->a, z->d, flag_get(z, cf)); break; // sbc a,d
  case 0x9B: z->a = subb(z, z->a, z->e, flag_get(z, cf)); break; // sbc a,e
  case 0x9C: z->a = subb(z, z->a, z->h, flag_get(z, cf)); break; // sbc a,h
  case 0x9D: z->a = subb(z, z->a, z->l, flag_get(z, cf)); break; // sbc a,l
  case 0x9E: z->a = subb(z, z->a, rb(z, z->hl), flag_get(z, cf)); break; // sbc a,(hl)
  case 0xDE: z->a = subb(z, z->a, nextb(z), flag_get(z, cf)); break; // sbc a,*

  case 0x09: addhl(z, z->bc); break; // add hl,bc
  case 0x19: addhl(z, z->de); break; // add hl,de
  case 0x29: addhl(z, z->hl); break; // add hl,hl
  case 0x39: addhl(z, z->sp); break; // add hl,sp

  case 0xF3:
    z->iff1 = 0;
    z->iff2 = 0;
    break; // di
  case 0xFB: z->iff_delay = 1; break; // ei
  case 0x00: break; // nop
  case 0x76: z->halted = 1; break; // halt

  case 0x3C: z->a = inc(z, z->a); break; // inc a
  case 0x04: z->b = inc(z, z->b); break; // inc b
  case 0x0C: z->c = inc(z, z->c); break; // inc c
  case 0x14: z->d = inc(z, z->d); break; // inc d
  case 0x1C: z->e = inc(z, z->e); break; // inc e
  case 0x24: z->h = inc(z, z->h); break; // inc h
  case 0x2C: z->l = inc(z, z->l); break; // inc l
  case 0x34: {
    uint8_t result = inc(z, rb(z, z->hl));
    wb(z, z->hl, result);
  } break; // inc (hl)

  case 0x3D: z->a = dec(z, z->a); break; // dec a
  case 0x05: z->b = dec(z, z->b); break; // dec b
  case 0x0D: z->c = dec(z, z->c); break; // dec c
  case 0x15: z->d = dec(z, z->d); break; // dec d
  case 0x1D: z->e = dec(z, z->e); break; // dec e
  case 0x25: z->h = dec(z, z->h); break; // dec h
  case 0x2D: z->l = dec(z, z->l); break; // dec l
  case 0x35: {
    uint8_t result = dec(z, rb(z, z->hl));
    wb(z, z->hl, result);
  } break; // dec (hl)

  case 0x03: ++z->bc; break; // inc bc
  case 0x13: ++z->de; break; // inc de
  case 0x23: ++z->hl; break; // inc hl
  case 0x33: ++z->sp; break; // inc sp

  case 0x0B: --z->bc; break; // dec bc
  case 0x1B: --z->de; break; // dec de
  case 0x2B: --z->hl; break; // dec hl
  case 0x3B: --z->sp; break; // dec sp

  case 0x27: daa(z); break; // daa

  case 0x2F:
    z->a = ~z->a;
    flag_set(z, nf, 1);
    flag_set(z, hf, 1);
    flag_set(z, xf, GET_BIT(3, z->a));
    flag_set(z, yf, GET_BIT(5, z->a));
    break; // cpl

  case 0x37:
    flag_set(z, cf, 1);
    flag_set(z, nf, 0);
    flag_set(z, hf, 0);
    flag_set(z, xf, GET_BIT(3, z->a));
    flag_set(z, yf, GET_BIT(5, z->a));
    break; // scf

  case 0x3F:
    flag_set(z, hf, flag_get(z, cf));
    flag_set(z, cf, !flag_get(z, cf));
    flag_set(z, nf, 0);
    flag_set(z, xf, GET_BIT(3, z->a));
    flag_set(z, yf, GET_BIT(5, z->a));
    break; // ccf

  case 0x07: {
    flag_set(z, cf, z->a >> 7);
    z->a = (z->a << 1) | flag_get(z, cf);
    flag_set(z, nf, 0);
    flag_set(z, hf, 0);
    flag_set(z, xf, GET_BIT(3, z->a));
    flag_set(z, yf, GET_BIT(5, z->a));
  } break; // rlca (rotate left)

  case 0x0F: {
    flag_set(z, cf, z->a & 1);
    z->a = (z->a >> 1) | (flag_get(z, cf) << 7);
    flag_set(z, nf, 0);
    flag_set(z, hf, 0);
    flag_set(z, xf, GET_BIT(3, z->a));
    flag_set(z, yf, GET_BIT(5, z->a));
  } break; // rrca (rotate right)

  case 0x17: {
    const bool cy = flag_get(z, cf);
    flag_set(z, cf, z->a >> 7);
    z->a = (z->a << 1) | cy;
    flag_set(z, nf, 0);
    flag_set(z, hf, 0);
    flag_set(z, xf, GET_BIT(3, z->a));
    flag_set(z, yf, GET_BIT(5, z->a));
  } break; // rla

  case 0x1F: {
    const bool cy = flag_get(z, cf);
    flag_set(z, cf, z->a & 1);
    z->a = (z->a >> 1) | (cy << 7);
    flag_set(z, nf, 0);
    flag_set(z, hf, 0);
    flag_set(z, xf, GET_BIT(3, z->a));
    flag_set(z, yf, GET_BIT(5, z->a));
  } break; // rra

  case 0xA7: land(z, z->a); break; // and a
  case 0xA0: land(z, z->b); break; // and b
  case 0xA1: land(z, z->c); break; // and c
  case 0xA2: land(z, z->d); break; // and d
  case 0xA3: land(z, z->e); break; // and e
  case 0xA4: land(z, z->h); break; // and h
  case 0xA5: land(z, z->l); break; // and l
  case 0xA6: land(z, rb(z, z->hl)); break; // and (hl)
  case 0xE6: land(z, nextb(z)); break; // and *

  case 0xAF: lxor(z, z->a); break; // xor a
  case 0xA8: lxor(z, z->b); break; // xor b
  case 0xA9: lxor(z, z->c); break; // xor c
  case 0xAA: lxor(z, z->d); break; // xor d
  case 0xAB: lxor(z, z->e); break; // xor e
  case 0xAC: lxor(z, z->h); break; // xor h
  case 0xAD: lxor(z, z->l); break; // xor l
  case 0xAE: lxor(z, rb(z, z->hl)); break; // xor (hl)
  case 0xEE: lxor(z, nextb(z)); break; // xor *

  case 0xB7: lor(z, z->a); break; // or a
  case 0xB0: lor(z, z->b); break; // or b
  case 0xB1: lor(z, z->c); break; // or c
  case 0xB2: lor(z, z->d); break; // or d
  case 0xB3: lor(z, z->e); break; // or e
  case 0xB4: lor(z, z->h); break; // or h
  case 0xB5: lor(z, z->l); break; // or l
  case 0xB6: lor(z, rb(z, z->hl)); break; // or (hl)
  case 0xF6: lor(z, nextb(z)); break; // or *

  case 0xBF: cp(z, z->a); break; // cp a
  case 0xB8: cp(z, z->b); break; // cp b
  case 0xB9: cp(z, z->c); break; // cp c
  case 0xBA: cp(z, z->d); break; // cp d
  case 0xBB: cp(z, z->e); break; // cp e
  case 0xBC: cp(z, z->h); break; // cp h
  case 0xBD: cp(z, z->l); break; // cp l
  case 0xBE: cp(z, rb(z, z->hl)); break; // cp (hl)
  case 0xFE: cp(z, nextb(z)); break; // cp *

  case 0xC3: jump(z, nextw(z)); break; // jm **
  case 0xC2: cond_jump(z, flag_get(z, zf) == 0); break; // jp nz, **
  case 0xCA: cond_jump(z, flag_get(z, zf) == 1); break; // jp z, **
  case 0xD2: cond_jump(z, flag_get(z, cf) == 0); break; // jp nc, **
  case 0xDA: cond_jump(z, flag_get(z, cf) == 1); break; // jp c, **
  case 0xE2: cond_jump(z, flag_get(z, pf) == 0); break; // jp po, **
  case 0xEA: cond_jump(z, flag_get(z, pf) == 1); break; // jp pe, **
  case 0xF2: cond_jump(z, flag_get(z, sf) == 0); break; // jp p, **
  case 0xFA: cond_jump(z, flag_get(z, sf) == 1); break; // jp m, **

  case 0x10: cyc += cond_jr(z, --z->b != 0); break; // djnz *
  case 0x18: z->pc += (int8_t) nextb(z); break; // jr *
  case 0x20: cyc += cond_jr(z, flag_get(z, zf) == 0); break; // jr nz, *
  case 0x28: cyc += cond_jr(z, flag_get(z, zf) == 1); break; // jr z, *
  case 0x30: cyc += cond_jr(z, flag_get(z, cf) == 0); break; // jr nc, *
  case 0x38: cyc += cond_jr(z, flag_get(z, cf) == 1); break; // jr c, *

  case 0xE9: z->pc = z->hl; break; // jp (hl)
  case 0xCD: call(z, nextw(z)); break; // call

  case 0xC4: cyc += cond_call(z, flag_get(z, zf) == 0); break; // cnz
  case 0xCC: cyc += cond_call(z, flag_get(z, zf) == 1); break; // cz
  case 0xD4: cyc += cond_call(z, flag_get(z, cf) == 0); break; // cnc
  case 0xDC: cyc += cond_call(z, flag_get(z, cf) == 1); break; // cc
  case 0xE4: cyc += cond_call(z, flag_get(z, pf) == 0); break; // cpo
  case 0xEC: cyc += cond_call(z, flag_get(z, pf) == 1); break; // cpe
  case 0xF4: cyc += cond_call(z, flag_get(z, sf) == 0); break; // cp
  case 0xFC: cyc += cond_call(z, flag_get(z, sf) == 1); break; // cm

  case 0xC9: ret(z); break; // ret
  case 0xC0: cyc += cond_ret(z, flag_get(z, zf) == 0); break; // ret nz
  case 0xC8: cyc += cond_ret(z, flag_get(z, zf) == 1); break; // ret z
  case 0xD0: cyc += cond_ret(z, flag_get(z, cf) == 0); break; // ret nc
  case 0xD8: cyc += cond_ret(z, flag_get(z, cf) == 1); break; // ret c
  case 0xE0: cyc += cond_ret(z, flag_get(z, pf) == 0); break; // ret po
  case 0xE8: cyc += cond_ret(z, flag_get(z, pf) == 1); break; // ret pe
  case 0xF0: cyc += cond_ret(z, flag_get(z, sf) == 0); break; // ret p
  case 0xF8: cyc += cond_ret(z, flag_get(z, sf) == 1); break; // ret m

  case 0xC7: call(z, 0x00); break; // rst 0
  case 0xCF: call(z, 0x08); break; // rst 1
  case 0xD7: call(z, 0x10); break; // rst 2
  case 0xDF: call(z, 0x18); break; // rst 3
  case 0xE7: call(z, 0x20); break; // rst 4
  case 0xEF: call(z, 0x28); break; // rst 5
  case 0xF7: call(z, 0x30); break; // rst 6
  case 0xFF: call(z, 0x38); break; // rst 7

  case 0xC5: pushw(z, z->bc); break; // push bc
  case 0xD5: pushw(z, z->de); break; // push de
  case 0xE5: pushw(z, z->hl); break; // push hl
  case 0xF5: pushw(z, z->af); break; // push af

  case 0xC1: z->bc = popw(z); break; // pop bc
  case 0xD1: z->de = popw(z); break; // pop de
  case 0xE1: z->hl = popw(z); break; // pop hl
  case 0xF1: z->af = popw(z); break; // pop af

  case 0xDB: {
    const uint8_t port = nextb(z);
    const uint8_t a = z->a;
    z->a = z->port_in(z, port);
    z->mem_ptr = (a << 8) | (z->a + 1);
  } break; // in a,(n)

  case 0xD3: {
    const uint8_t port = nextb(z);
    z->port_out(z, port, z->a);
    z->mem_ptr = (port + 1) | (z->a << 8);
  } break; // out (n), a

  case 0x08: {
    uint16_t af = z->af;
    z->af = z->a_f_;
    z->a_f_ = af;
  } break; // ex af,af'
  case 0xD9: {
    uint16_t bc = z->bc, de = z->de, hl = z->hl;

    z->bc = z->b_c_;
    z->de = z->d_e_;
    z->hl = z->h_l_;

    z->b_c_ = bc;
    z->d_e_ = de;
    z->h_l_ = hl;
  } break; // exx

  case 0xCB: cyc += exec_opcode_cb(z, nextb(z)); break;
  case 0xED: cyc += exec_opcode_ed(z, nextb(z)); break;
  case 0xDD: cyc += exec_opcode_ddfd(z, nextb(z), &z->ix); break;
  case 0xFD: cyc += exec_opcode_ddfd(z, nextb(z), &z->iy); break;

  default: break; // fprintf(stderr, "unknown opcode %02X\n", opcode); break;
  }
  return cyc;
}

// executes a DD/FD opcode (IZ = IX or IY)
unsigned exec_opcode_ddfd(z80* const z, uint8_t opcode, uint16_t* const iz) {
  unsigned cyc = cyc_ddfd[opcode];
  inc_r(z);

#define IZD displace(z, *iz, nextb(z))
#define IZH (*iz >> 8)
#define IZL (*iz & 0xFF)

  switch (opcode) {
  case 0xE1: *iz = popw(z); break; // pop iz
  case 0xE5: pushw(z, *iz); break; // push iz

  case 0xE9: jump(z, *iz); break; // jp iz

  case 0x09: addiz(z, iz, z->bc); break; // add iz,bc
  case 0x19: addiz(z, iz, z->de); break; // add iz,de
  case 0x29: addiz(z, iz, *iz); break; // add iz,iz
  case 0x39: addiz(z, iz, z->sp); break; // add iz,sp

  case 0x84: z->a = addb(z, z->a, IZH, 0); break; // add a,izh
  case 0x85: z->a = addb(z, z->a, *iz & 0xFF, 0); break; // add a,izl
  case 0x8C: z->a = addb(z, z->a, IZH, flag_get(z, cf)); break; // adc a,izh
  case 0x8D: z->a = addb(z, z->a, *iz & 0xFF, flag_get(z, cf)); break; // adc a,izl

  case 0x86: z->a = addb(z, z->a, rb(z, IZD), 0); break; // add a,(iz+*)
  case 0x8E: z->a = addb(z, z->a, rb(z, IZD), flag_get(z, cf)); break; // adc a,(iz+*)
  case 0x96: z->a = subb(z, z->a, rb(z, IZD), 0); break; // sub (iz+*)
  case 0x9E: z->a = subb(z, z->a, rb(z, IZD), flag_get(z, cf)); break; // sbc (iz+*)

  case 0x94: z->a = subb(z, z->a, IZH, 0); break; // sub izh
  case 0x95: z->a = subb(z, z->a, *iz & 0xFF, 0); break; // sub izl
  case 0x9C: z->a = subb(z, z->a, IZH, flag_get(z, cf)); break; // sbc izh
  case 0x9D: z->a = subb(z, z->a, *iz & 0xFF, flag_get(z, cf)); break; // sbc izl

  case 0xA6: land(z, rb(z, IZD)); break; // and (iz+*)
  case 0xA4: land(z, IZH); break; // and izh
  case 0xA5: land(z, *iz & 0xFF); break; // and izl

  case 0xAE: lxor(z, rb(z, IZD)); break; // xor (iz+*)
  case 0xAC: lxor(z, IZH); break; // xor izh
  case 0xAD: lxor(z, *iz & 0xFF); break; // xor izl

  case 0xB6: lor(z, rb(z, IZD)); break; // or (iz+*)
  case 0xB4: lor(z, IZH); break; // or izh
  case 0xB5: lor(z, *iz & 0xFF); break; // or izl

  case 0xBE: cp(z, rb(z, IZD)); break; // cp (iz+*)
  case 0xBC: cp(z, IZH); break; // cp izh
  case 0xBD: cp(z, *iz & 0xFF); break; // cp izl

  case 0x23: *iz += 1; break; // inc iz
  case 0x2B: *iz -= 1; break; // dec iz

  case 0x34: {
    uint16_t addr = IZD;
    wb(z, addr, inc(z, rb(z, addr)));
  } break; // inc (iz+*)

  case 0x35: {
    uint16_t addr = IZD;
    wb(z, addr, dec(z, rb(z, addr)));
  } break; // dec (iz+*)

  case 0x24: *iz = IZL | ((inc(z, IZH)) << 8); break; // inc izh
  case 0x25: *iz = IZL | ((dec(z, IZH)) << 8); break; // dec izh
  case 0x2C: *iz = (IZH << 8) | inc(z, IZL); break; // inc izl
  case 0x2D: *iz = (IZH << 8) | dec(z, IZL); break; // dec izl

  case 0x2A: *iz = rw(z, nextw(z)); break; // ld iz,(**)
  case 0x22: ww(z, nextw(z), *iz); break; // ld (**),iz
  case 0x21: *iz = nextw(z); break; // ld iz,**

  case 0x36: {
    uint16_t addr = IZD;
    wb(z, addr, nextb(z));
  } break; // ld (iz+*),*

  case 0x70: wb(z, IZD, z->b); break; // ld (iz+*),b
  case 0x71: wb(z, IZD, z->c); break; // ld (iz+*),c
  case 0x72: wb(z, IZD, z->d); break; // ld (iz+*),d
  case 0x73: wb(z, IZD, z->e); break; // ld (iz+*),e
  case 0x74: wb(z, IZD, z->h); break; // ld (iz+*),h
  case 0x75: wb(z, IZD, z->l); break; // ld (iz+*),l
  case 0x77: wb(z, IZD, z->a); break; // ld (iz+*),a

  case 0x46: z->b = rb(z, IZD); break; // ld b,(iz+*)
  case 0x4E: z->c = rb(z, IZD); break; // ld c,(iz+*)
  case 0x56: z->d = rb(z, IZD); break; // ld d,(iz+*)
  case 0x5E: z->e = rb(z, IZD); break; // ld e,(iz+*)
  case 0x66: z->h = rb(z, IZD); break; // ld h,(iz+*)
  case 0x6E: z->l = rb(z, IZD); break; // ld l,(iz+*)
  case 0x7E: z->a = rb(z, IZD); break; // ld a,(iz+*)

  case 0x44: z->b = IZH; break; // ld b,izh
  case 0x4C: z->c = IZH; break; // ld c,izh
  case 0x54: z->d = IZH; break; // ld d,izh
  case 0x5C: z->e = IZH; break; // ld e,izh
  case 0x7C: z->a = IZH; break; // ld a,izh

  case 0x45: z->b = IZL; break; // ld b,izl
  case 0x4D: z->c = IZL; break; // ld c,izl
  case 0x55: z->d = IZL; break; // ld d,izl
  case 0x5D: z->e = IZL; break; // ld e,izl
  case 0x7D: z->a = IZL; break; // ld a,izl

  case 0x60: *iz = IZL | (z->b << 8); break; // ld izh,b
  case 0x61: *iz = IZL | (z->c << 8); break; // ld izh,c
  case 0x62: *iz = IZL | (z->d << 8); break; // ld izh,d
  case 0x63: *iz = IZL | (z->e << 8); break; // ld izh,e
  case 0x64: break; // ld izh,izh
  case 0x65: *iz = (IZL << 8) | IZL; break; // ld izh,izl
  case 0x67: *iz = IZL | (z->a << 8); break; // ld izh,a
  case 0x26: *iz = IZL | (nextb(z) << 8); break; // ld izh,*

  case 0x68: *iz = (IZH << 8) | z->b; break; // ld izl,b
  case 0x69: *iz = (IZH << 8) | z->c; break; // ld izl,c
  case 0x6A: *iz = (IZH << 8) | z->d; break; // ld izl,d
  case 0x6B: *iz = (IZH << 8) | z->e; break; // ld izl,e
  case 0x6C: *iz = (IZH << 8) | IZH; break; // ld izl,izh
  case 0x6D: break; // ld izl,izl
  case 0x6F: *iz = (IZH << 8) | z->a; break; // ld izl,a
  case 0x2E: *iz = (IZH << 8) | nextb(z); break; // ld izl,*

  case 0xF9: z->sp = *iz; break; // ld sp,iz

  case 0xE3: {
    const uint16_t val = rw(z, z->sp);
    ww(z, z->sp, *iz);
    *iz = val;
    z->mem_ptr = val;
  } break; // ex (sp),iz

  case 0xCB: {
    uint16_t addr = IZD;
    uint8_t op = nextb(z);
    cyc += exec_opcode_dcb(z, op, addr);
  } break;

  default: {
    // any other FD/DD opcode behaves as a non-prefixed opcode:
    cyc += exec_opcode(z, opcode);
    // R should not be incremented twice:
    z->r = (z->r & 0x80) | ((z->r - 1) & 0x7f);
  } break;
  }

#undef IZD
#undef IZH
#undef IZL
  return cyc;
}

// executes a CB opcode
unsigned exec_opcode_cb(z80* const z, uint8_t opcode) {
  unsigned cyc = 8;
  inc_r(z);

  // decoding instructions from http://z80.info/decoding.htm#cb
  uint8_t x_ = (opcode >> 6) & 3; // 0b11
  uint8_t y_ = (opcode >> 3) & 7; // 0b111
  uint8_t z_ = opcode & 7; // 0b111

  uint8_t hl = 0;
  uint8_t* reg = 0;
  switch (z_) {
  case 0: reg = &z->b; break;
  case 1: reg = &z->c; break;
  case 2: reg = &z->d; break;
  case 3: reg = &z->e; break;
  case 4: reg = &z->h; break;
  case 5: reg = &z->l; break;
  case 6:
    hl = rb(z, z->hl);
    reg = &hl;
    break;
  case 7: reg = &z->a; break;
  }

  switch (x_) {
  case 0: {
    switch (y_) {
    case 0: *reg = cb_rlc(z, *reg); break;
    case 1: *reg = cb_rrc(z, *reg); break;
    case 2: *reg = cb_rl(z, *reg); break;
    case 3: *reg = cb_rr(z, *reg); break;
    case 4: *reg = cb_sla(z, *reg); break;
    case 5: *reg = cb_sra(z, *reg); break;
    case 6: *reg = cb_sll(z, *reg); break;
    case 7: *reg = cb_srl(z, *reg); break;
    }
  } break; // rot[y] r[z]
  case 1: { // BIT y, r[z]
    cb_bit(z, *reg, y_);

    // in bit (hl), x/y flags are handled differently:
    if (z_ == 6) {
      flag_set(z, yf, GET_BIT(5, z->mem_ptr >> 8));
      flag_set(z, xf, GET_BIT(3, z->mem_ptr >> 8));
      cyc += 4;
    }
  } break;
  case 2: *reg &= ~(1 << y_); break; // RES y, r[z]
  case 3: *reg |= 1 << y_; break; // SET y, r[z]
  }

  if ((x_ != 1) && (z_ == 6)) { // BIT (HL) not included
    wb(z, z->hl, hl);
    cyc += 7;
  }
  return cyc;
}

// executes a displaced CB opcode (DDCB or FDCB)
unsigned exec_opcode_dcb(z80* const z, uint8_t opcode, uint16_t addr) {
  unsigned cyc = 0;
  uint8_t val = rb(z, addr);
  uint8_t result = 0;

  // decoding instructions from http://z80.info/decoding.htm#ddcb
  uint8_t x_ = (opcode >> 6) & 3; // 0b11
  uint8_t y_ = (opcode >> 3) & 7; // 0b111
  uint8_t z_ = opcode & 7; // 0b111

  switch (x_) {
  case 0: {
    // rot[y] (iz+d)
    switch (y_) {
    case 0: result = cb_rlc(z, val); break;
    case 1: result = cb_rrc(z, val); break;
    case 2: result = cb_rl(z, val); break;
    case 3: result = cb_rr(z, val); break;
    case 4: result = cb_sla(z, val); break;
    case 5: result = cb_sra(z, val); break;
    case 6: result = cb_sll(z, val); break;
    case 7: result = cb_srl(z, val); break;
    }
  } break;
  case 1: {
    result = cb_bit(z, val, y_);
    flag_set(z, yf, GET_BIT(5, addr >> 8));
    flag_set(z, xf, GET_BIT(3, addr >> 8));
  } break; // bit y,(iz+d)
  case 2: result = val & ~(1 << y_); break; // res y, (iz+d)
  case 3: result = val | (1 << y_); break; // set y, (iz+d)

  default: break;
  //fprintf(stderr, "unknown XYCB opcode: %02X\n", opcode); break;
  }

  // ld r[z], rot[y] (iz+d)
  // ld r[z], res y,(iz+d)
  // ld r[z], set y,(iz+d)
  if (x_ != 1 && z_ != 6) {
    switch (z_) {
    case 0: z->b = result; break;
    case 1: z->c = result; break;
    case 2: z->d = result; break;
    case 3: z->e = result; break;
    case 4: z->h = result; break;
    case 5: z->l = result; break;
    case 6: wb(z, z->hl, result); break;
    case 7: z->a = result; break;
    }
  }

  if (x_ == 1) {
    // bit instructions take 20 cycles, others take 23
    cyc += 20;
  } else {
    wb(z, addr, result);
    cyc += 23;
  }
  return cyc;
}

// executes a ED opcode
unsigned exec_opcode_ed(z80* const z, uint8_t opcode) {
  unsigned cyc = cyc_ed[opcode];
  inc_r(z);
  switch (opcode) {
  case 0x47: z->i = z->a; break; // ld i,a
  case 0x4F: z->r = z->a; break; // ld r,a

  case 0x57:
    z->a = z->i;
    flag_set(z, sf, z->a >> 7);
    flag_set(z, zf, z->a == 0);
    flag_set(z, hf, 0);
    flag_set(z, nf, 0);
    flag_set(z, pf, z->iff2);
    break; // ld a,i

  case 0x5F:
    z->a = z->r;
    flag_set(z, sf, z->a >> 7);
    flag_set(z, zf, z->a == 0);
    flag_set(z, hf, 0);
    flag_set(z, nf, 0);
    flag_set(z, pf, z->iff2);
    break; // ld a,r

  case 0x45:
  case 0x55:
  case 0x5D:
  case 0x65:
  case 0x6D:
  case 0x75:
  case 0x7D:
    z->iff1 = z->iff2;
    ret(z);
    break; // retn
  case 0x4D: ret(z); break; // reti

  case 0xA0: ldi(z); break; // ldi
  case 0xB0: {
    ldi(z);

    if (z->bc != 0) {
      z->pc -= 2;
      cyc += 5;
      z->mem_ptr = z->pc + 1;
    }
  } break; // ldir

  case 0xA8: ldd(z); break; // ldd
  case 0xB8: {
    ldd(z);

    if (z->bc != 0) {
      z->pc -= 2;
      cyc += 5;
      z->mem_ptr = z->pc + 1;
    }
  } break; // lddr

  case 0xA1: cpi(z); break; // cpi
  case 0xA9: cpd(z); break; // cpd
  case 0xB1: {
    cpi(z);
    if (z->bc != 0 && !flag_get(z, zf)) {
      z->pc -= 2;
      cyc += 5;
      z->mem_ptr = z->pc + 1;
    } else {
      z->mem_ptr += 1;
    }
  } break; // cpir
  case 0xB9: {
    cpd(z);
    if (z->bc != 0 && !flag_get(z, zf)) {
      z->pc -= 2;
      cyc += 5;
    } else {
      z->mem_ptr += 1;
    }
  } break; // cpdr

  case 0x40: in_r_c(z, &z->b); break; // in b, (c)
  case 0x48: in_r_c(z, &z->c); break; // in c, (c)
  case 0x50: in_r_c(z, &z->d); break; // in d, (c)
  case 0x58: in_r_c(z, &z->e); break; // in e, (c)
  case 0x60: in_r_c(z, &z->h); break; // in h, (c)
  case 0x68: in_r_c(z, &z->l); break; // in l, (c)
  case 0x70: {
    uint8_t val;
    in_r_c(z, &val);
  } break; // in (c)
  case 0x78:
    in_r_c(z, &z->a);
    z->mem_ptr = z->bc + 1;
    break; // in a, (c)

  case 0xA2: ini(z); break; // ini
  case 0xB2:
    ini(z);
    if (z->b > 0) {
      z->pc -= 2;
      cyc += 5;
    }
    break; // inir
  case 0xAA: ind(z); break; // ind
  case 0xBA:
    ind(z);
    if (z->b > 0) {
      z->pc -= 2;
      cyc += 5;
    }
    break; // indr

  case 0x41: z->port_out(z, z->c, z->b); break; // out (c), b
  case 0x49: z->port_out(z, z->c, z->c); break; // out (c), c
  case 0x51: z->port_out(z, z->c, z->d); break; // out (c), d
  case 0x59: z->port_out(z, z->c, z->e); break; // out (c), e
  case 0x61: z->port_out(z, z->c, z->h); break; // out (c), h
  case 0x69: z->port_out(z, z->c, z->l); break; // out (c), l
  case 0x71: z->port_out(z, z->c, 0); break; // out (c), 0
  case 0x79:
    z->port_out(z, z->c, z->a);
    z->mem_ptr = z->bc + 1;
    break; // out (c), a

  case 0xA3: outi(z); break; // outi
  case 0xB3: {
    outi(z);
    if (z->b > 0) {
      z->pc -= 2;
      cyc += 5;
    }
  } break; // otir
  case 0xAB: outd(z); break; // outd
  case 0xBB: {
    outd(z);
    if (z->b > 0) {
      z->pc -= 2;
      cyc += 5;
    }
  } break; // otdr

  case 0x42: sbchl(z, z->bc); break; // sbc hl,bc
  case 0x52: sbchl(z, z->de); break; // sbc hl,de
  case 0x62: sbchl(z, z->hl); break; // sbc hl,hl
  case 0x72: sbchl(z, z->sp); break; // sbc hl,sp

  case 0x4A: adchl(z, z->bc); break; // adc hl,bc
  case 0x5A: adchl(z, z->de); break; // adc hl,de
  case 0x6A: adchl(z, z->hl); break; // adc hl,hl
  case 0x7A: adchl(z, z->sp); break; // adc hl,sp

  case 0x43: {
    const uint16_t addr = nextw(z);
    ww(z, addr, z->bc);
    z->mem_ptr = addr + 1;
  } break; // ld (**), bc

  case 0x53: {
    const uint16_t addr = nextw(z);
    ww(z, addr, z->de);
    z->mem_ptr = addr + 1;
  } break; // ld (**), de

  case 0x63: {
    const uint16_t addr = nextw(z);
    ww(z, addr, z->hl);
    z->mem_ptr = addr + 1;
  } break; // ld (**), hl

  case 0x73: {
    const uint16_t addr = nextw(z);
    ww(z, addr, z->sp);
    z->mem_ptr = addr + 1;
  } break; // ld (**),sp

  case 0x4B: {
    const uint16_t addr = nextw(z);
    z->bc = rw(z, addr);
    z->mem_ptr = addr + 1;
  } break; // ld bc, (**)

  case 0x5B: {
    const uint16_t addr = nextw(z);
    z->de = rw(z, addr);
    z->mem_ptr = addr + 1;
  } break; // ld de, (**)

  case 0x6B: {
    const uint16_t addr = nextw(z);
    z->hl = rw(z, addr);
    z->mem_ptr = addr + 1;
  } break; // ld hl, (**)

  case 0x7B: {
    const uint16_t addr = nextw(z);
    z->sp = rw(z, addr);
    z->mem_ptr = addr + 1;
  } break; // ld sp,(**)

  case 0x44:
  case 0x54:
  case 0x64:
  case 0x74:
  case 0x4C:
  case 0x5C:
  case 0x6C:
  case 0x7C: z->a = subb(z, 0, z->a, 0); break; // neg

  case 0x46:
  case 0x66: z->interrupt_mode = 0; break; // im 0
  case 0x56:
  case 0x76: z->interrupt_mode = 1; break; // im 1
  case 0x5E:
  case 0x7E: z->interrupt_mode = 2; break; // im 2

  case 0x67: {
    uint8_t a = z->a;
    uint8_t val = rb(z, z->hl);
    z->a = (a & 0xF0) | (val & 0xF);
    wb(z, z->hl, (val >> 4) | (a << 4));
    z->f = 0 |
      flag_val(cf, flag_get(z, cf)) | /* cf unmodified */
      flag_val(nf, 0) |
      flag_val(hf, 0) |
      flag_val(xf, GET_BIT(3, z->a)) |
      flag_val(yf, GET_BIT(5, z->a)) |
      flag_val(zf, z->a == 0) |
      flag_val(sf, z->a >> 7) |
      flag_val(pf, parity(z->a));
    z->mem_ptr = z->hl + 1;
  } break; // rrd

  case 0x6F: {
    uint8_t a = z->a;
    uint8_t val = rb(z, z->hl);
    z->a = (a & 0xF0) | (val >> 4);
    wb(z, z->hl, (val << 4) | (a & 0xF));

    z->f = 0 |
      flag_val(cf, flag_get(z, cf)) | /* cf unmodified */
      flag_val(nf, 0) |
      flag_val(hf, 0) |
      flag_val(xf, GET_BIT(3, z->a)) |
      flag_val(yf, GET_BIT(5, z->a)) |
      flag_val(zf, z->a == 0) |
      flag_val(sf, z->a >> 7) |
      flag_val(pf, parity(z->a));
    z->mem_ptr = z->hl + 1;
  } break; // rld

  default: break;
  //fprintf(stderr, "unknown ED opcode: %02X\n", opcode); break;
  }
  return cyc;
}

#undef GET_BIT
