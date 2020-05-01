#!/usr/bin/env python

# "what can JP instructions look like?"
# ./enum65536.py | grep jp | sort | unique
# jp (hl); jp (ix); jp (iy); jp <hex>; jp <cc>,<hex>

# "what can JR instructions look like?
# ./enum65536.py | grep jp | sort | unique
# jr <hex>; jr <cc>,<hex>

from z80dis import z80
from struct import pack
from binascii import hexlify

ADDR = 0xDEAD

for i in range(65536):
    data = pack('>H', i) + b'\xAB\xCD\xEF\x00'
    decoded = z80.decode(data, ADDR)

    hexstr = hexlify(data[0:decoded.len]).decode('utf-8')
    disasm = z80.disasm(decoded)
    print('%s %04X: %s' % (disasm.ljust(16), ADDR, hexstr))
