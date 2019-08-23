#!/usr/bin/env python

from z80dis import z80
from struct import pack
from binascii import hexlify

ADDR = 0xDEAD

for i in range(65536):
	data = pack('>H', i) + b'\xAB\xCD\xEF\x00'
	decoded = z80.decode(data, ADDR)

	hexstr = hexlify(data[0:decoded.len]).decode('utf-8')
	disasm = z80.disasm(decoded)
	print('%04X: %s %s' % (ADDR, hexstr.ljust(8), disasm))
