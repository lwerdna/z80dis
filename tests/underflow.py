#!/usr/bin/env python

# what happens

from z80dis import z80
from struct import pack
from binascii import hexlify

ADDR = 0xDEAD

def doit(data):
    decoded = z80.decode(data, ADDR)
    hexstr = hexlify(data[0:decoded.len]).decode('utf-8')
    disasm = z80.disasm(decoded)
    print('%04X: %s %s' % (ADDR, hexstr, disasm))

print('1-byte')
for i in range(256):
    data = i.to_bytes(1, 'big')
    doit(data)

print('2-byte')
for i in range(65536):
    data = pack('>H', i)
    doit(data)

print('3-byte')
for i in range(65536):
    data = pack('>H', i) + b'\x00'
    doit(data)

print('4-byte')
for i in range(65536):
    data = pack('>H', i) + b'\x00\x00'
    doit(data)

print('PASS')
