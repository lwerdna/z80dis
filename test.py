#!/usr/bin/env python

import re
import sys
import struct
import ctypes
import random

from z80dis import z80

RED = '\x1B[31m'
NORMAL = '\x1B[0m'
GREEN = '\x1B[32m'
YELLOW = '\x1B[33m'

gofer = None
cbuf = None
ibuf = None
def disasm_libopcodes(data, addr):
    # initialize disassembler, if necessary
    global gofer, cbuf, ibuf
    if not gofer:
        gofer = ctypes.CDLL("gofer.so")
        cbuf = ctypes.create_string_buffer(256)
        ibuf = ctypes.c_int(0)
    gofer.get_disasm_libopcodes(addr, data, 4, ctypes.byref(cbuf), ctypes.byref(ibuf))
    return [cbuf.value.decode('utf-8'), ibuf.value]

def disasm_z80ex(data, addr):
    # initialize disassembler, if necessary
    global gofer, cbuf, ibuf
    if not gofer:
        gofer = ctypes.CDLL("gofer.so")
        cbuf = ctypes.create_string_buffer(256)
        ibuf = ctypes.c_int(0)
    gofer.get_disasm_z80ex(addr, data, 4, ctypes.byref(cbuf), ctypes.byref(ibuf))
    return [cbuf.value.decode('utf-8'), ibuf.value]

def xyz(byte):
    x = byte >> 6
    y = (byte >> 3) & 7
    z = byte & 7
    p = y >> 1
    q = y & 1
    return (x,y,z,p,q)

def xyz_print(byte):
    (x,y,z,p,q) = xyz(byte)
    print('(x,y,z,p,q)=(%d,%d,%d,%d,%d)' % (x,y,z,p,q))

def normalize(distxt):
    tokens = []

    # libopcode's version of error
    if distxt.startswith('defb '):
        return 'ERROR'
    # z80ex's version of error
    if distxt == 'NOP*':
        return 'ERROR'
    # our version of error
    if distxt == '':
        return 'ERROR'

    distxt = distxt.lower()

    tmp = distxt.split(' ', 1)
    opcode = tmp[0]
    tokens.append(('OPC', opcode))

    rest = tmp[1] if tmp[1:] else ''
    while rest:
        # eat whitespace
        m = re.match(r'^\s+', rest)
        if m:
            rest = rest[len(m.group(0)):]
            continue

        # ix-0x80 -> ix+0xFFFFFF80
        m = re.match(r'^(i[xy])-(0x[a-fA-F0-9]+)\)', rest)
        if m:
            tokens.append(('STRING', m.group(1))) # ix or iy
            tokens.append(('PUNC', '+'))

            val = int(m.group(2), 16)
            tokens.append(('QUANTITY', (val ^ 0xFFFFFFFF)+1))
            rest = rest[len(m.group(0))-1:]
            continue

        # ix-15 -> ix+0xFFFFFFF1
        m = re.match(r'^(i[xy])-(\d+)\)', rest)
        if m:
            tokens.append(('STRING', m.group(1))) # ix or iy
            tokens.append(('PUNC', '+'))

            val = int(m.group(2))
            tokens.append(('QUANTITY', (val ^ 0xFFFFFFFF)+1))
            rest = rest[len(m.group(0))-1:]
            continue

        m = re.match(r'^(i[xy])\+0x0+\)', rest)
        if m:
            tokens.append(('STRING', m.group(1)))
            rest = rest[len(m.group(0))-1:]
            continue

        m = re.match(r'^(i[xy])\+#0+\)', rest)
        if m:
            tokens.append(('STRING', m.group(1)))
            rest = rest[len(m.group(0))-1:]
            continue

        m = re.match(r'^#([a-fA-F0-9]+)', rest)
        if m:
            val = int(m.group(1),16)
            tokens.append(('QUANTITY', val))
            rest = rest[len(m.group(0)):]
            continue

        m = re.match(r'^0x([a-fA-F0-9]+)', rest)
        if m:
            val = int(m.group(1),16)
            tokens.append(('QUANTITY', val))
            rest = rest[len(m.group(0)):]
            continue

        m = re.match(r'^([0-9]+)', rest)
        if m:
            val = int(m.group(1),10)
            tokens.append(('QUANTITY', val))
            rest = rest[len(m.group(0)):]
            continue            

        m = re.match(r'^(\w+)', rest)
        if m:
            tokens.append(('STRING', m.group(1)))
            rest = rest[len(m.group(0)):]
            continue

        if rest[0] in list(',()\'+-'):
            tokens.append(('PUNC', rest[0]))
            rest = rest[1:]
            continue

        raise Exception('dunno how to tokenize: ...%s (original input: %s)' % (rest, distxt))

    # tokens done, return result
    result = ''
    for (typ,val) in tokens:
        if typ in ['OPC']:
            result += val
            if tokens[1:]:
                result += ' '
        elif typ in ['OPC', 'PUNC', 'STRING']:
            result += val
        elif typ == 'QUANTITY':
            result += '0x%X' % val
        else:
            raise Exception('dunno how to convert token type: %s' % typ)

    return result

def differential_disassemble(data, addr):
    if data[0] in [0xCD, 0xED]:
        (x,y,z,p,q) = xyz(data[1])
    else:
        (x,y,z,p,q) = xyz(data[0])

    dc = z80.decode(data, addr)
    a = z80.decoded2str(dc)
    alen = dc.len
    [b, blen] = disasm_libopcodes(data, addr)
    [c, clen] = disasm_z80ex(data, addr)

    # comparison versions
    a_ = normalize(a)
    b_ = normalize(b)
    c_ = normalize(c)

    hexstr = ' '.join(map(lambda x: '%02X'%x, data))
    print('(%d,%d,%d) %04X: %s -' % \
        (alen, blen, clen, addr, hexstr), end='')

    # PASSED, we agree with one of the two oracles
    if a_ == b_ or a_ == c_:
        print(GREEN+a+NORMAL+'-    -'+b+'-    -'+c+'-')
    # PASS, we nop and z80ex errors
    elif a_ == 'nop' and c_ == 'ERROR':
        print(a+'-    -'+b+'-    -'+c+'-'+YELLOW+' PASS'+NORMAL)
    # FAIL
    else:
        print(RED+a+NORMAL+'-    -'+b+'-    -'+c+'-')
        print('comparator a: -%s-' % a_)
        print('comparator b: -%s-' % b_)
        print('comparator c: -%s-' % c_)
        xyz_print(data[0])
        sys.exit(-1)

    # COMPARE LENGTH
    if alen != clen:
        if data[0:2] in [b'\xDD\xCB', b'\xFD\xCB'] and alen == clen-1:
            print(YELLOW + 'length mismatch, our %d vs. z80ex\'s %d is OK for DDCB/FDCB' % (alen, clen) + NORMAL)
        else: 
            print(RED + 'length mismatch, our %d vs. z80ex\'s %d' % (alen, clen) + NORMAL)
            sys.exit(-1)

if __name__ == '__main__':
    if sys.argv[1:] and sys.argv[1] == 'CB':
        for insword in range(65536):
            data = b'\xCB' + struct.pack('<H', insword) + b'\xAB\xCD'
            differential_disassemble(data, 0)

    elif sys.argv[1:] and sys.argv[1] == 'ED':
        for insword in range(65536):
            data = b'\xED' + struct.pack('<H', insword) + b'\xAB\xCD'
            differential_disassemble(data, 0)

    # all DD prefixes, except DDCB which is another prefix
    elif sys.argv[1:] and sys.argv[1] == 'DD':
        for insword in range(65536):
            data = b'\xDD' + struct.pack('<H', insword) + b'\xAB\xCD'
            if data[1] == 0xCB:
                continue
            differential_disassemble(data, 0)

    # all FD prefixes, except FDCB which is another prefix
    elif sys.argv[1:] and sys.argv[1] == 'FD':
        for insword in range(65536):
            data = b'\xFD' + struct.pack('<H', insword) + b'\xAB\xCD'
            if data[1] == 0xCB:
                continue
            differential_disassemble(data, 0)

    # all DDCB prefixes
    elif sys.argv[1:] and sys.argv[1] == 'DDCB':
        for insword in range(65536):
            data = b'\xDD\xCB' + struct.pack('<H', insword) + b'\xAB\xCD'
            differential_disassemble(data, 0)

    # all FDCB prefixes
    elif sys.argv[1:] and sys.argv[1] == 'FDCB':
        for insword in range(65536):
            data = b'\xFD\xCB' + struct.pack('<H', insword) + b'\xAB\xCD'
            differential_disassemble(data, 0)

    # explicitly written 4 bytes
    elif sys.argv[1:]:
        insword = int(sys.argv[1],16)
        data = struct.pack('>I', insword) + b'\xAA\xBB'
        xyz_print(data[0])
        print('libopcodes: %s' % disasm_libopcodes(data, 0))
        print('     z80ex: %s' % disasm_z80ex(data, 0))
        differential_disassemble(data, 0)
    
    # all non-prefixed opcodes
    else:
       for insword in range(65536):
            data = struct.pack('>H', insword) + b'\xAB\xCD\xEF\x00' #struct.pack('<H', random.randint(65535))
            differential_disassemble(data, 0xDEAD)
