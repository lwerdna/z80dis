#!/usr/bin/env python

# See accompanying README for usage.
# You probably want `decode()` and `disassemble()`.

# The decoding structure is based on http://www.z80.info/decoding.htm
# with modifications to match libopcodes or z80ex.

import sys
import struct
from enum import Enum, auto, unique

#------------------------------------------------------------------------------
# ENUMERATIONS
#------------------------------------------------------------------------------

@unique
class DECODE_STATUS(Enum):
    OK = 0
    INVALID_INSTRUCTION = auto()
    ERROR = auto()

@unique
class INSTRTYPE(Enum):
    NOP = 0
    LOAD_EXCHANGE = auto()
    BLOCK_TRANSFER_SEARCH = auto()
    ARITHMETIC_LOGICAL = auto()
    ROTATE_SHIFT = auto()
    BIT_MANIPULATION = auto()
    JUMP_CALL_RETURN = auto()
    INPUT_OUTPUT = auto()
    CPU_CONTROL = auto()

@unique
class OPER_TYPE(Enum):
    NONE = 0
    # main register set
    # 8 bit registers individually
    REG_A = auto()
    REG_F = auto()
    REG_B = auto()
    REG_C = auto()
    REG_D = auto()
    REG_E = auto()
    REG_H = auto()
    REG_L = auto()
    REG_C_DEREF = auto()
    # 8 bit registers in 16-bit groups
    REG_AF = auto()
    REG_BC = auto()
    REG_DE = auto()
    REG_HL = auto()
    # deref'd: (BC), (DE), (HL)
    REG_BC_DEREF = auto() # (BC)
    REG_DE_DEREF = auto() # (DE)
    REG_HL_DEREF = auto() # (HL)
    # alternate register set
    REG_A_ = auto()
    REG_F_ = auto()
    REG_B_ = auto()
    REG_C_ = auto()
    REG_D_ = auto()
    REG_E_ = auto()
    REG_H_ = auto()
    REG_L_ = auto()
    # 8 bit registers in 16-bit groups
    REG_AF_ = auto()
    REG_BC_ = auto()
    REG_DE_ = auto()
    REG_HL_ = auto()
    # special purpose registers
    REG_I = auto()
    REG_R = auto()
    REG_IX = auto()
    REG_IXH = auto()    # IX hi
    REG_IXL = auto()    # IX lo
    REG_IY = auto()
    REG_IYH = auto()    # IY hi
    REG_IYL = auto()    # IY lo
    REG_SP = auto()
    REG_PC = auto()
    # deref'd
    REG_SP_DEREF = auto()
    ADDR = auto()
    ADDR_DEREF = auto()
    MEM_DISPL_IX = auto()
    MEM_DISPL_IY = auto()
    #
    # when the source is an embedded instruction:
    # LD A = auto() RES 7 = auto() (IX+5)
    # "attempt reset of bit 7 at (IX+5) = auto() store result in A"
    SUB_INSTR = auto()
    #
    IMM = auto()
    CON = auto()
    OPER_TYPE_LAST_I = auto()

@unique
class OP(Enum):
    NOP = 0
    NONI = auto()
    # LOAD_EXCHANGE
    LD = auto()
    EX = auto()
    EXX = auto()
    POP = auto()
    PUSH = auto()
    MARKER_END_LOAD_EXCHANGE = auto()
    # BLOCK_TRANSFER_SEARCH
    LDI = auto()                # load and increment ([DE++] <- [HL++]; BC--;)
    CPI = auto()                # compare and increment
    INI = auto()                # in and increment ([HL++] <- port[C]; B--
    OUTI = auto()                # out and increment
    LDD = auto()                # as above = auto() but decrement DE = auto() HL
    CPD = auto()
    IND = auto()
    OUTD = auto()
    LDIR = auto()                # as above = auto() but repeated while BC != 0
    CPIR = auto()
    INIR = auto()
    OTIR = auto()
    LDDR = auto()
    CPDR = auto()
    INDR = auto()
    OTDR = auto()
    MARKER_END_BLOCK_TRANSFER_SEARCH = auto()
    # ARITHMETIC_LOGICAL
    ADD = auto()
    ADC = auto()
    SUB = auto()
    SBC = auto()
    AND = auto()
    XOR = auto()
    OR = auto()
    CP = auto()
    RLC = auto()
    RRC = auto()
    RL = auto()
    RR = auto()
    SLA = auto()
    SRA = auto()
    SLL = auto()
    SRL = auto()
    INC = auto()
    DEC = auto()
    NEG = auto()

    DAA = auto()				# decimal adjust A
    CPL = auto()				# complement A
    SCF = auto()				# set carry flag
    CCF = auto()				# clear carry flag
    MARKER_END_ARITHMETIC_LOGICAL = auto()
    # ROTATE_SHIFT
    RLCA = auto()				# rotate left *TO* carry A
    RRCA = auto()				# right
    RLA = auto()				# rotate left *THRU* carry A
    RRA = auto()				# right
    RRD = auto()
    RLD = auto()
    MARKER_END_ROTATE_SHIFT = auto()
    # BIT_MANIPULATION
    BIT = auto()				# bit test
    RES = auto()				# bit reset
    SET = auto()				# bit set
    MARKER_END_BIT_MANIPULATION = auto()
    # JUMP_CALL_RETURN
    CALL = auto()
    RET = auto()
    JP = auto()					# jump
    JR = auto()					# jump relative
    DJNZ = auto()				# decrement = auto() jump if not zero
    RETI = auto()				# return from interrupt
    RETN = auto()				# return from NMI
    MARKER_END_JUMP_CALL_RETURN = auto()
    # INPUT_OUTPUT
    IN = auto()
    OUT = auto()
    MARKER_END_INPUT_OUTPUT = auto()
    # CPU_CONTROL
    HALT = auto()
    DI = auto()				# disable interrupts
    EI = auto()				# enable interrupts
    IM = auto()				# set interrupt mode
    RST = auto()			# reset
    MARKER_END_CPU_CONTROL = auto()
    Z80_LAST_I = auto()

@unique
class FLAGS(Enum):
    FLAG_C = 1				# carry flag (add/adc, sub/sbb, rla/rra/rls/rrs, rlca/rlc,sla, AND,OR,XOR resets)!
    FLAG_N = 2				# cleared after ADD, set after SUB (used by DAA)
    FLAG_PV = 4				# parity/overflow flag
    FLAG_F3 = 8
    FLAG_H = 16				# half-carry flag (if carry from bit 3 to 4)
    FLAG_F5 = 32
    FLAG_Z = 64				# zero flag
    FLAG_S = 128			# sign flag (msb of accumulator)

@unique
class CC(Enum):
    ALWAYS = 0
    C = auto()				# carry
    NOT_C = auto()
    N = auto()				# add/sub flag
    NOT_N = auto()
    P = auto()				# P=1 PARITY (even)
    NOT_P = auto()			# P=0 NOT PARITY (odd)
    H = auto()				# half carry
    NOT_H = auto()
    Z = auto()				# zero
    NOT_Z = auto()
    S = auto()				# sign (just msb of A)
    NOT_S = auto()
    Z80_CC_LAST_I = auto()

@unique
class PREFIX(Enum):
    NONE = 0
    CB = auto()
    ED = auto()
    DD = auto()
    FD = auto()
    DDCB = auto()
    FDCB = auto()

# ------------------------------------------------------------
# structs
# ------------------------------------------------------------
class Decoded():
    def __init__(self):
        self.status = DECODE_STATUS.ERROR

        self.len = 0
        # instruction type
        self.typ = None				# Z80_INSTRTYPE
        self.op = None				# Z80_OP
        # list of (OPER_TYPE, VALUE)
        self.operands = []
        # whether the entire instruction's result gets written to a reg
        # (special case for DDCB, FDCB)
        self.metaLoad = OPER_TYPE.NONE

    def __str__(self):
        result = ''

        result += '%s\n' % str(self.op)
        for i in range(len(self.operands)):
            result += '.operands[%d] = %s' % (i, self.operands[i])
            if i < len(self.operands)-1:
                result += '\n'

        return result

#------------------------------------------------------------------------------
# DECODING
#------------------------------------------------------------------------------

# tables
# (from http:#z80.info/decoding.htm)

TABLE_R = [
    OPER_TYPE.REG_B,
    OPER_TYPE.REG_C,
    OPER_TYPE.REG_D,
    OPER_TYPE.REG_E,
    OPER_TYPE.REG_H,
    OPER_TYPE.REG_L,
    OPER_TYPE.REG_HL_DEREF,
    OPER_TYPE.REG_A
]

# register pairs featuring SP
TABLE_RP = [
    OPER_TYPE.REG_BC,
    OPER_TYPE.REG_DE,
    OPER_TYPE.REG_HL,
    OPER_TYPE.REG_SP
]

# register pairs featuring AF
TABLE_RP2 = [
    OPER_TYPE.REG_BC,
    OPER_TYPE.REG_DE,
    OPER_TYPE.REG_HL,
    OPER_TYPE.REG_AF
]
# condition codes
# see cc table in ZiLOG manual for RET cc
TABLE_CC = [
    CC.NOT_Z,			# NZ
    CC.Z,				# Z
    CC.NOT_C,			# NC
    CC.C,				# C
    CC.NOT_P,			# PO (parity odd == no pariy
    CC.P,				# PE (parity even == parity
    CC.NOT_S,			# P sign positive
    CC.S				# M sign negative
]
# arithm/logic
TABLE_ALU_OP = [
    OP.ADD, OP.ADC, OP.SUB, OP.SBC, OP.AND, OP.XOR, OP.OR, OP.CP
]
# rotate
TABLE_ROT = [
    OP.RLC, OP.RRC, OP.RL, OP.RR, OP.SLA, OP.SRA, OP.SLL, OP.SRL
]
# interrupt modes
TABLE_IM = [
    0, 0, 1, 2, 0, 0, 1, 2
]
TABLE_BLI = [
    [OP.LDI, OP.CPI, OP.INI, OP.OUTI],
    [OP.LDD, OP.CPD, OP.IND, OP.OUTD],
    [OP.LDIR, OP.CPIR, OP.INIR, OP.OTIR],
    [OP.LDDR, OP.CPDR, OP.INDR, OP.OTDR]
]
TABLE_ASSORTED = [
    OP.RLCA, OP.RRCA, OP.RLA, OP.RRA, OP.DAA, OP.CPL, OP.SCF, OP.CCF
]

#------------------------------------------------------------------------------
# DECODING
#------------------------------------------------------------------------------

def int8(b):
    if b & 0x80:
        return -((b ^ 255) + 1)
    else:
        return b

def uint16(b0,b1):
    return (b1<<8)|b0

def reorder(dc):
    if len(dc.operands) == 2:
        (dc.operands[0], dc.operands[1]) = (dc.operands[1], dc.operands[0])

def xyz(byte):
    x = byte >> 6
    y = (byte >> 3) & 7
    z = byte & 7
    p = y >> 1
    q = y & 1
    return (x,y,z,p,q)

def will_deref_hl(opc):
    (x,y,z,p,q) = xyz(opc)
    # INC r[y], DEC r[y], LD r[y],n and TABLE_R[6] == '(HL)'
    if x == 0 and (z in [4,5,6]) and y == 6:
        return True
    # LD r[y],r[z] and TABLE_R[6] == '(HL)'
    if x == 1 and not (z == 6 and y == 6) and (y == 6 or z == 6):
        return True
    # alu[y] r[z] and TABLE_R[6] == '(HL)'
    if x == 2 and z == 6:
        return True
    # jp (HL)
    if x == 3 and z == 1 and q == 1 and p == 2:
        return True
    # else
    return False

def decode_unprefixed(data, addr, result):
    (x,y,z,p,q) = xyz(data[0])
    result.len += 1
    if x == 0:
        # relative jumps and assorted ops
        if z == 0:
            if y == 0:
                result.op = OP.NOP
            elif y == 1:
                result.op = OP.EX
                result.operands.append((OPER_TYPE.REG_AF,))
                result.operands.append((OPER_TYPE.REG_AF_,))
            elif y == 2:
                result.op = OP.DJNZ
                result.operands.append((OPER_TYPE.ADDR, addr + 2 + int8(data[1])))
                result.len += 1
            else: # 3,4,5,6,7
                result.op = OP.JR
                result.operands.append((OPER_TYPE.CON, TABLE_CC[y-4]))
                result.operands.append((OPER_TYPE.ADDR, addr + 2 + int8(data[1])))
                result.len += 1
                if y == 3:
                    result.operands = [result.operands[1]]

        elif z == 1:
            # 16-bit load immediate
            if q == 0:
                result.op = OP.LD
                result.operands.append((TABLE_RP[p],))
                result.operands.append((OPER_TYPE.IMM, uint16(data[1], data[2])))
                result.len += 2
            elif q == 1:
                result.op = OP.ADD
                result.operands.append((OPER_TYPE.REG_HL,))
                result.operands.append((TABLE_RP[p],))

        elif z == 2:
            result.op = OP.LD
            if p == 0:
                result.operands.append((OPER_TYPE.REG_BC_DEREF,))
                result.operands.append((OPER_TYPE.REG_A,))
            elif p == 1:
                result.operands.append((OPER_TYPE.REG_DE_DEREF,))
                result.operands.append((OPER_TYPE.REG_A,))
            elif p == 2 or p == 3:
                result.operands.append((OPER_TYPE.ADDR_DEREF, uint16(data[1], data[2])))
                result.len += 2
                result.operands.append((OPER_TYPE.REG_HL if p == 2 else OPER_TYPE.REG_A,))

            if q:
                reorder(result)

        elif z == 3:
            if q:
                result.op = OP.DEC
            else:
                result.op = OP.INC
            result.operands.append((TABLE_RP[p],))

        elif z == 4:
            result.op = OP.INC
            result.operands.append((TABLE_R[y],))

        elif z == 5:
            result.op = OP.DEC
            result.operands.append((TABLE_R[y],))

        elif z == 6:
            result.op = OP.LD
            result.operands.append((TABLE_R[y],))
            result.operands.append((OPER_TYPE.IMM, data[1]))
            result.len += 1

        elif z == 7:
            result.op = TABLE_ASSORTED[y]

    elif x == 1:
        # exception! not a load!
        if z == 6 and y == 6:
            result.op = OP.HALT
        # 8-bit loading
        else:
            result.op = OP.LD
            result.operands.append((TABLE_R[y],))
            result.operands.append((TABLE_R[z],))

    elif x == 2:
        result.op = TABLE_ALU_OP[y]
        result.operands.append((OPER_TYPE.REG_A,))
        result.operands.append((TABLE_R[z],))
        if result.op in [OP.SUB, OP.AND, OP.XOR, OP.OR, OP.CP]:
            result.operands = [result.operands[1]]

    elif x == 3:
        # conditional return
        # no prefix, x==3, z==0
        if z == 0:
            result.op = OP.RET
            result.operands.append((OPER_TYPE.CON, TABLE_CC[y]))

        # pop and various ops
        # no prefix, x==3, z==1
        elif z == 1:
            if q:
                if p == 0:
                    result.op = OP.RET
                elif p == 1:
                    result.op = OP.EXX
                elif p == 2:
                    result.op = OP.JP
                    result.operands.append((OPER_TYPE.REG_HL_DEREF,))
                elif p == 3:
                    result.op = OP.LD
                    result.operands.append((OPER_TYPE.REG_SP,))
                    result.operands.append((OPER_TYPE.REG_HL,))

            else:
                result.op = OP.POP
                result.operands.append((TABLE_RP2[p],))

        # no prefix, x==3, z==2
        elif z == 2:
            result.op = OP.JP
            result.operands.append((OPER_TYPE.CON, TABLE_CC[y]))
            result.operands.append((OPER_TYPE.ADDR, uint16(data[1], data[2])))
            result.len += 2

        # no prefix, x==3, z==3
        elif z == 3:
            if y == 0:
                result.op = OP.JP
                result.operands.append((OPER_TYPE.ADDR, uint16(data[1], data[2])))
                result.len += 2
            elif y == 1:
                pass
            elif y == 2:
                result.op = OP.OUT
                result.operands.append((OPER_TYPE.ADDR_DEREF, data[1]))
                result.len += 1
                result.operands.append((OPER_TYPE.REG_A,))
            elif y == 3:
                result.op = OP.IN
                result.operands.append((OPER_TYPE.REG_A,))
                result.operands.append((OPER_TYPE.ADDR_DEREF, data[1]))
                result.len += 1
            elif y == 4:
                result.op = OP.EX
                result.operands.append((OPER_TYPE.REG_SP_DEREF,))
                result.operands.append((OPER_TYPE.REG_HL,))
            elif y == 5:
                result.op = OP.EX
                result.operands.append((OPER_TYPE.REG_DE,))
                result.operands.append((OPER_TYPE.REG_HL,))
            elif y == 6:
                result.op = OP.DI
            elif y == 7:
                result.op = OP.EI

        elif z == 4:
            result.op = OP.CALL
            result.operands.append((OPER_TYPE.CON, TABLE_CC[y]))
            result.operands.append((OPER_TYPE.ADDR, uint16(data[1], data[2])))
            result.len += 2
        elif z == 5:
            if q:
                if p:
                    # fuckup! this is DD, ED, or FD prefix!!
                    pass
                else:
                    result.op = OP.CALL
                    result.operands.append((OPER_TYPE.ADDR, uint16(data[1], data[2])))
                    result.len += 2

            else:
                result.op = OP.PUSH
                result.operands.append((TABLE_RP2[p],))

        elif z == 6:
            # accumulator and immediate
            result.op = TABLE_ALU_OP[y]
            result.operands.append((OPER_TYPE.REG_A,))
            result.operands.append((OPER_TYPE.IMM, data[1]))
            result.len += 1
            if result.op in [OP.SUB, OP.AND, OP.XOR, OP.OR, OP.CP]:
                result.operands = [result.operands[1]]

        elif z == 7:
            result.op = OP.RST
            result.operands.append((OPER_TYPE.IMM, 8*y))

def decode_cb(data, addr, result):
    (x,y,z,p,q) = xyz(data[0])
    result.len += 1
    if x:
        if x == 1:
            result.op = OP.BIT
        elif x == 2:
            result.op = OP.RES
        elif x == 3:
            result.op = OP.SET

        result.operands.append((OPER_TYPE.IMM, y))
        result.operands.append((TABLE_R[z],))
    else:
        result.op = TABLE_ROT[y]
        result.operands.append((TABLE_R[z],))

def decode_ed(data, addr, result):
    (x,y,z,p,q) = xyz(data[0])
    result.len += 1
    if x == 0 or x == 3:
        result.op = OP.NONI
    elif x == 1:
        if z == 0:
            result.op = OP.IN
            if y != 6:
                result.operands.append((TABLE_R[y],))
            else:
                result.operands.append((OPER_TYPE.REG_F,))
            result.operands.append((OPER_TYPE.REG_C_DEREF,))

        elif z == 1:
            result.op = OP.OUT
            result.operands.append((OPER_TYPE.REG_C_DEREF,))
            if y != 6:
                result.operands.append((TABLE_R[y],))
            else:
                result.operands.append((OPER_TYPE.IMM, 0))

        elif z == 2:
            result.op = OP.SBC
            result.operands.append((OPER_TYPE.REG_HL,))
            result.operands.append((TABLE_RP[p],))
            if q:
                result.op = OP.ADC

        elif z == 3:
            result.op = OP.LD
            result.operands.append((OPER_TYPE.ADDR_DEREF, uint16(data[1], data[2])))
            result.len += 2
            result.operands.append((TABLE_RP[p],))
            if q:
                reorder(result)

        elif z == 4:
            result.op = OP.NEG

        elif z == 5:
            if y & 1:
                result.op = OP.RETI
            else:
                result.op = OP.RETN

        elif z == 6:
            result.op = OP.IM
            result.operands.append((OPER_TYPE.IMM, TABLE_IM[y]))

        elif z == 7:
            if y == 0:
                result.op = OP.LD
                result.operands.append((OPER_TYPE.REG_I,))
                result.operands.append((OPER_TYPE.REG_A,))
            elif y == 1:
                result.op = OP.LD
                result.operands.append((OPER_TYPE.REG_R,))
                result.operands.append((OPER_TYPE.REG_A,))
            elif y == 2:
                result.op = OP.LD
                result.operands.append((OPER_TYPE.REG_A,))
                result.operands.append((OPER_TYPE.REG_I,))
            elif y == 3:
                result.op = OP.LD
                result.operands.append((OPER_TYPE.REG_A,))
                result.operands.append((OPER_TYPE.REG_R,))
            elif y == 4:
                result.op = OP.RRD
            elif y == 5:
                result.op = OP.RLD
            else:
                result.op = OP.NOP

    elif x == 2:
        if z <= 3 and y >= 4:
            result.op = TABLE_BLI[y-4][z]
        else:
            result.op = OP.NONI

def decode(data, addr):
    result = Decoded()

    # prefix determination
    prefix = PREFIX.NONE
    if data[0] == 0xCB:
        prefix = PREFIX.CB
        data = data[1:]
        result.len = 1
    elif data[0] == 0xED:
        prefix = PREFIX.ED
        data = data[1:]
        result.len = 1
    elif data[0] in [0xDD, 0xFD]:
        if data[1] in [0xDD, 0xED, 0xFD]:
            result.len = 1
            result.opc = OP.NOP
            return result
        if data[1] == 0xCB:
            prefix = PREFIX.DDCB if data[0] == 0xDD else PREFIX.FDCB
            data = data[2:]
            result.len = 2
        else:
            result.len = 1
            prefix = PREFIX.DD if data[0] == 0xDD else PREFIX.FD
            data = data[1:]
            result.len = 1

    # 2. UNPREFIXED OPCODES
    if prefix == PREFIX.NONE:
        decode_unprefixed(data, addr, result)

    # 3. CB-PREFIXED OPCODES
    elif prefix == PREFIX.CB:
        decode_cb(data, addr, result)

    # 4. ED-PREFIXED OPCODES
    elif prefix == PREFIX.ED:
        decode_ed(data, addr, result)

    # 5. DD-PREFIXED OPCODES
    # 6. FD-PREFIXED OPCODES
    elif prefix == PREFIX.DD or prefix == PREFIX.FD:
        if prefix == PREFIX.DD:
            (ra, rb, rc, rd) = (OPER_TYPE.REG_IX, OPER_TYPE.REG_IXH, OPER_TYPE.REG_IXL, OPER_TYPE.MEM_DISPL_IX)
        else:
            (ra, rb, rc, rd) = (OPER_TYPE.REG_IY, OPER_TYPE.REG_IYH, OPER_TYPE.REG_IYL, OPER_TYPE.MEM_DISPL_IY)

        if will_deref_hl(data[0]):
            if data[0] == 0xE9: # exceptional "JP (HL)" case
                displ = 0
                decode_unprefixed(data, addr, result)
            else:
                displ = int8(data[1])
                result.len += 1
                decode_unprefixed(data[0:1]+data[2:], addr, result)

            for i in range(len(result.operands)):
                if result.operands[i][0] == OPER_TYPE.REG_HL_DEREF:
                    result.operands[i] = (rd, displ)

        elif data[0] == 0xEB: # exceptional "EX DE,HL" case
            decode_unprefixed(data, addr+1, result)

        else:
            decode_unprefixed(data, addr+1, result)

            for i in range(len(result.operands)):
                if result.operands[i][0] == OPER_TYPE.REG_HL:
                    result.operands[i] = (ra,)
                if result.operands[i][0] == OPER_TYPE.REG_H:
                    result.operands[i] = (rb,)
                if result.operands[i][0] == OPER_TYPE.REG_L:
                    result.operands[i] = (rc,)

    # 7. DDCB/FDCB-PREFIXED OPCODES
    elif prefix in [PREFIX.DDCB, PREFIX.FDCB]:
        displ = int8(data[0])
        result.len += 1
        data = data[1:]
        decode_cb(data, addr, result)
        replacement = OPER_TYPE.MEM_DISPL_IX if prefix == PREFIX.DDCB else OPER_TYPE.MEM_DISPL_IY
        (x,y,z,p,q) = xyz(data[0])

        if x == 0:
            result.operands[0] = (replacement, displ)
            if z != 6:
                result.metaLoad = TABLE_R[z]
        elif x == 1:
            result.operands[1] = (replacement, displ)
        elif x == 2 or x == 3:
            result.operands[1] = (replacement, displ)
            if z != 6:
                result.metaLoad = TABLE_R[z]

    # done, recap
    if result.op == OP.NONI:
        result.status = DECODE_STATUS.INVALID_INSTRUCTION
    else:
        result.status = DECODE_STATUS.OK

        if result.op.value < OP.MARKER_END_LOAD_EXCHANGE.value:
            result.typ = INSTRTYPE.LOAD_EXCHANGE
        elif result.op.value < OP.MARKER_END_BLOCK_TRANSFER_SEARCH.value:
            result.typ = INSTRTYPE.BLOCK_TRANSFER_SEARCH
        elif result.op.value < OP.MARKER_END_ARITHMETIC_LOGICAL.value:
            result.typ = INSTRTYPE.ARITHMETIC_LOGICAL
        elif result.op.value < OP.MARKER_END_ROTATE_SHIFT.value:
            result.typ = INSTRTYPE.ROTATE_SHIFT
        elif result.op.value < OP.MARKER_END_BIT_MANIPULATION.value:
            result.typ = INSTRTYPE.BIT_MANIPULATION
        elif result.op.value < OP.MARKER_END_JUMP_CALL_RETURN.value:
            result.typ = INSTRTYPE.JUMP_CALL_RETURN
        elif result.op.value < OP.MARKER_END_INPUT_OUTPUT.value:
            result.typ = INSTRTYPE.INPUT_OUTPUT
        elif result.op.value < OP.MARKER_END_CPU_CONTROL.value:
            result.typ = INSTRTYPE.CPU_CONTROL

    return result

#------------------------------------------------------------------------------
# STRING MAKING
#------------------------------------------------------------------------------

CC_TO_STR = {
    CC.ALWAYS:"1", CC.NOT_N:"nn", CC.N:"n", CC.NOT_Z:"nz", CC.Z:"z",
    CC.NOT_C:"nc", CC.C:"c", CC.NOT_P:"po", CC.P:"pe", CC.NOT_S:"p", CC.S:"m",
    CC.NOT_H:"nh", CC.H:"h"
}

OP_TO_STR = {
    OP.NOP:"nop", OP.NONI:"noni", OP.LD:"ld", OP.EX:"ex", OP.EXX:"exx",
    OP.POP:"pop", OP.PUSH:"push", OP.LDI:"ldi", OP.CPI:"cpi", OP.INI:"ini",
    OP.OUTI:"outi", OP.LDD:"ldd", OP.CPD:"cpd", OP.IND:"ind", OP.OUTD:"outd",
    OP.LDIR:"ldir", OP.CPIR:"cpir", OP.INIR:"inir", OP.OTIR:"otir",
    OP.LDDR:"lddr", OP.CPDR:"cpdr", OP.INDR:"indr", OP.OTDR:"otdr",
    OP.ADD:"add", OP.ADC:"adc", OP.SUB:"sub", OP.SBC:"sbc", OP.AND:"and",
    OP.XOR:"xor", OP.OR:"or", OP.CP:"cp", OP.RLC:"rlc", OP.RRC:"rrc",
    OP.RL:"rl", OP.RR:"rr", OP.SLA:"sla", OP.SRA:"sra", OP.SLL:"sll",
    OP.SRL:"srl", OP.INC:"inc", OP.DEC:"dec", OP.NEG:"neg", OP.DAA:"daa",
    OP.CPL:"cpl", OP.SCF:"scf", OP.CCF:"ccf", OP.RETN:"retn", OP.RLCA:"rlca",
    OP.RRCA:"rrca", OP.RLA:"rla", OP.RRA:"rra", OP.RRD:"rrd", OP.RLD:"rld",
    OP.BIT:"bit", OP.RES:"res", OP.SET:"set", OP.CALL:"call", OP.RET:"ret",
    OP.JP:"jp", OP.JR:"jr", OP.DJNZ:"djnz", OP.RETI:"reti", OP.IN:"in",
    OP.OUT:"out", OP.HALT:"halt", OP.DI:"di", OP.EI:"ei", OP.IM:"im",
    OP.RST:"rst"
}

REG_TO_STR = {
    OPER_TYPE.REG_A:"a", OPER_TYPE.REG_F:"f", OPER_TYPE.REG_B:"b", OPER_TYPE.REG_C:"c",
    OPER_TYPE.REG_D:"d", OPER_TYPE.REG_E:"e", OPER_TYPE.REG_H:"h", OPER_TYPE.REG_L:"l",
    OPER_TYPE.REG_C_DEREF:"(c)", OPER_TYPE.REG_AF:"af", OPER_TYPE.REG_BC:"bc",
    OPER_TYPE.REG_DE:"de", OPER_TYPE.REG_HL:"hl", OPER_TYPE.REG_BC_DEREF:"(bc)",
    OPER_TYPE.REG_DE_DEREF:"(de)", OPER_TYPE.REG_HL_DEREF:"(hl)", OPER_TYPE.REG_A_:"a'",
    OPER_TYPE.REG_F_:"f'", OPER_TYPE.REG_B_:"b'", OPER_TYPE.REG_C_:"c'",
    OPER_TYPE.REG_D_:"d'", OPER_TYPE.REG_E_:"e'", OPER_TYPE.REG_H_:"h'",
    OPER_TYPE.REG_L_:"l'", OPER_TYPE.REG_AF_:"af'", OPER_TYPE.REG_BC_:"bc'",
    OPER_TYPE.REG_DE_:"de'", OPER_TYPE.REG_HL_:"hl'", OPER_TYPE.REG_I:"i",
    OPER_TYPE.REG_R:"r", OPER_TYPE.REG_IX:"ix", OPER_TYPE.REG_IXH:"ixh",
    OPER_TYPE.REG_IXL:"ixl", OPER_TYPE.REG_IY:"iy", OPER_TYPE.REG_IYH:"iyh",
    OPER_TYPE.REG_IYL:"iyl", OPER_TYPE.REG_SP:"sp", OPER_TYPE.REG_PC:"pc",
    OPER_TYPE.REG_SP_DEREF:"(sp)"
}

def uint2str(d):
    if d == 0:
        return '0'
    if d >= 16:
        return '0x%x' % d
    return '%d' % d

def displ2str(d):
    if d == 0:
        return ''
    if d >= 16:
        return '+0x%x' % d
    if d > 0:
        return '+%d' % d
    if d <= -16:
        return '-0x%x' % (-d)
    return '%d' % d

def oper2str(operType, val=None):
    if operType == OPER_TYPE.ADDR:
        if val < 0:
            val = val & 0xFFFF
        return '0x%04x' % val

    elif operType == OPER_TYPE.ADDR_DEREF:
        return '(0x%04x)' % val

    elif operType in [OPER_TYPE.MEM_DISPL_IX, OPER_TYPE.MEM_DISPL_IY]:
        lookup = {OPER_TYPE.MEM_DISPL_IX:'ix', OPER_TYPE.MEM_DISPL_IY:'iy'}
        return '(%s%s)' % (lookup[operType], displ2str(val))

    elif operType == OPER_TYPE.ADDR:
        return '0x%04X' % val

    elif operType == OPER_TYPE.IMM:
        return uint2str(val)

    elif operType == OPER_TYPE.CON:
        return CC_TO_STR[val]

    else:
        # eg REG_A -> 'A'
        return REG_TO_STR[operType]

def decoded2str(decoded):
    if decoded.status != DECODE_STATUS.OK:
        return ''

    result = OP_TO_STR[decoded.op]

    if decoded.operands:
        result += ' '

    result += ','.join(map(lambda operTypeAndValue: oper2str(*operTypeAndValue), decoded.operands))

    if decoded.metaLoad != OPER_TYPE.NONE:
        result = 'ld %s,%s' % (REG_TO_STR[decoded.metaLoad], result)

    return result

def disasm(data, pc = 0):
    if type(data) == bytes:
        decoded = decode(data, pc)
    else:
        decoded = data

    return decoded2str(decoded)
