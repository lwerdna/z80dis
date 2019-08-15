#!/usr/bin/env python

# example use:
#
# z80.disasm(b'\xCB\xE7', 0) -> 'set 4,a' 
# 
# decoded = z80.decode(b'\xCB\xE7', 0)
# z80.disasm(decoded) -> 'set 4,a'

import sys
from enum import Enum, auto, unique
import struct

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
class SRCDST(Enum):
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
    SRCDST_LAST_I = auto()

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
        # "type" of source, destination
        self.lhs = None				# SRCDST
        self.rhs = None				# SRCDST
        # if further resolution is needed of src, dst
        self.lhs_ = None
        self.rhs_ = None
        # source of load
        self.imm = None
        # width {8, 16}
        self.width = None
        # whether the entire instruction's result gets written to a reg
        # (special case for DDCB, FDCB)
        self.metaLoad = SRCDST.NONE

#------------------------------------------------------------------------------
# DECODING
#------------------------------------------------------------------------------

# tables
# (from http:#z80.info/decoding.htm)

TABLE_R = [
    SRCDST.REG_B,
    SRCDST.REG_C,
    SRCDST.REG_D,
    SRCDST.REG_E,
    SRCDST.REG_H,
    SRCDST.REG_L,
    SRCDST.REG_HL_DEREF,
    SRCDST.REG_A
]

# register pairs featuring SP
TABLE_RP = [
    SRCDST.REG_BC,
    SRCDST.REG_DE,
    SRCDST.REG_HL,
    SRCDST.REG_SP
]

# register pairs featuring AF
TABLE_RP2 = [
    SRCDST.REG_BC,
    SRCDST.REG_DE,
    SRCDST.REG_HL,
    SRCDST.REG_AF
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
    (dc.lhs, dc.rhs) = (dc.rhs, dc.lhs)
    (dc.lhs_, dc.rhs_) = (dc.rhs_, dc.lhs_)

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
                result.lhs = SRCDST.REG_AF
                result.rhs = SRCDST.REG_AF_
            elif y == 2:
                result.op = OP.DJNZ
                result.lhs = SRCDST.ADDR
                result.lhs_ = addr + 2 + int8(data[1])
                result.len += 1
            else: # 3,4,5,6,7
                result.op = OP.JR
                result.lhs = SRCDST.CON
                result.lhs_ = TABLE_CC[y-4]
                result.rhs = SRCDST.ADDR
                result.rhs_ = addr + 2 + int8(data[1])
                result.len += 1
                if y == 3:
                    result.lhs = result.rhs
                    result.lhs_ = result.rhs_
                    result.rhs = None

        elif z == 1:
            # 16-bit load immediate
            if q == 0:
                result.op = OP.LD
                result.lhs = TABLE_RP[p]
                result.rhs = SRCDST.IMM
                result.rhs_ = uint16(data[1], data[2])
                result.len += 2
            elif q == 1:
                result.op = OP.ADD
                result.lhs = SRCDST.REG_HL
                result.rhs = SRCDST(TABLE_RP[p])    # from register pair

        elif z == 2:
            result.op = OP.LD
            if p == 0:
                result.lhs = SRCDST.REG_BC_DEREF
                result.rhs = SRCDST.REG_A
            elif p == 1:
                result.lhs = SRCDST.REG_DE_DEREF
                result.rhs = SRCDST.REG_A
            elif p == 2:
                result.lhs = SRCDST.ADDR_DEREF
                result.lhs_ = uint16(data[1], data[2])
                result.len += 2
                result.rhs = SRCDST.REG_HL
            else:
                result.lhs = SRCDST.ADDR_DEREF
                result.lhs_ = uint16(data[1], data[2])
                result.len += 2
                result.rhs = SRCDST.REG_A

            if q:
                reorder(result)

        elif z == 3:
            if q:
                result.op = OP.DEC
            else:
                result.op = OP.INC
            result.lhs = TABLE_RP[p]

        elif z == 4:
            result.op = OP.INC
            result.lhs = TABLE_R[y]

        elif z == 5:
            result.op = OP.DEC
            result.lhs = TABLE_R[y]

        elif z == 6:
            result.op = OP.LD
            result.lhs = TABLE_R[y]
            result.rhs = SRCDST.IMM
            result.rhs_ = data[1]
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
            result.lhs = TABLE_R[y]
            result.rhs = TABLE_R[z]

    elif x == 2:
        result.op = TABLE_ALU_OP[y]
        result.lhs = SRCDST.REG_A
        result.rhs = TABLE_R[z]
        if result.op in [OP.SUB, OP.AND, OP.XOR, OP.OR, OP.CP]:
            result.lhs = result.rhs
            result.rhs = None

    elif x == 3:
        # conditional return
        # no prefix, x==3, z==0
        if z == 0:
            result.op = OP.RET
            result.lhs = SRCDST.CON
            result.lhs_ = TABLE_CC[y]

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
                    result.lhs = SRCDST.REG_HL_DEREF
                elif p == 3:
                    result.op = OP.LD
                    result.lhs = SRCDST.REG_SP
                    result.rhs = SRCDST.REG_HL

            else:
                result.op = OP.POP
                result.lhs = TABLE_RP2[p]

        # no prefix, x==3, z==2
        elif z == 2:
            result.op = OP.JP
            result.lhs = SRCDST.CON
            result.lhs_ = TABLE_CC[y]
            result.rhs = SRCDST.ADDR
            result.rhs_ = uint16(data[1], data[2])
            result.len += 2
            
        # no prefix, x==3, z==3
        elif z == 3:
            if y == 0:
                result.op = OP.JP
                result.lhs = SRCDST.ADDR
                result.lhs_ = uint16(data[1], data[2])
                result.len += 2
            elif y == 1:
                pass
            elif y == 2:
                result.op = OP.OUT
                result.lhs = SRCDST.ADDR_DEREF
                result.lhs_ = data[1]
                result.len += 1
                result.rhs = SRCDST.REG_A
            elif y == 3:
                result.op = OP.IN
                result.lhs = SRCDST.REG_A
                result.rhs = SRCDST.ADDR_DEREF
                result.rhs_ = data[1]
                result.len += 1
            elif y == 4:
                result.op = OP.EX
                result.lhs = SRCDST.REG_SP_DEREF
                result.rhs = SRCDST.REG_HL
            elif y == 5:
                result.op = OP.EX
                result.lhs = SRCDST.REG_DE
                result.rhs = SRCDST.REG_HL
            elif y == 6:
                result.op = OP.DI
            elif y == 7:
                result.op = OP.EI

        elif z == 4:
            result.op = OP.CALL
            result.lhs = SRCDST.CON
            result.lhs_ = TABLE_CC[y]
            result.rhs = SRCDST.ADDR
            result.rhs_ = uint16(data[1], data[2])
            result.len += 2
        elif z == 5:
            if q:
                if p:
                    # fuckup! this is DD, ED, or FD prefix!!
                    pass
                else:
                    result.op = OP.CALL
                    result.lhs = SRCDST.ADDR
                    result.lhs_ = uint16(data[1], data[2])
                    result.len += 2

            else:
                result.op = OP.PUSH
                result.lhs = TABLE_RP2[p]

        elif z == 6:
            # accumulator and immediate
            result.op = TABLE_ALU_OP[y]
            result.lhs = SRCDST.REG_A
            result.rhs = SRCDST.IMM
            result.rhs_ = data[1]
            result.len += 1
            if result.op in [OP.SUB, OP.AND, OP.XOR, OP.OR, OP.CP]:
                result.lhs = result.rhs
                result.lhs_ = result.rhs_
                result.rhs = None

        elif z == 7:
            result.op = OP.RST
            result.lhs = SRCDST.IMM
            result.lhs_ = 8*y

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

        result.rhs = TABLE_R[z]
        result.lhs = SRCDST.IMM
        result.lhs_ = y
    else:
        result.op = TABLE_ROT[y]
        result.lhs = TABLE_R[z]

def decode_ed(data, addr, result):
    (x,y,z,p,q) = xyz(data[0])
    result.len += 1
    if x == 0 or x == 3:
        result.op = OP.NONI
    elif x == 1:
        if z == 0:
            result.op = OP.IN
            result.rhs = SRCDST.REG_C_DEREF
            if y != 6:
                result.lhs = TABLE_R[y]
            else:
                result.lhs = SRCDST.REG_F

        elif z == 1:
            result.op = OP.OUT
            result.lhs = SRCDST.REG_C_DEREF
            if y != 6:
                result.rhs = TABLE_R[y]
            else:
                result.rhs = SRCDST.IMM
                result.rhs_ = 0

        elif z == 2:
            result.op = OP.SBC
            result.lhs = SRCDST.REG_HL
            result.rhs = TABLE_RP[p]
            if q:
                result.op = OP.ADC

        elif z == 3:
            result.op = OP.LD
            result.lhs = SRCDST.ADDR_DEREF
            result.lhs_ = uint16(data[1], data[2])
            result.len += 2
            result.rhs = TABLE_RP[p]
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
            result.lhs = SRCDST.IMM
            result.lhs_ = TABLE_IM[y]

        elif z == 7:
            if y == 0:
                result.op = OP.LD
                result.lhs = SRCDST.REG_I
                result.rhs = SRCDST.REG_A
            elif y == 1:
                result.op = OP.LD
                result.lhs = SRCDST.REG_R
                result.rhs = SRCDST.REG_A
            elif y == 2:
                result.op = OP.LD
                result.lhs = SRCDST.REG_A
                result.rhs = SRCDST.REG_I
            elif y == 3:
                result.op = OP.LD
                result.lhs = SRCDST.REG_A
                result.rhs = SRCDST.REG_R
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

    #print(prefix)

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
            (ra, rb, rc, rd) = (SRCDST.REG_IX, SRCDST.REG_IXH, SRCDST.REG_IXL, SRCDST.MEM_DISPL_IX)
        else:
            (ra, rb, rc, rd) = (SRCDST.REG_IY, SRCDST.REG_IYH, SRCDST.REG_IYL, SRCDST.MEM_DISPL_IY)

        if will_deref_hl(data[0]):
            if data[0] == 0xE9: # exceptional "JP (HL)" case
                displ = 0
                decode_unprefixed(data, addr, result)
            else:
                displ = int8(data[1])
                result.len += 1
                decode_unprefixed(data[0:1]+data[2:], addr, result)

            if result.rhs == SRCDST.REG_HL_DEREF:
                result.rhs = rd
                result.rhs_ = 0 if result.op == OP.JP else displ
            elif result.lhs == SRCDST.REG_HL_DEREF:
                result.lhs = rd
                result.lhs_ = 0 if result.op == OP.JP else displ

        elif data[0] == 0xEB: # exceptional "EX DE,HL" case
            decode_unprefixed(data, addr+1, result)

        else:
            decode_unprefixed(data, addr+1, result)

            if result.rhs == SRCDST.REG_HL:
                result.rhs = ra
            if result.rhs == SRCDST.REG_H:
                result.rhs = rb
            if result.rhs == SRCDST.REG_L:
                result.rhs = rc

            if result.lhs == SRCDST.REG_HL:
                result.lhs = ra
            if result.lhs == SRCDST.REG_H:
                result.lhs = rb
            if result.lhs == SRCDST.REG_L:
                result.lhs = rc

    # 7. DDCB/FDCB-PREFIXED OPCODES
    elif prefix in [PREFIX.DDCB, PREFIX.FDCB]:
        displ = int8(data[0])
        result.len += 1
        data = data[1:]
        decode_cb(data, addr, result)
        replacement = SRCDST.MEM_DISPL_IX if prefix == PREFIX.DDCB else SRCDST.MEM_DISPL_IY
        (x,y,z,p,q) = xyz(data[0])

        if x == 0:
            result.lhs = replacement
            result.lhs_ = displ
            if z != 6:
                result.metaLoad = TABLE_R[z]
        elif x == 1:
            result.rhs = replacement
            result.rhs_ = displ
        elif x == 2 or x == 3:
            result.rhs = replacement
            result.rhs_ = displ
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
    SRCDST.REG_A:"a", SRCDST.REG_F:"f", SRCDST.REG_B:"b", SRCDST.REG_C:"c",
    SRCDST.REG_D:"d", SRCDST.REG_E:"e", SRCDST.REG_H:"h", SRCDST.REG_L:"l",
    SRCDST.REG_C_DEREF:"(c)", SRCDST.REG_AF:"af", SRCDST.REG_BC:"bc",
    SRCDST.REG_DE:"de", SRCDST.REG_HL:"hl", SRCDST.REG_BC_DEREF:"(bc)",
    SRCDST.REG_DE_DEREF:"(de)", SRCDST.REG_HL_DEREF:"(hl)", SRCDST.REG_A_:"a'",
    SRCDST.REG_F_:"f'", SRCDST.REG_B_:"b'", SRCDST.REG_C_:"c'",
    SRCDST.REG_D_:"d'", SRCDST.REG_E_:"e'", SRCDST.REG_H_:"h'",
    SRCDST.REG_L_:"l'", SRCDST.REG_AF_:"af'", SRCDST.REG_BC_:"bc'",
    SRCDST.REG_DE_:"de'", SRCDST.REG_HL_:"hl'", SRCDST.REG_I:"i",
    SRCDST.REG_R:"r", SRCDST.REG_IX:"ix", SRCDST.REG_IXH:"ixh",
    SRCDST.REG_IXL:"ixl", SRCDST.REG_IY:"iy", SRCDST.REG_IYH:"iyh",
    SRCDST.REG_IYL:"iyl", SRCDST.REG_SP:"sp", SRCDST.REG_PC:"pc",
    SRCDST.REG_SP_DEREF:"(sp)"
}

def uint2str(d):
    if d == 0: return '0'
    if d >= 16: return '0x%x' % d
    return '%d' % d 

def displ2str(d):
    if d == 0: return ''
    if d >= 16: return '+0x%x' % d
    if d > 0: return '+%d' % d
    if d <= -16: return '-0x%x' % (-d)
    return '%d' % d 

def oper2str(operType, val):
    if operType == SRCDST.ADDR:
        if val < 0:
            val = val & 0xFFFF
        return '0x%04x' % val

    elif operType == SRCDST.ADDR_DEREF:
        return '(0x%04x)' % val

    elif operType in [SRCDST.MEM_DISPL_IX, SRCDST.MEM_DISPL_IY]:
        lookup = {SRCDST.MEM_DISPL_IX:'ix', SRCDST.MEM_DISPL_IY:'iy'}
        return '(%s%s)' % (lookup[operType], displ2str(val))

    elif operType == SRCDST.ADDR:
        return '0x%04X' % val        

    elif operType == SRCDST.IMM:
        return uint2str(val)

    elif operType == SRCDST.CON:
        return CC_TO_STR[val]

    else:
        # eg REG_A -> 'A'
        return REG_TO_STR[operType]

def decoded2str(decoded):
    if decoded.status != DECODE_STATUS.OK:
        return ''

    # opcode
    result = OP_TO_STR[decoded.op]

    # destination
    if decoded.lhs:
        result += ' '
        result += oper2str(decoded.lhs, decoded.lhs_)

    if decoded.rhs:
        if decoded.lhs != -1:
            result += ','

        result += oper2str(decoded.rhs, decoded.rhs_)

    if decoded.metaLoad != SRCDST.NONE:
        result = 'ld %s,%s' % (REG_TO_STR[decoded.metaLoad], result)

    return result

def disasm(data, pc = 0):
    if type(data) == bytes:
        decoded = decode(data, pc)
    else:
        decoded = data

    return decoded2str(decoded)
