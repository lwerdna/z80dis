# z80dis

A python disassembler library for Z80

# Use

```
>>> from z80dis import z80
>>> z80.disasm(b'\xCB\xE7', 0)
'set 4,a'
```

Or, if you'd like access to the instruction internals, like opcode identifier, length, and operands:

```
>>> decoded = z80.decode(b'\xCB\xE7', 0)
>>> decoded.op
<OP.SET: 58>
>>> decoded.operands[0]
(<OPER_TYPE.IMM: 45>, 4)
>>> decoded.operands[1]
(<OPER_TYPE.REG_A: 1>,)
>>> decoded.len
2
```

The decoded structure can be supplied to disasm() to make a string:

```
>>> z80.disasm(decoded)
'set 4,a'
```

