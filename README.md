# z80dis

This is a python disassembler library for Z80.

The decoding structure started from http://www.z80.info/decoding.htm and then made to sometimes agree with libopcodes or z80ex.

# Use

```
>>> z80.disasm(b'\xCB\xE7', 0)
'set 4,a'
```

Or, if you'd like access to the instruction internals, like opcode identifier and operands:

```
>>> decoded = z80.decode(b'\xCB\xE7', 0)
>>> z80.disasm(decoded)
'set 4,a'
```

