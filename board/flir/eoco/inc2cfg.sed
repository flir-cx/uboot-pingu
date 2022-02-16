#!/bin/sed -f
s/\(setmem *\/32[^0]*\)\(0x[0-9a-fA-F]*\)\( *=[^0]*\)\(0x[0-9a-fA-F]*\)/DATA 4  \2   \4/g
