#!/bin/sed -f
s/\(setmem *\/32[^0]*\)\(0x[0-9a-fA-F]*\)\( *=[^0]*\)\(0x[0-9a-fA-F]*\)/DATA.Set sd:\2 %le %long \4/g
