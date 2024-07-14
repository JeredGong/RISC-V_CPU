# RISC-V_CPU_excepition log
## 7.9-7.10
Half finished interrupt, this used exclusively hardware to realize interrupts & exceptions. Though,it diverges from the riscv too far.
Obselete

## 7.12
According to riscv,Gong made an outline.
Tarnished implemented csr instusctions. csr instructions are implemented on the pipe, using a third datapath similar to other input circuits of the alu.
Noted: 
Some csr instructions use imm(which uses rs2 datapath), some use rs1, so the rs3 may be placed to rs1 or rs2's datapath,according to the csr inst type.
csr instructions still conform the pipe's rule, running through the five stages.
Haven't checked!!!

## 7.13
csr instruction checked
added exception unit.

Note:
Exception unit is in EXE stage. All exception will flow to here.
Only exception detection part checked.
csr assignment unfinished.
Clock rate drops by half.

## 7.14
Implemented csr assignment when exception occurs. This part is done at the WB stage, two stages after the exception detected.
Unchecked on board.
