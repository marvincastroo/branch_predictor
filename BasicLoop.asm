.section .text
.globl _start
_start:
# Basic loop
  ADDI t1, zero, 10 # t1 to 10
  ADDI t2, zero, 1 #t2 to 1
  ADDI t3, zero, 18
loop:
  SUB t1, t1, t2
  BNE t1, zero, loop
 
  ADDI t1, zero, 0 
 
  ADDI t1, t1, 1
  ADDI t5, zero, 3
  BEQ t1, t5, loop1
  
  ADDI t1, t1, 1
  ADDI t5, zero, 3
  BEQ t1, t5, loop1
  
  ADDI t1, t1, 1
  ADDI t5, zero, 3
  BEQ t1, t5, loop1
  
  loop1:
  ADDI t1, zero, 0
  
  ADDI t1, t1, 1
  ADDI t5, zero, 2
  BEQ t1, t5, loop2
  
  ADDI t1, t1, 1
  ADDI t5, zero, 2
  BEQ t1, t5, loop2
  ADDI t1, zero, 0
  
  ADDI t1, t1, 1
  ADDI t5, zero, 2
  BEQ t1, t5, loop2
  
   ADDI t1, t1, 1
  ADDI t5, zero, 2
  BEQ t1, t5, loop2
  ADDI t1, zero, 0
    
  loop2:
  
  
  
 
# END
  ECALL
