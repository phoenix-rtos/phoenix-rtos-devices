.386
.model tiny
_TEXT segment 'CODE'

db 1024 dup(?)
__stack0: 
db 1024 dup(?)
__stack1: 

public __chkfl
__chkfl: db 0

public __isr_func
__isr_func: dw 0


public __isr_stub
__isr_stub:
  cli
  push  ax
  push  bx
  mov   bx, cs
  mov   ax, ss
  mov   ss, bx
  mov   bx, sp
  mov   sp, offset __stack0
  test  byte ptr cs:[__chkfl], 0ffh
  jz    __isr_stub1
  mov   sp, offset __stack1
__isr_stub1:
  push  eax
  push  ebx
  push  ecx
  push  edx
  push  esi
  push  edi
  push  ebp
  push  ds
  push  es

  mov   ax, cs
  mov   ds, ax
  mov   es, ax
  call  word ptr cs:[__isr_func]

  pop   es
  pop   ds
  pop   ebp
  pop   edi
  pop   esi
  pop   edx
  pop   ecx
  pop   ebx
  pop   eax
  cli
  mov   ss, ax
  mov   sp, bx
  pop   bx
  pop   ax
  iret

_TEXT ends
end
