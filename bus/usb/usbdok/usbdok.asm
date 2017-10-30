.386
.model tiny

extern _init: near
extern _main: near
extern _umass_state: near
extern _umass_read: near
extern _umass_write: near

_TEXT segment 'CODE'

org 0
__drv_head:
  dw    0ffffh
  dw    0
  dw    0002h
  dw    offset __drv_str
  dw    offset __drv_int
  db    2, 0, 0, 0, 0, 0, 0, 0

__drv_bpbary:
  dw    offset __drv_bpb, offset __drv_bpb

__drv_label1:
  db    12 dup(0)

__drv_label2:
  db    12 dup(0)

__drv_bpb:
  dw    512
  db    1
  dw    1
  db    1
  dw    64
  dw    128
  db    0fah
  dw    3
  dw    1
  dw    1
  dd    0
  dd    0
  dw    0
  db    0
  dd    0
  db    'IMMOS_DOK  '
  db    'FAT16   '

__drv_buff:
  db    512 dup(?)

__drv_ptr:
  dw    2 dup(?)

  db 2048 dup(?)
__drv_stack:


__drv_str:
  mov   word ptr cs:[__drv_ptr], bx
  mov   word ptr cs:[__drv_ptr + 2], es
  retf


__drv_int:
  pushf
  push  ax
  push  bx
  cli
  cld
  mov   bx, cs
  mov   ax, ss
  mov   ss, bx
  mov   bx, sp
  mov   sp, offset __drv_stack
  sti
  push  eax
  push  ebx
  push  ecx
  push  edx
  push  esi
  push  edi
  push  ebp
  push  ds
  push  es

  mov   bx, word ptr cs:[__drv_ptr]
  mov   es, word ptr cs:[__drv_ptr + 2]

  cmp   byte ptr es:[bx + 2], 0
  jz    __drv_init

  cmp   byte ptr es:[bx + 2], 1
  jz    __drv_media

  cmp   byte ptr es:[bx + 2], 2
  jz    __drv_buildbpb

  cmp   byte ptr es:[bx + 2], 4
  jz    __drv_read

  cmp   byte ptr es:[bx + 2], 8
  jz    __drv_write

  cmp   byte ptr es:[bx + 2], 9
  jz    __drv_write

  mov   word ptr es:[bx + 3], 8003h

__drv_int1:
  mov   bx, word ptr cs:[__drv_ptr]
  mov   es, word ptr cs:[__drv_ptr + 2]
  or   word ptr es:[bx + 3], 100h

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
  popf
  retf


__drv_init:
  mov   ax, cs
  mov   ds, ax
  mov   es, ax
  call  _init
  mov   bx, word ptr cs:[__drv_ptr]
  mov   es, word ptr cs:[__drv_ptr + 2]
  or    ax, ax
  jnz   __drv_init1

  mov   word ptr es:[bx + 3], 0
  mov   byte ptr es:[bx + 0dh], 2
  mov   word ptr es:[bx + 0eh], 0ffffh
  mov   word ptr es:[bx + 10h], cs
  mov   word ptr es:[bx + 12h], offset __drv_bpbary
  mov   word ptr es:[bx + 14h], cs
  jmp   __drv_int1

__drv_init1:
  mov   word ptr es:[bx + 3], 0
  mov   byte ptr es:[bx + 0dh], 0
  mov   word ptr es:[bx + 0eh], 0
  mov   word ptr es:[bx + 10h], cs
  jmp   __drv_int1


__drv_media:
  movzx ax, byte ptr es:[bx + 1]
  push  ax
  mov   ax, cs
  mov   ds, ax
  mov   es, ax
  call  _umass_state
  add   sp, 2
  mov   bx, word ptr cs:[__drv_ptr]
  mov   es, word ptr cs:[__drv_ptr + 2]
  mov   byte ptr es:[bx + 0eh], 1
  or    ax, ax
  jnz   __drv_media1

  mov   byte ptr es:[bx + 0eh], 0ffh
  mov   word ptr es:[bx + 0fh], offset __drv_label1
  mov   word ptr es:[bx + 11h], cs
  cmp   byte ptr es:[bx + 1], 0
  jz    __drv_media1
  mov   word ptr es:[bx + 0fh], offset __drv_label2

__drv_media1:
  mov   word ptr es:[bx + 3], 0
  jmp   __drv_int1


__drv_buildbpb:
  push  word ptr 1
  push  dword ptr 0
  xor   eax, eax
  mov   ax, cs
  shl   eax, 4
  xor   edx, edx
  mov   dx, offset __drv_buff
  add   eax, edx
  push  eax
  movzx ax, byte ptr es:[bx + 1]
  push  ax
  mov   ax, cs
  mov   ds, ax
  mov   es, ax
  call  _umass_read
  add   sp, 12
  or    ax, ax
  jnz   __drv_buildbpb2

  mov   si, offset __drv_buff + 2bh
  mov   di, offset __drv_label1
  mov   bx, word ptr cs:[__drv_ptr]
  mov   es, word ptr cs:[__drv_ptr + 2]
  cmp   byte ptr es:[bx + 1], 0
  jz    __drv_buildbpb1
  mov   di, offset __drv_label2

__drv_buildbpb1:
  mov   ax, cs
  mov   es, ax
  mov   cx, 11
  rep
  movsb
  mov   bx, word ptr cs:[__drv_ptr]
  mov   es, word ptr cs:[__drv_ptr + 2]
  mov   word ptr es:[bx + 12h], offset __drv_buff + 0bh
  mov   word ptr es:[bx + 14h], cs
  mov   word ptr es:[bx + 3], 0
  jmp   __drv_int1

__drv_buildbpb2:
  mov   bx, word ptr cs:[__drv_ptr]
  mov   es, word ptr cs:[__drv_ptr + 2]
  mov   word ptr es:[bx + 12h], offset __drv_bpb
  mov   word ptr es:[bx + 14h], cs
  mov   word ptr es:[bx + 3], 0
  jmp   __drv_int1


__drv_read:
  push  word ptr es:[bx + 12h]
  movzx eax, word ptr es:[bx + 14h]
  cmp   ax, 0ffffh
  jnz   __drv_read2
  mov   eax, dword ptr es:[bx + 1ah]
__drv_read2:
  push  eax
  movzx eax, word ptr es:[bx + 10h]
  shl   eax, 4
  movzx edx, word ptr es:[bx + 0eh]
  add   eax, edx
  push  eax
  movzx ax, byte ptr es:[bx + 1]
  push  ax
  mov   ax, cs
  mov   ds, ax
  mov   es, ax
  call  _umass_read
  add   sp, 12
  mov   bx, word ptr cs:[__drv_ptr]
  mov   es, word ptr cs:[__drv_ptr + 2]
  or    ax, ax
  jnz   __drv_read3
  mov   word ptr es:[bx + 3], 0
  jmp   __drv_int1

__drv_read3:
  cmp   ax, -34
  jz    __drv_read5
  cmp   ax, -32
  jnz   __drv_read6
  mov   word ptr es:[bx + 16h], offset __drv_label1
  cmp   byte ptr es:[bx + 1], 0
  jz    __drv_read4
  mov   word ptr es:[bx + 16h], offset __drv_label2
__drv_read4:
  mov   word ptr es:[bx + 18h], cs
  mov   word ptr es:[bx + 3], 800fh
  jmp   __drv_int1

__drv_read5:
  mov   word ptr es:[bx + 12h], 0
  mov   word ptr es:[bx + 18h], cs
  mov   word ptr es:[bx + 3], 800bh
  jmp   __drv_int1

__drv_read6:
  mov   word ptr es:[bx + 12h], 0
  mov   word ptr es:[bx + 18h], cs
  mov   word ptr es:[bx + 3], 8002h
  jmp   __drv_int1


__drv_write:
  push  word ptr es:[bx + 12h]
  movzx eax, word ptr es:[bx + 14h]
  cmp   ax, 0ffffh
  jnz   __drv_write2
  mov   eax, dword ptr es:[bx + 1ah]
__drv_write2:
  push  eax
  movzx eax, word ptr es:[bx + 10h]
  shl   eax, 4
  movzx edx, word ptr es:[bx + 0eh]
  add   eax, edx
  push  eax
  movzx ax, byte ptr es:[bx + 1]
  push  ax
  mov   ax, cs
  mov   ds, ax
  mov   es, ax
  call  _umass_write
  add   sp, 12
  mov   bx, word ptr cs:[__drv_ptr]
  mov   es, word ptr cs:[__drv_ptr + 2]
  or    ax, ax
  jnz   __drv_write3
  mov   word ptr es:[bx + 3], 0
  jmp   __drv_int1

__drv_write3:
  cmp   ax, -34
  jz    __drv_write5
  cmp   ax, -32
  jnz   __drv_write6
  mov   word ptr es:[bx + 16h], offset __drv_label1
  cmp   byte ptr es:[bx + 1], 0
  jz    __drv_write4
  mov   word ptr es:[bx + 16h], offset __drv_label2
__drv_write4:
  mov   word ptr es:[bx + 18h], cs
  mov   word ptr es:[bx + 3], 800fh
  jmp   __drv_int1

__drv_write5:
  mov   word ptr es:[bx + 12h], 0
  mov   word ptr es:[bx + 18h], cs
  mov   word ptr es:[bx + 3], 800ah
  jmp   __drv_int1

__drv_write6:
  mov   word ptr es:[bx + 12h], 0
  mov   word ptr es:[bx + 18h], cs
  mov   word ptr es:[bx + 3], 8002h
  jmp   __drv_int1


__drv_start:
  mov sp, offset __drv_stack
  mov ax, cs
  mov ds, ax
  mov es, ax
  mov ss, ax
  call _main
  mov ax, 4c00h
  int 21h

_TEXT ends
end __drv_start

