.data
array: .float  111580.458282 -167174.411201 -349272.896423 239001.562000 175942.811493 -149843.657102 -77524.913999 -112955.295469 -492257.707267 -398753.669721 -229076.651482 -469511.839788 144308.792695 -505235.710433
xor:   0
sum:   .float  0.0
.text
   addi   $t1, $t0, 0
   addi   $t2, $t1, 56
   lw     $t4, 0($t2) 
   lwc1   $f2, 4($t2)
loop:
   beq    $t1, $t2, store
   lw     $t3, 0($t1)
   xor    $t4, $t4, $t3
   lwc1   $f1, 0($t1)
   add.s  $f2, $f2, $f1
   addi   $t1, $t1, 4
   j      loop
store:
   sw     $t4, 0($t2)
   swc1   $f2, 4($t2)
