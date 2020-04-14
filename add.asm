.ORIG x3000
        LEA R1, addy
        LDW R2, R1, #0
        AND R0, R0, #0
        ADD R0, R0, #1
        STW R0, R2, #0

        LEA R1, data
        LDW R2, R1, #0
        ADD R0, R0, #9
        ADD R0, R0, #10
        AND R3, R3, #0

loop    LDB R4, R2, #0
        ADD R3, R3, R4
        ADD R2, R2, #1
        ADD R0, R0, #-1
        BRp loop

        STW R3, R2, #0
        
    ;   STW R3, R0, #0
        ADD R2, R2, #3
        STW R3, R2, #0

    ;   .FILL xA000
        HALT

addy    .FILL x4000
data    .FILL xC000
.END