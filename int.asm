        .ORIG   x1200
        STW     R0, R6, #-2 ;push R0, R2 onto stack
        STW     R2, R6, #-4

        LEA     R2, addy
        LDW     R2, R2, #0
        LDW     R0, R2, #0
        ADD     R0, R0, #1
        STW     R0, R2, #0

        LDW     R0, R6, #-2 ;restore R0, R2
        LDW     R2, R6, #-4
        RTI

addy    .FILL   x4000
        .END