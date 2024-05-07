jz	macro 	LBL	
	btfsc	STATUS,Z
	goto	LBL
	endm
jnz	macro 	LBL	
	btfss	STATUS,Z
	goto	LBL
	endm
jc	macro 	LBL	
	btfss	STATUS,C
	goto	LBL
	endm
jnc	macro 	LBL	
	btfsc	STATUS,C
	goto	LBL
	endm
bank0   macro
        bcf     STATUS,RP0
	bcf	STATUS,RP1
        endm
bank1   macro
        bsf     STATUS,RP0
	bcf	STATUS,RP1
        endm
bank2   macro
        bcf     STATUS,RP0
	bsf	STATUS,RP1
        endm
bank3   macro
        bsf     STATUS,RP0
	bsf	STATUS,RP1
        endm
let	macro	aa,bb	
	movlw	bb
	movwf	aa
	endm
movab macro	aa,bb	
	movfw	aa
	movwf	bb
	endm

