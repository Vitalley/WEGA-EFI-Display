
;******************************************************************** 
;                  Binary To BCD Conversion Routine 
;      This routine converts a 16 Bit binary Number to a 5 Digit 
; BCD Number. This routine is useful since PIC16C55 &amp; PIC16C57 
; have  two 8 bit ports and one 4 bit port ( total of 5 BCD digits) 
; 
;       The 16 bit binary number is input in locations H_byte and 
; L_byte with the high byte in H_byte. 
;       The 5 digit BCD number is returned in R0, R1 and R2 with R0 
; containing the MSD in its right most nibble. 
; 
;   Performance : 
;               Program Memory  :       35 
;               Clock Cycles    :       885 
; 
;*******************************************************************; 
; 
count  equ      16 
temp   equ      17 
; 
H_byte  equ     10 
L_byte  equ     11 
R0      equ     12              ; RAM Assignments 
R1      equ     13 
R2      equ     14 
; 
	include         "picreg.h" 
; 
B2_BCD  bcf     STATUS,0                ; clear the carry bit 
	movlw   .16 
	movwf   count 
	clrf    R0 
	clrf    R1 
	clrf    R2 
loop16  rlf     L_byte 
	rlf     H_byte 
	rlf     R2 
	rlf     R1 
	rlf     R0 
; 
	decfsz  count 
	goto    adjDEC 
	RETLW   0 
; 
adjDEC  movlw   R2 
	movwf   FSR 
	call    adjBCD 
; 
	movlw   R1 
	movwf   FSR 
	call    adjBCD 
; 
	movlw   R0 
	movwf   FSR 
	call    adjBCD 
; 
	goto    loop16 
; 
adjBCD  movlw   3 
	addwf   0,W 
	movwf   temp 
	btfsc   temp,3          ; test if result &gt; 7 
	movwf   0 
	movlw   30 
	addwf   0,W 
	movwf   temp 
	btfsc   temp,7          ; test if result &gt; 7 
	movwf   0               ; save as MSD 
	RETLW   0 
; 
;******************************************************************** 
;               Test Program 
;********************************************************************* 
main    movlw   0FF 
	movwf   H_byte 
	movwf   L_byte          ; The 16 bit binary number = FFFF 
	call    B2_BCD          ; After conversion the Decimal Number 
;                               ; in R0,R1,R2 = 06,55,35 
; 
self    goto    self 
; 
	org     1FF 
	goto    main 
; 
	END 
 
 
