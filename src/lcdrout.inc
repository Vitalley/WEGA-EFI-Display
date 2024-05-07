;**************************************************************
; The LCD Module Subroutines                                      
;**************************************************************

;**************************************************************
;*SendChar - Sends character to LCD                                
;*This routine splits the character into the upper and lower        
;*nibbles and sends them to the LCD, upper nibble first.           
;**************************************************************
;
; LCD Module commands
; ===================
LINE1          EQU  0x080   ; Set display to line 1 character 0
LINE2          EQU  0x0C0   ; Set display to line 2 character 0

FUNCTION_SET   EQU  0x028   ; 4 bits, 2 lines, 5x7 Font 

;CLR_DISP - clears all display, and cursor home to addr 0
CLR_DISP       EQU  0x001   ; Clear the Display

;RTN_HOME - cursor home and display unshifted, DDRAM unchanged
RTN_HOME    EQU    0X002

;DISP_... - Display on / off control, C = cursor ON, B = chr pos blink
DISP_ON        EQU  0x00C   ; Display on
DISP_ON_C      EQU  0x00E   ; Display on, Cursor on
DISP_ON_B      EQU  0x00F   ; Display on, Cursor on, Blink cursor
DISP_OFF       EQU  0x008   ; Display off

;ENTRY_... - sets entry mode, S = display shift on, INC / DEC cursor move dir
ENTRY_INC      EQU  0x006   ;
ENTRY_INC_S    EQU  0x007   ;
ENTRY_DEC      EQU  0x004   ;                                  
ENTRY_DEC_S    EQU  0x005   ;

;SHIFT_. - Cursor & Display shift, S = Display, C = cursor, L = Left, R = Right
SHIFT_C_L     EQU  0x010
SHIFT_C_R     EQU  0x014
SHIFT_S_L     EQU  0x018
SHIFT_S_R     EQU  0x01C

DD_RAM_ADDR    EQU  0x080   ; Least Significant 7-bit are for address
DD_RAM_UL      EQU  0x080   ; Upper Left coner of the Display




;==============================================================
sch_b
Send_Char
	movwf   Char            ; Character to be sent is in W
	call    Wait_Busy       ; Wait for LCD to be ready
	movlw   0x0f
	andwf   LCD_DATA,F      ; Clear the upper nibble
	movf    Char,w          
	andlw   0xF0            ; Get upper nibble
	iorwf   LCD_DATA,F      ; Send data to LCD
	bcf     RW   		; Set LCD to write
	bsf     RS    		; Set LCD to data mode
	bsf     E     		; toggle E for LCD
	bcf     E
	movlw   0x0f
	andwf   LCD_DATA,F      ; Clear the upper nibble
	swapf   Char,W
	andlw   0xF0            ; Get lower nibble
	iorwf   LCD_DATA,F      ; Send data to LCD
	bsf     E     		; toggle E for LCD
	bcf     E
	return

;**************************************************************
;* Send_Cmd - Sends command to LCD                              
;* This routine splits the command into the upper and lower    
;* nibbles and sends them to the LCD, upper nibble first.      
;**************************************************************
scmd
Send_Cmd
	movwf   Char            ; Character to be sent is in W
	call    Wait_Busy       ; Wait for LCD to be ready
	movlw   0x0f
	andwf   LCD_DATA,F      ; Clear the upper nibble
	movf    Char,w          
	andlw   0xF0            ; Get upper nibble
	iorwf   LCD_DATA,F      ; Send data to LCD
	bcf     RW    ; Set LCD to write
	bcf     RS     ; Set LCD to command mode
	bsf     E      ; toggle E for LCD
	bcf     E
	movlw   0x0f
	andwf   LCD_DATA,F      ; Clear the upper nibble
	swapf   Char,W
	andlw   0xF0            ; Get lower nibble
	iorwf   LCD_DATA,F      ; Send data to LCD
	bsf     E      ; toggle E for LCD
	bcf     E
	return

;**************************************************************
;* This routine checks the busy flag, returns when not busy    
;*  Affects:                                                   
;*      Temp - Returned with busy/address                      
;**************************************************************

Wait_Busy
	bsf     STATUS, RP0     ; Select Register page 1
	movlw   0xf0            ; Set port to input
	iorwf   LCD_DATA_TRIS,W ; Only set upper half of port
	movwf   LCD_DATA_TRIS
	bcf     STATUS, RP0     ; Select Register page 0
	bcf     RS    		; Set LCD for Command mode
	bsf     RW   		; Setup to read busy flag
	bsf     E     		; Set E high
	movf    LCD_DATA, W     ; Read upper nibble busy flag, DDRam address
	bcf     E     		; Set E low
	andlw   0xF0            ; Mask out lower nibble
	movwf   Temp
	bsf     E     		; Toggle E to get lower nibble
	swapf   LCD_DATA, w     ; Read lower nibble busy flag, DDRam address
	bcf     E
	andlw   0x0F            ; Mask out upper nibble
	iorwf   Temp,f          ; Combine nibbles, RES in Temp !!!!!!!!!!!!!!!
	btfsc   Temp, 7         ; Check busy flag, high = busy
	goto    Wait_Busy       ; If busy, check again
	bcf     RW 
	bsf     STATUS, RP0     ; Select Register page 1
	movlw   0x0F
	andwf   LCD_DATA_TRIS,W
	movwf   LCD_DATA_TRIS   ; Set Port for output
	bcf     STATUS, RP0     ; Select Register page 0
	return

;**************************************************************
del10mks
	goto $+1
	goto $+1
	goto $+1
	goto $+1
	goto $+1
	goto $+1
	goto $+1
	goto $+1
	goto $+1
	return
;**************************************************************
; Initilize the LCD Display Module
;**************************************************************
LCD_Init:

	bcf     E     ; Clear all controll lines
	bcf     RS
	bcf     RW
	call    delay20		; Wait for 40ms for LCD to get powered up
	call    delay20
	movlw   0x0f
	andwf   LCD_DATA,F      ; Clear the upper nibble
	movlw   0x030           ; Command for 4-bit interface high nibble
	iorwf   LCD_DATA,F      ; Send data to LCD      
	bsf     STATUS, RP0     ; Select Register page 1
	movlw   0x0F
	andwf   LCD_DATA_TRIS,W
	movwf   LCD_DATA_TRIS   ; Set Port for output
	BCF     STATUS, RP0     ; Select Register page 0
	bsf     E     		; Clock the initalize command to LCD module
	bcf     E
	call    delay5		; Delay for at least 4.1ms before continuing
	bsf     E     		; Clock the initalize command to LCD module
	bcf     E
	call    d100mks        	; delay for at least 100usec before continuing
	bsf     E     		; Clock the initalize command to LCD module
	bcf     E
	call    Wait_Busy       ; From here on, the Busy Bit will be valid.
	movlw   0x0f
	andwf   LCD_DATA,F      ; Clear the upper nibble
	movlw   0x020           ; Command for 4-bit interface high nibble 
	iorwf   LCD_DATA,F      ; Send data to LCD      
	bsf     E     		; Clock the initalize command to LCD module
	bcf     E
	movlw   FUNCTION_SET    ; Send the function set command 4-bit I/F, Font, Number of lines
	call    Send_Cmd        ; Can now use the Send_Cmd routine since Busy Bit Valid and in 4bit mode.
	


	return
; Программирование символов для изображения линейки  
SET_CGRAM	movlw	40	; Установка начального адреса CGRAM
	call	Send_Cmd



	let	temp1,.64
	MOVLW	HIGH SYM_DOT
	MOVWF	PCLATH
	MOVLW	LOW SYM_DOT
	MOVWF	temp2
	CALL	READ_STR
	CALL	sch_b
	INCF	temp2,F
	DECFSZ	temp1,F
	GOTO	$-4


	movlw	80	; Процедура DDRAM SET
	call	Send_Cmd
	return

LPG_CHAR 	movlw	40+.40	; Установка начального адреса CGRAM
	call	Send_Cmd

	MOVF	LPG_LEVEL,W	;LPG_LEVE=0
	BTFSS	STATUS,Z
	GOTO	LCHR
	let	temp1,.8	; РИСУЕМ ЗНАК ОБРЫВА ДАТЧИКА
	MOVLW	HIGH SYM_F_ER
	MOVWF	PCLATH
	MOVLW	LOW SYM_F_ER
	MOVWF	temp2
	CALL	READ_STR
	CALL	sch_b
	INCF	temp2,F
	DECFSZ	temp1,F
	GOTO	$-4
	RETURN

	
LCHR	MOVLW	.142
	CALL	sch_b
	MOVLW	.159
	CALL	sch_b

;	let LPG_LEVEL,4

	MOVLW	.5
	MOVWF	temp1
MCH	MOVFW	LPG_LEVEL
	SUBWF	temp1,W
	BTFSS	STATUS,C
	BTFSC	STATUS,Z
	GOTO	$+3
	MOVLW	.159
	GOTO	$+2
	MOVLW	.145
	CALL	sch_b
	DECFSZ	temp1,F
	GOTO	MCH
	MOVLW	.159
	CALL	sch_b
	movlw	80	; Процедура DDRAM SET
	call	Send_Cmd
	RETURN

LAMBDA_CHAR	movlw	40+.48	; Установка начального адреса CGRAM
	call	Send_Cmd

	movlw	80	; Процедура DDRAM SET
	call	Send_Cmd
	RETURN
