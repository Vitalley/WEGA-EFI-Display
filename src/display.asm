; Исправлена ошибка зависания дисплея
; 10,12,15 Вывод температуры в полном формате без таблицы, ошибка около 2,5%
; 11.12.15 Вывод расхода топлива литры, час
; 04.01.16 Исправлен вывод  дробных чисел
; 03.03.16 Добавлена проверка на максимальный и минимальный размер пакета 
; 08.07.16 Добавлена библитека вывода с удлиннёнными состояниями на выходе (lcdslow.inc)
; 08.07.16 Диначеское отображение информации в зависимости от состояния
; 27.07.16 Мелкие исправления в коде
; 31.10.16 Добавлена обработка прерывания INTF
; 02.02.18 Добавлена проверка флага ошибки приёма байт
; 24.05.18 Перенесена функция управления подсветкой из прерывания в основной цикл - это вызывало подвисание дисплея
; 03.11.21 Добавлена возможность работы с универсальной характеристикой ДАД

	list      p=16f628a           ; list directive to define processor
;	#include <C:\Program Files\MPLAB IDE\MCHIP_Tools\p16F628a.inc>       ; processor specific variable definitions
	#include <p16F628a.inc>
	errorlevel  -302              ; suppress message 302 from list file
	#include macro.inc

 __CONFIG _CP_OFF & _WDT_OFF & _BODEN_OFF & _PWRTE_ON & _XT_OSC & _LVP_OFF

;====================================================================
;
;            DISPLAY 16х2 FOR WEGA INJECTION SYSTEM
;
;====================================================================
	
; Переменные
W_TEMP	EQU		07F

	cblock	20

AF
NNN
BBB
CCC
DDD

STAT		; Отображает режим работы (побитовый) [0]-Motor on [1]-Warm_Up on [2]-IAC [3]-Accel [4]-D.Accel [5]-ЭПХХ [6]-ЭМР [7] AFTER START ENR
ERR		; Биты ошибок Bit:[0]-WDT, [1]-BOR, [2]-T.Inj, [3]-Err.Copm, [4]-COLT,[5]-MAP
STPCNT		; Позиция шагового двигателя
A_TMP		; *Показания ДТВ
COLT		; Показания ДТОЖ
MAP		; Разряжение 0FH -Offset, 0FAH - 100КПа
TPS		; Положение дроссельной заслонки <14H IAC
O2S		; Показаняи датчика кислорода
POWER		; Напряжение боротовой сети/10,2
SPD		; Обороты двигателя
T_INJ_L
T_INJ_H		; Время впрыска(мкс)
;
INJ_LAG_L 
INJ_LAG_H

RqFl		; Время впрыска при давлении 110КПа
RqFh
RCO_L	; Время впрыска при пуске на холодную 30ms
RCO_H
MCKl	; ; Счётчик моточасов
MCKh
WARM_80		; Обогащение при прогреве горячего вдигателя (+80)
WARM_20		; Обогащение при прогреве холодного двигателя (-40)

res1 
		;
WARM_40
WARM_50
D_TPS		; Delta TPS
ENR_12

res2		; 
ExtConfig

UOZ		
IAT	

SPD_ENR
WARM_ENR		; ENRICHMENT

	Temp	; (LCD)-флаг busy и текущий адрес DDRAM
	Char	; (LCD)-cохранение при выводе символа или команды
	temp1, temp2
	STATUS_TEMP
	PCLATH_TEMP
	FLAG	; Флаги приёма пакетов [0]- AF ,[1]- 00,[2]-version, [3]-Size
	FLAG2	; [0]- хз, [1]- флаг приятия данных, [2]- флаг обновления дисплея [3] - флаг переноса отображения рхх более 99 [7]- регулировка подсветки
	SIZE	; Размер пакета (получает в посылке)
	BT_CNT	; Сётчик байт приёма
	IN_CNT		; Счётчик приёма
	CHK_SUM_RX	; Контрольная сумма приёма
	Colt1
	SPDCNTh	; Счётчик скорости
	SPDCNTl
	SPEED	; Скорость
           DispEr ; Счётчик ожидания готовности ЖКИ
	BLINK	;РЕГИСТР МЕРЦАНИЯ
	LPG_LEVEL;Уровень газа
	cnt1,cnt2	;регистры программируемой задержки
	rab,rab0,rab1,rab2,rab3,count,aargb2,aargb1,temp
MULc		; 8 bit multiplicand
MULp		; 8 bit multiplier
H_byte		; High byte of the 16 bit result
L_byte		; Low byte of the 16 bit result

APRX_BUFF:16	; Buffer
APRX_B_IND
	endc

; Назначение портов дисплея
LCD_DATA         EQU     PORTB
LCD_DATA_TRIS    EQU     TRISB

#define	E	PORTB,3	;Сигнал управления Е
#define	RW	PORTA,2	;Сигнал управления R/W
#define	RS	PORTA,3	;Сигнал управления RS

#DEFINE	BRIGHT	PORTA,4	;Порт регулировки яркости дисплея

#DEFINE	VERSION	010H	 ; ВЕРСИЯ ПРИНИМАЕМОГО ПАКЕТА(другие не принимает)

;Параметры настройки компараторов для измерения уровня топлива
; VRCON voltage 1010xxxx
#DEFINE	LPG_L	b'10101110'	; >2.9V Обрыв датчика уровня топлива
#DEFINE	LPG_FULL	b'10101010'	; <2.08v
#DEFINE	LPG_L3	b'10100110'	; <1.25v
#DEFINE	LPG_L2	b'10100100'	; <0.83v
#DEFINE	LPG_L1	b'10100010'	; <0.42v
#DEFINE	LPG_NULL	b'10100001'	; <0.21v


	ORG	0		; processor reset vector
	NOP
	GOTO	INIT		; go to beginning of program
;------------------------------------------------------------------
;		ОБРАБОТКА ПРЕРЫВАНИЙ
;------------------------------------------------------------------
	ORG	4		; Вектор прерываний
	MOVWF 	W_TEMP		; Копировать W во временный регистр независимо от текущего банка
	SWAPF 	STATUS,W		; Обменять полубайты в регистре STATUS и записать в W		
	CLRF 	STATUS		;
	MOVWF 	STATUS_TEMP 	; Сохранить STATUS во временном регистре банка 0
	MOVF 	PCLATH, W 		;
	MOVWF 	PCLATH_TEMP		;
	CLRF 	PCLATH 		;
	btfsc	INTCON,INTF		;импульс датчика скорости
	goto	WHEEL
;	BTFSC	PIR1,CCP1IF		;
;	GOTO	COMP_EVNT
	BTFSC	PIR1,TMR1IF		; Проверка переполнение таймера ТМР1
	GOTO	TMR_OF
	BTFSC	INTCON,T0IF
	GOTO	TMR_EV 		; Проверка переполнение таймера ТМР0
	BTFSC 	PIR1,RCIF    	; USART Rx?
	GOTO 	Int_Rx
	GOTO 	RET
;------------------------ INT exit-----------------------
RET 	MOVF	PCLATH_TEMP, W 	;
	MOVWF	PCLATH 		;
	SWAPF	STATUS_TEMP,W 	;Обменять полубайты оригинального значения STATUS и записать в W (восстановить текущий банк)
	MOVWF	STATUS		;Восстановить значение STATUS из регистра W
	SWAPF	W_TEMP,F		;Обменять полубайты в регистре W_TEMP и сохранить результат в W_TEMP
	SWAPF	W_TEMP,W		;Обменять полубайты в регистре W_TEMP и восстановить оригинальное значение W без воздействия на STATUS
	RETFIE


;------------------------------------------------------------------
;			ИНИЦИАЛИЗАЦИЯ
;------------------------------------------------------------------
INIT	bank1
	let	TRISA,b'00000011'
	let	TRISB,b'00000011'
	let	OPTION_REG,b'10000111'	; Подтягивающие резисторы включены(Предделитель на tmr0)
	let	PIE1,B'00100001' 	; Записать маску прерываний в регистр PIE1
	let	VRCON,B'10100001'
; Настройка УСАРТ
	MOVLW	.25        		; Установить скорость обмена данными(F=9615 Hz)
	MOVWF	SPBRG
	MOVLW	B'00100100'  		; Передача 8 - разрядных данных, включить передатчик,
 	MOVWF	TXSTA		; ВЫСОКОСКОРОСТНОЙ асинхронный режим

	;BSF      PIE1,TXIE,0		; Разрешить прерывания от передатчика USART
  	BSF      PIE1,RCIE		; Разрешить прерывания от приемника USART    


	bank0
	CLRWDT
  	MOVLW	B'10010000'		; РАБОТА ПОРТА РАЗРЕШЕНА 8 БИТ ДАННЫХ
	MOVWF	RCSTA		; включить модуль USART
	let 	CMCON,b'00000010'	; Настраиваем компараторы 
	CLRF	PORTB
	CLRF	PORTA
	CLRF	TMR0
	CLRF	FLAG
	CLRF	FLAG2
	CLRF	BT_CNT
	CLRF	CHK_SUM_RX
	CLRF	ERR
	CLRWDT
	let	LPG_LEVEL,.7
;	let 	INTCON,B'11010000'	; ГЛОБАЛЬНЕО РАЗРЕШЕНИЕ ПРЕРЫВАНИЙ
	let	T1CON,B'00000000'	; Инициализация ТМР1, предделитель 1:8
	let 	INTCON,B'11110000'	; ГЛОБАЛЬНЕО РАЗРЕШЕНИЕ ПРЕРЫВАНИЙ

	CALL	LCD_Init        ; Инициализация дисплея

	CLRWDT	
	MOVLW	CLR_DISP	;
	CALL	Send_Cmd
	MOVLW	DISP_ON	
	CALL	Send_Cmd
	MOVLW	ENTRY_INC	
	CALL	Send_Cmd
;
	CLRWDT

;	CALL 	Wait_Busy	; Чтение текущего адреса ОЗУ ЖКИ
	CALL	SET_CGRAM
	CLRWDT	
	MOVLW	01	;Очистка дисплея
	CALL	Send_Cmd
	MOVLW	LINE1
	CALL	Send_Cmd


	let	temp1,.15
	MOVLW	HIGH LOGO
	MOVWF	PCLATH
	MOVLW	LOW LOGO
	MOVWF	temp2
	CALL	READ_STR
	CALL	sch_b
	INCF	temp2,F
	DECFSZ	temp1,F
	GOTO	$-4



NN	MOVLW	LINE2
	CALL	Send_Cmd
	CLRWDT



	let	temp1,.5
	MOVLW	HIGH N_CON
	MOVWF	PCLATH
	MOVLW	LOW N_CON
	MOVWF	temp2
	CALL	READ_STR
	CALL	sch_b
	INCF	temp2,F
	DECFSZ	temp1,F
	GOTO	$-4

 CALL SPEEDW

	CLRWDT
	GOTO	FUEL




CC 	BTFSC	FLAG2,1
	GOTO	CCNXT
;	CALL	DELAY
	GOTO	NN
CCNXT	CLRWDT
	BTFSS	FLAG2,2
	GOTO	CC
	BCF	FLAG2,2
;MOVLW	01	;Очистка дисплея
;	call	Send_Cmd


	MOVLW	LINE1
	CALL	Send_Cmd

	MOVLW	01H
	BTFSC	STAT,2	;ХХ
	MOVLW	00H
	BTFSC	STAT,3	;акселератор
	MOVLW	0D9H
	BTFSC	STAT,4	;деакселератор
	MOVLW	0DAH
	BTFSC	STAT,5	; ЭПХХ
	MOVLW	03H
	BTFSC	STAT,6	; ЭМР
	MOVLW	02H
	BTFSS	STAT,0	; проверка пуска двигателя
	GOTO	$+3
	BTFSC	STAT,7	; Прогрев после пуска показывает только на заведённом двигателе
	MOVLW	0EDH
	CALL	sch_b


;----------------------------------------------
;	Вывод тахометра
;----------------------------------------------
	Movfw	SPD
	MOVWF	MULc
	let	MULp,.25
	CALL	MUL8

	Movfw	L_byte
	MOVWF	aargb2
	Movfw	H_byte
	MOVWF	aargb1
	CALL	BCD16	;Преобразование к десятичному виду


	MOVF	rab0,w	;Тысячи на дисплей
	ADDLW	30
	CALL	sch_b
	MOVF	rab1,w	;Сотни на дисплей
	ADDLW	30
	CALL	sch_b
	MOVF	rab2,w	;Десятки на дисплей
	ADDLW	30
	CALL	sch_b
	MOVF	rab3,w	;Единицы на дисплей
	ADDLW	30
	CALL	sch_b

;	movlw	020H
;	call	sch_b
;-----------------------------------------
;	вывод температуры
;-----------------------------------------
	BTFSS	ERR,4	;Проверяем ошибу ДТОЖ
	GOTO	CLT
	BTFSS	BLINK,3	; РЕГИСТР МОРГАНИЯ
	GOTO	CLT
	MOVLW	20	; ОЧИЩАЕМ ЗНАКОМЕСТА
	CALL	sch_b
	MOVLW	20
	CALL	sch_b
	MOVLW	20
	CALL	sch_b
	MOVLW	20
	CALL	sch_b
	GOTO	CLT_EX	; 


CLT
	MOVLW	.44	; отрицательные температуры 45
	SUBWF	A_TMP,W
	MOVWF	Colt1
	MOVLW	20
	BTFSC	STATUS,C
	GOTO	$+5

	Movfw	Colt1
	SUBLW	.0
	MOVWF	Colt1
	MOVLW	'-'	; -

	CALL	sch_b
	Movfw	Colt1
	MOVWF	aargb2
	CLRF	aargb1	
	CALL	BCD16	;Преобразование к десятичному виду

	MOVF	rab2,w	;Сотни на дисплей
	ADDLW	30
	CALL	sch_b
	MOVF	rab3,w	;Десятки на дисплей
	ADDLW	30
	CALL	sch_b
	
	MOVLW	04H	; градусы
	CALL	sch_b
CLT_EX

;----------------------------------------------
;	Вывод напряжения на дисплей
;----------------------------------------------
	Movfw	POWER
	MOVWF	aargb2
	CLRF	aargb1

conv	CALL	BCD16	;Преобразование к десятичному виду

	MOVF	rab1,w	;Тысячи на дисплей
	ADDLW	30
	CALL	sch_b
	MOVF	rab2,w	;Сотни на дисплей
	ADDLW	30
	CALL	sch_b

	BTFSC	FLAG2,3	; Флаг больших шагов РХХ, освобождаем знакоместо
	GOTO	$+6
	MOVLW	','	; ЗПТ
	CALL	sch_b
	MOVF	rab3,w	;Десятки на дисплей
	ADDLW	30
	CALL	sch_b

	MOVLW	07H
	CALL	sch_b


;----------------------------------------------
;	Вывод позиции РХХ
;----------------------------------------------

	Movfw	STPCNT
	MOVWF	aargb2
	CLRF	aargb1
	CALL	BCD16	;Преобразование к десятичному виду


	MOVF	rab1,w	;Тысячи на дисплей
	BTFSC	STATUS,Z
	GOTO	$+5
	ADDLW	30
	CALL	sch_b
	BSF	FLAG2,3	; Флаг переноса, число шаков более 99
	GOTO	$+2
	BCF	FLAG2,3	;
	MOVF	rab2,w	;Сотни на дисплей
	ADDLW	30
	CALL	sch_b
	MOVF	rab3,w	;Десятки на дисплей
	ADDLW	30
	CALL	sch_b

	MOVLW	0C1h
	CALL	sch_b
;	MOVLW	020H
;	CALL	sch_b
; ВТОРАЯ ЛИНИЯ
	CLRWDT
	MOVLW	LINE2
	CALL	Send_Cmd
	BTFSC	STAT,0	;пуск двигателя
	GOTO	SCR1
;вывод моточасов
	Movfw	MCKl
	MOVWF	aargb2
	Movfw	MCKh
	MOVWF	aargb1
	CALL	BCD16	;Преобразование к десятичному виду

	MOVLW	'M'
	CALL	sch_b	
	MOVLW	'o'
	CALL	sch_b	
	MOVLW	't'
	CALL	sch_b	
	MOVLW	'o'
	CALL	sch_b	
	MOVLW	':'
	CALL	sch_b	
	MOVF	rab,w	;Десятки Тысяч на дисплей
	ADDLW	30
	CALL	sch_b
	MOVF	rab0,w	;Тысячи на дисплей
	ADDLW	30
	CALL	sch_b
	MOVF	rab1,w	;Сотни на дисплей
	ADDLW	30
	CALL	sch_b
	MOVF	rab2,w	;Десятки на дисплей
	ADDLW	30
	CALL	sch_b
	MOVF	rab3,w	;Единицы на дисплей
	ADDLW	30
	CALL	sch_b
	MOVLW	'h'
	CALL	sch_b	
	MOVLW	20
	CALL	sch_b
	MOVLW	'E'
	CALL	sch_b	
	CALL HEXWIEV	; Вывод кода ошибки EFI

	MOVLW	20
	CALL	sch_b	


 GOTO FUEL


SCR1	MOVLW	06
	CALL	sch_b	
	
;-------------------------
; Вывод показания ДАД
;-------------------------
 	BTFSS	ERR,5	;Проверяем ошибу ДАД
	GOTO	$+7
	BTFSS	BLINK,3	; РЕГИСТР МОРГАНИЯ
	GOTO	$+5
	MOVLW	20	; ОЧИЩАЕМ ЗНАКОМЕСТА
	CALL	sch_b
	MOVLW	20
	GOTO	MAR_EX	; ВТОРОЕ ЗНАКОМЕСТО ОЩИШАЕТСЯ ПОСЛЕ ПЕРХОДА
	Movfw	MAP
;!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
;	BTFSS	ExtConfig,0
;	GOTO	OLDMAP
	ADDLW	-.16
	BTFSS	STATUS,C
	GOTO	MAP_OFF	
	MOVLW	HIGH MAP_Trans2
	MOVWF	PCLATH
	GOTO	MAP_Tr	
	
OLDMAP	ADDLW	-.16
	ADDLW	-(.230-.16+.1)
	BTFSC	STATUS,C
	GOTO	MAP_OFF
	MOVLW	HIGH MAP_Trans
	MOVWF	PCLATH
MAP_Tr	MOVLW	.16
	SUBWF	MAP,W
	MOVWF	temp2
	CALL	READ_STR
	MOVWF	aargb2
	CLRF	aargb1
	CALL	BCD16	;Преобразование к десятичному виду
	MOVF	rab2,w	;Сотни на дисплей
	ADDLW	30
	CALL	sch_b
	MOVF	rab3,w	;Десятки на дисплей
	ADDLW	30
	CALL	sch_b
	GOTO	$+5
MAP_OFF	MOVLW	"-"
	CALL	sch_b
	MOVLW	"-"
MAR_EX	CALL	sch_b

	MOVLW	"K"
	CALL	sch_b
	MOVLW	0BEH
	CALL	sch_b

	MOVLW	020
	CALL	sch_b

	BTFSS	STAT,1
	GOTO	VEH
;-----------------------------------------------
;	обогащение на прогреве
;-----------------------------------------------
	bank1
	Movfw	WARM_ENR
	bank0
	MOVWF	MULc
	let	MULp,.100
	CALL	MUL8

	CLRF	aargb1
	RLF	L_byte,W
	RLF	H_byte,W
	MOVWF	aargb2


	CALL	BCD16	;Преобразование к десятичному виду



	MOVF	rab1,w	;Сотни на дисплей
	ADDLW	30
	CALL	sch_b
	MOVF	rab2,w	;Десятки на дисплей
	ADDLW	30
	CALL	sch_b
	MOVF	rab3,w	;Единицы на дисплей
	ADDLW	30
	CALL	sch_b

	MOVLW	025H
	CALL	sch_b
	MOVLW	020H
	CALL	sch_b
	GOTO	ECON
;-----------------------------------------------
;	обогащение по VE
;-----------------------------------------------

VEH	MOVLW	020
	CALL	sch_b
	bank1
	Movfw	SPD_ENR
	bank0
	MOVWF	MULc
	let	MULp,.100
	CALL	MUL8

	CLRF	aargb1
;	RLF	L_byte,W
;	RLF	H_byte,W
	Movfw	H_byte
	MOVWF	aargb2
	CALL	BCD16	;Преобразование к десятичному виду

	MOVF	rab2,w	;Десятки на дисплей
	ADDLW	30
	CALL	sch_b

	MOVF	rab3,w	;Единицы на дисплей
	ADDLW	30
	CALL	sch_b

	MOVLW	025H
	CALL	sch_b

	MOVLW	020
	CALL	sch_b

;	MOVLW	020H
;	CALL	sch_b
;test
; CALL HEXWIEV
; Вывод расхода топлива
; умножаем обороты на время впрыска и считаем по таблице
; старшый битт в таблице - точка
ECON	Movfw	SPD
	MOVWF	MULc
	RLF 	T_INJ_L,W
	MOVWF	temp1
	RLF	T_INJ_H,W
	MOVWF	temp2
	RLF	temp1,W
	RLF	temp2,W


	MOVWF	MULp
	CALL	MUL8
;	MOVLW	HIGH Fuel
;	MOVWF	PCLATH
;	RLF	L_byte,W
;	RLF	H_byte,W
;	CALL	READ_STR+1

; добавляю новый расчёт
	
	RLF	L_byte,F
	RLF	H_byte,F
	RLF	L_byte,W
	RLF	H_byte,W



;	MOVWF	temp2
;	BTFSC	temp2,7
;	XORLW	B'10000000'
	MOVWF	aargb2
	CLRF	aargb1
	CALL	BCD16	;Преобразование к десятичному виду


	MOVF	rab1,F
	BTFSC	STATUS,Z
	GOTO	DESL
	MOVF	rab1,w	;Десятки на дисплей
	ADDLW	30
	CALL	sch_b
	MOVF	rab2,w	;Десятки на дисплей
	ADDLW	30
	CALL	sch_b
	GOTO	LTR





;	BTFSS	temp2,7
;	GOTO	$+3
;	MOVLW	"."
;	GOTO	$+3
DESL	MOVF	rab2,w	;Десятки на дисплей
	ADDLW	30
	CALL	sch_b
	MOVLW	"."
	CALL	sch_b
	MOVF	rab3,w	;Единицы на дисплей
	ADDLW	30
	CALL	sch_b



LTR	MOVLW	0BBH	; Знак литры
	CALL	sch_b
		


	MOVF	rab1,F
	BTFSC	STATUS,Z
	GOTO	FUEL
	MOVLW	020
	CALL	sch_b
; Опознавание включения подсветки и регулировки яркости дисплея
FUEL	; Определение включения освещения
	bank1
	let	VRCON,B'10100011'
	bank0
	CALL	delay5
	BTFSC	CMCON,C1OUT
	BSF	FLAG2,7;
	BTFSS	CMCON,C1OUT
	BCF	FLAG2,7;

; ПРОВЕРКА НА ОБРЫВ ДАТЧИКА УРОВНЯ
	bank1
	let	VRCON,LPG_L
	bank0
	CALL	delay5
	BTFSC	CMCON,C2OUT
	GOTO	$+4
	MOVLW	.0
	MOVWF	LPG_LEVEL	
	GOTO	LEVL


	bank1
	let	VRCON,LPG_FULL
	bank0
	CALL	delay5
	BTFSC	CMCON,C2OUT
	GOTO	$+4
	MOVLW	.6
	MOVWF	LPG_LEVEL	
	GOTO	LEVL




	bank1
	let	VRCON,LPG_L3
	bank0
	CALL	delay5
	BTFSC	CMCON,C2OUT
	GOTO	$+4
	MOVLW	.5
	MOVWF	LPG_LEVEL	
	GOTO	LEVL

	bank1
	let	VRCON,LPG_L2
	bank0
	CALL	delay5
	BTFSC	CMCON,C2OUT
	GOTO	$+4
	MOVLW	.4
	MOVWF	LPG_LEVEL	
	GOTO	LEVL

	bank1
	let	VRCON,LPG_L1
	bank0
	CALL	delay5
	BTFSC	CMCON,C2OUT
	GOTO	$+4
	MOVLW	.3
	MOVWF	LPG_LEVEL	
	GOTO	LEVL

	bank1
	let	VRCON,LPG_NULL
	bank0
	CALL	delay5
	BTFSC	CMCON,C2OUT
	GOTO	$+4
	MOVLW	.2
	MOVWF	LPG_LEVEL	
	GOTO	LEVL

	MOVLW	.1
	MOVWF	LPG_LEVEL


LEVL	
; Моргание дисплея при ошибках
	MOVF	ERR,W
	ANDLW	B'00110000'
	BTFSC	STATUS,Z
	GOTO	BRGHT
;	BTFSS	BLINK,3
;	GOTO	BR_UP
;	GOTO	BR_DWN

; Регулировка яркости
BRGHT	BTFSS	FLAG2,7;CMCON,C1OUT
	GOTO	BR_DWN
BR_UP	BTFSS	BRIGHT
	BSF	BRIGHT
	GOTO	BR_NEXT
BR_DWN	BTFSC	BRIGHT
	BCF	BRIGHT

BR_NEXT	;BCF	BRIGHT test port


	DECFSZ	LPG_LEVEL,W	;LPG Empty?
	GOTO	LPG_CHAR_S
	BTFSS	BLINK,3	; РЕГИСТР МОРГАНИЯ
	GOTO	LPG_CHAR_S
	MOVLW	20	; ОЧИЩАЕМ ЗНАКОМЕСТА
	CALL	sch_b
	GOTO	CC
	

LPG_CHAR_S	MOVLW	05H	;Код знака топлива
	CALL	sch_b
	CALL	LPG_CHAR	;Обвновление знакогенератора

	BTFSS	RCSTA,OERR	;Проверка ошибки принятия байт
	GOTO	CC
	bcf	RCSTA,CREN
	BSF	RCSTA,CREN
	GOTO CC
;=============================================================
;	TMR 0
;=============================================================
TMR_EV	BCF	INTCON,T0IF		; Сброс флага прерывания

	DECF	BLINK,F

	DECFSZ	IN_CNT,F		; Счётчик приёма
	GOTO	RET
	BCF	FLAG2,1   ; CLRF	FLAG2 заменим на изменение соответсвующего флага

	GOTO	CHSM_ERR



; =============================================================
; 	Обработка датчика скорости
;==============================================================
WHEEL	bcf	T1CON,TMR1ON	;выключаем таймер
	BCF	INTCON,INTF
	movf	TMR1L,W
	movwf	SPDCNTl
	movf	TMR1H,W
	movwf	SPDCNTh		;запоминаем значение таймера

	CLRF	TMR1L
	CLRF	TMR1H
	BSF	T1CON,TMR1ON	; Включаем таймер


	GOTO	RET
;=============================================================
;	TMR 1
;=============================================================
TMR_OF	BCF	T1CON,TMR1ON	; Останавливаем таймер
	BCF	PIR1,TMR1IF		; Сбрасываем флаг прерывания
	CLRF	TMR1L
	CLRF	TMR1H
	GOTO	RET



; =============================================================
; 	Приём данных с COM-порта
;==============================================================
Int_Rx	BCF	PIR1,RCIF

	Movfw	RCREG
;	MOVWF	TXREG
;	bank1 
;	BTFSS    	TXSTA,TRMT		; ПЕРЕДАЧА 
;	GOTO    	 $-1		; ДАННЫХ
;	bank0



	BTFSC	FLAG,3		; Привём данных
	GOTO	READ_DATA
	BTFSC	FLAG,2		; 
	GOTO	GET_SIZE
	BTFSC	FLAG,1
	GOTO	GET_VERSION
	BTFSC	FLAG,0
	GOTO	NUL
	INCF	BT_CNT,F
	Movfw	RCREG		; проверка на байт синхронизации
	ADDWF	CHK_SUM_RX,F	; считаем контрольную сумму
	XORLW	0AFH
	BTFSS	STATUS,Z
	GOTO	CHSM_ERR
	BSF	FLAG,0
	GOTO	RET

NUL	Movfw	RCREG		; ПРОВЕРЯЕМ ВТОРОЙ БАЙТ ПАКЕТА
	ADDWF	CHK_SUM_RX,F	; считаем контрольную сумму
	XORLW	01H
	BTFSS	STATUS,Z
	GOTO	CHSM_ERR
	INCF	BT_CNT,F
	BSF	FLAG,1
	GOTO	RET


GET_VERSION	Movfw	RCREG
	ADDWF	CHK_SUM_RX,F	; считаем контрольную сумму	
	
	SUBLW	VERSION
	BTFSS	STATUS,Z
	GOTO	CHSM_ERR

	INCF	BT_CNT,F
	BSF	FLAG,2
	GOTO	RET

GET_SIZE	Movfw	RCREG
	ADDWF	CHK_SUM_RX,F	; считаем контрольную сумму
	MOVWF	SIZE
	SUBLW	.64		; ОГРАНИЧЕНИЕ НА МАКСИМАЛЬНУЮ ДЛИННУ ПАКЕТА
	BTFSS	STATUS,C
	GOTO	CHSM_ERR
	MOVLW	.5		; ПРОВЕРКА НА МИНИМАЛЬНЫЙ РАЗМЕР ПАКЕТА
	SUBWF	SIZE,W	
	BTFSS	STATUS,C
	GOTO	CHSM_ERR



	INCF	BT_CNT,F
	
	BSF	FLAG,3
	GOTO	RET

READ_DATA	INCF	BT_CNT,F
	MOVF	BT_CNT,W		; КОЛИЧЕСТВО БАЙТ В ПАКЕТЕ +1
	XORWF	SIZE,W		;
	BTFSC	STATUS,Z		; ПРОВЕРЯЕМ НА ПЕРЕПОЛНЕНИЕ
	GOTO	CHK_SUM		; ОБНУЛЕНИЕ СЧЁТЧИКА И КОНТРОЛЬНОЙ СУММЫ


	movab	BT_CNT,FSR		; ЗАГРУЖАЕМ СЧЁТЧИК БАЙТ В РЕГИСТР КОСВЕННОЙ АДРЕСАЦИИ	
	MOVLW	0A0H		; Добавляем начало блока переменных
	ADDWF	FSR,F		; Приращаем FSR на начальное значение
	Movfw	RCREG
	ADDWF	CHK_SUM_RX,F	; считаем контрольную сумму
	bank1
	MOVWF	INDF		; Сохраняем во временный буфер памяти
	bank0

	let	IN_CNT,0FFH		;СЧЁТЧИК ПРИЁМА
	BSF	FLAG2,0

	GOTO	RET
; Проверяем контрольную сумму
CHK_SUM	Movfw	RCREG
	XORWF	CHK_SUM_RX,W
	BTFSS	STATUS,Z
	GOTO	CHSM_ERR
	BSF	FLAG2,2
	BSF	FLAG2,1  ;Флаг принятых данных
	let	IN_CNT,0FFH
; Копируем данные в рабочие регистры
	bank1
	Movfw	STAT
	bank0
	MOVWF	STAT
	bank1
	Movfw	ERR
	bank0
	MOVWF	ERR
	bank1
	Movfw	A_TMP
	bank0
	MOVWF	A_TMP
	bank1
	Movfw	COLT
	bank0
	MOVWF	COLT
	bank1
	Movfw	SPD
	bank0
	MOVWF	SPD
	bank1
	Movfw	MAP
	bank0
	MOVWF	MAP
	bank1
	Movfw	STPCNT
	bank0
	MOVWF	STPCNT
	bank1
	Movfw	O2S
	bank0
	MOVWF	O2S
	bank1
	Movfw	POWER
	bank0
	MOVWF	POWER
	bank1
	Movfw	T_INJ_L
	bank0
	MOVWF	T_INJ_L
	bank1
	Movfw	T_INJ_H
	bank0
	MOVWF	T_INJ_H
	bank1
	Movfw	MCKl
	bank0
	MOVWF	MCKl
	bank1
	Movfw	MCKh
	bank0
	MOVWF	MCKh

CHSM_ERR	
	CLRF	FLAG
	CLRF	CHK_SUM_RX
	CLRF	BT_CNT
	GOTO	RET
;=======================================================================================
READ_STR	MOVF	temp2,W		; Чтение таблицы DT
	MOVWF	PCL


delay20	MOVLW	10
	MOVWF	cnt1
	CLRF	cnt2
	CLRWDT
	DECFSZ	cnt2,f
	GOTO	$+2
	DECFSZ	cnt1,f
	GOTO	$-4
	RETURN

delay5	CLRWDT
	MOVLW	4
	MOVWF	cnt1
	CLRF	cnt2
	DECFSZ	cnt2,f
	GOTO	$+2
	DECFSZ	cnt1,f
	GOTO	$-3
	NOP
	RETURN

DELAY	MOVLW	.255
	MOVWF	cnt1
	CLRF	cnt2
	DECFSZ	cnt2,f
	GOTO	$+3
	CLRWDT
	DECFSZ	cnt1,f
	GOTO	$-4
	RETURN

d100mks	let temp2,.25
dll	NOP			;4 x 50 = 100 mkc
	DECFSZ temp2,f
	GOTO dll
	RETURN

;****************************************************
; Преобразование двухбайтного числа в десятичное
;****************************************************
BCD16   	BCF     STATUS,0        
	MOVLW   .16
	MOVWF   count
	CLRF    rab
	CLRF    rab1
	CLRF    rab2
	CLRF    rab3
loop16  	RLF     aargb2, F
	RLF     aargb1, F
	RLF     rab3, F
	RLF     rab2, F
	RLF     rab1, F
	DECFSZ  count, F
	GOTO    adjDEC


	Movfw	rab1		; добавил распаковку десяток тысяч
	ANDLW	0FH
	MOVWF	rab
	Movfw	rab2		; добавил распаковку тысяч
	ANDLW	0F0
	MOVWF	rab0
	SWAPF	rab0,f

	Movfw	rab2		; Распаковка BCD
	ANDLW	0F
	MOVWF	rab1
	Movfw	rab3
	ANDLW	0F0
	MOVWF	rab2
	SWAPF	rab2,f
	Movfw	rab3
	ANDLW	0F
	MOVWF	rab3
	RETLW   0

adjDEC  	MOVLW   rab3
	MOVWF   FSR
	CALL    adjBCD
	MOVLW   rab2
	MOVWF   FSR
	CALL    adjBCD
	MOVLW   rab1
	MOVWF   FSR
	CALL    adjBCD
	GOTO    loop16
adjBCD  	MOVLW   3
	ADDWF   0,W
	MOVWF   temp
	BTFSC   temp,3          ; test if result > 7
	MOVWF   0
	MOVLW   30
	ADDWF   0,W
	MOVWF   temp
	BTFSC   temp,7          ; test if result > 7
	MOVWF   0               ; save as MSD
	RETLW   0



;-----------------------------------------------------------------------
;           Библиотека символов знакогенератора и сообщений
;-----------------------------------------------------------------------
	#include MUL8X8.ASM
	#include lcdrout.inc
;	#include lcdslow.inc ; Вариант с увеличенными задержками между состояниями линий
	ORG	400
LOGO	DT	20, 0AFH,"C", 0A9H, 0E0H,20,"WEGA",20,"2015",20
N_CON	DT	20,"n/c       "
; CGRAM
SYM_DOT	DT	.128,.128,.140,.158,.158,.140,.128,.128	; 00H Символ ХХ (точка)
SYM_O	DT	.128,.132,.142,.145,.145,.142,.132,.128	; 01H Символ рабочего режима (кружок)
SYM_BO	DT	.142,.159,.155,.145,.145,.155,.159,.142	; 02H
SYM_L	DT	.128,.159,.128,.159,.128,.159,.128,.128	; 03H
SYM_DEG	DT	.140,.146,.146,.140,.128,.128,.128,.128 ; 04H Знак градуса
SYM_FUEL	DT	.142,.159,.145,.145,.145,.145,.145,.159 ; Уровень топлива
SYM_LAMBD	DT	.128,.128,.128,.159,.159,.128,.128,.128 ; Указатель лямбда-зонда
SYM_V	DT	.128,.128,.128,.128,.148,.148,.136,.128 ; Знак маленькая V - напряжение бортсети
;SYM_F_ER	DT	.142,.159,.142,.149,.155,.149,.142,.159 ; Знак обрыва датчика уровня топлива (при инициализации не загружается. а заменяет знак уровня топлива)
SYM_F_ER	DT	.14,.31,.113,.106,.4,.106,.113,.159 ; Знак обрыва датчика уровня топлива (при инициализации не загружается. а заменяет знак уровня топлива)


;-----------------------------------------------------------------------
;           Таблица преобразования показаний ДАД
;-----------------------------------------------------------------------
; MAP sensor MPX4100 pressure=ОКРУГЛ((B2/5+0,152)/0,01059;0)
	ORG	500
MAP_Trans	DT	.20,.21,.21,.21,.22,.22,.22,.23,.23,.24
	DT	.24,.24,.25,.25,.25,.26,.26,.27,.27,.27
	DT	.28,.28,.28,.29,.29,.30,.30,.30,.31,.31
	DT	.31,.32,.32,.32,.33,.33,.34,.34,.34,.35
	DT	.35,.35,.36,.36,.37,.37,.37,.38,.38,.38
	DT	.39,.39,.40,.40,.40,.41,.41,.41,.42,.42
	DT	.42,.43,.43,.44,.44,.44,.45,.45,.45,.46
	DT	.46,.47,.47,.47,.48,.48,.48,.49,.49,.50
	DT	.50,.50,.51,.51,.51,.52,.52,.52,.53,.53
	DT	.54,.54,.54,.55,.55,.55,.56,.56,.57,.57
	DT	.57,.58,.58,.58,.59,.59,.60,.60,.60,.61
	DT	.61,.61,.62,.62,.62,.63,.63,.64,.64,.64
	DT	.65,.65,.65,.66,.66,.67,.67,.67,.68,.68
	DT	.68,.69,.69,.70,.70,.70,.71,.71,.71,.72
	DT	.72,.72,.73,.73,.74,.74,.74,.75,.75,.75
	DT	.76,.76,.77,.77,.77,.78,.78,.78,.79,.79
	DT	.80,.80,.80,.81,.81,.81,.82,.82,.82,.83
	DT	.83,.84,.84,.84,.85,.85,.85,.86,.86,.87
	DT	.87,.87,.88,.88,.88,.89,.89,.90,.90,.90
	DT	.91,.91,.91,.92,.92,.92,.93,.93,.94,.94
	DT	.94,.95,.95,.95,.96,.96,.97,.97,.97,.98
	DT	.98,.98,.99,.99,.99
	ORG	600
MAP_Trans2	DT	.13,.13,.14,.15,.16,.16,.17,.18,.19,.20,.20,.21,.22,.23,.23,.24,.25,.26,.27,.27,.28,.29,.30
	DT	.30,.31,.32,.33,.34,.34,.35,.36,.37,.38,.38,.39,.40,.41,.41,.42,.43,.44,.45,.45,.46,.47,.48
	DT	.48,.49,.50,.51,.52,.52,.53,.54,.55,.55,.56,.57,.58,.59,.59,.60,.61,.62,.63,.63,.64,.65,.66
	DT	.66,.67,.68,.69,.70,.70,.71,.72,.73,.73,.74,.75,.76,.77,.77,.78,.79,.80,.80,.81,.82,.83,.84
	DT	.84,.85,.86,.87,.88,.88,.89,.90,.91,.91,.92,.93,.94,.95,.95,.96,.97,.98,.98,.99,.100,.101,.102
	DT	.102,.103,.104,.105,.105,.106,.107,.108,.109,.109,.110,.111,.112,.113,.113,.114,.115,.116,.116
	DT	.117,.118,.119,.120,.120,.121,.122,.123,.123,.124,.125,.126,.127,.127,.128,.129,.130,.130,.131
	DT	.132,.133,.134,.134,.135,.136,.137,.138,.138,.139,.140,.141,.141,.142,.143,.144,.145,.145,.146
	DT	.147,.148,.148,.149,.150,.151,.152,.152,.153,.154,.155,.155,.156,.157,.158,.159,.159,.160,.161
	DT	.162,.163,.163,.164,.165,.166,.166,.167,.168,.169,.170,.170,.171,.172,.173,.173,.174,.175,.176
	DT	.177,.177,.178,.179,.180,.180,.181,.182,.183,.184,.184,.185,.186,.187,.188,.188,.189,.190,.191
	DT	.191,.192,.193,.194,.195,.195,.196,.197,.198,.198,.199

;-----------------------------------------------------------------------
;           Таблица преобразования показаний расхода топлива
;-----------------------------------------------------------------------
;	ORG	700
;Fuel
; include F_485.asm




; Вывод двоичных символов (для отладки)
HEXWIEV	let	temp1,39
	SWAPF	ERR,W
	ANDLW	0FH
	ADDLW	30
	SUBWF	temp1,F
	BTFSS	STATUS,C
	ADDLW	7

	CALL	sch_b
	let	temp1,39
	Movfw	ERR
	ANDLW	0FH
	ADDLW	30
	SUBWF	temp1,F
	BTFSS	STATUS,C
	ADDLW	7
	CALL	sch_b
	RETURN
;=============================
; Вывод скорости
;============================
SPEEDW	Movfw	SPDCNTl
	MOVWF	aargb2
	Movfw	SPDCNTh
	MOVWF	aargb1
	CALL	BCD16	;Преобразование к десятичному виду

	MOVF	rab,w	;Десятки Тысяч на дисплей
	ADDLW	30
	CALL	sch_b
	MOVF	rab0,w	;Тысячи на дисплей
	ADDLW	30
	CALL	sch_b
	MOVF	rab1,w	;Сотни на дисплей
	ADDLW	30
	CALL	sch_b
	MOVF	rab2,w	;Десятки на дисплей
	ADDLW	30
	CALL	sch_b
	MOVF	rab3,w	;Единицы на дисплей
	ADDLW	30
	CALL	sch_b
	MOVLW	20
	CALL	sch_b
	MOVLW	20
	CALL	sch_b
	MOVLW	20
	CALL	sch_b
	MOVLW	20
	CALL	sch_b
	RETURN

APROXIMIZER

	movab	APRX_B_IND,FSR	; ЗАГРУЖАЕМ СЧЁТЧИК БАЙТ В РЕГИСТР КОСВЕННОЙ АДРЕСАЦИИ	
	MOVLW	0A0H		; Добавляем начало блока переменных
	ADDWF	FSR,F		; Приращаем FSR на начальное значение
	Movfw	RCREG
	ADDWF	CHK_SUM_RX,F	; считаем контрольную сумму
	bank1
	MOVWF	INDF		; Сохраняем во временный буфер памяти
	bank0







	END
