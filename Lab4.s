;------------------ Laboratorio 4 ---------------------
;Universidad del Valle de Guatemala
;Autor: Santiago José Rivera Lemus
;Carné: 20269
;Programación de microcontroladores
;------------------------------------------------------
    
PROCESSOR 16F887

; PIC16F887 Configuration Bit Settings

; Assembly source line config statements

; CONFIG1
  CONFIG  FOSC = INTRC_NOCLKOUT ; Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
  CONFIG  WDTE = OFF            ; Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
  CONFIG  PWRTE = ON            ; Power-up Timer Enable bit (PWRT enabled)
  CONFIG  MCLRE = OFF           ; RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
  CONFIG  CP = OFF              ; Code Protection bit (Program memory code protection is disabled)
  CONFIG  CPD = OFF             ; Data Code Protection bit (Data memory code protection is disabled)
  CONFIG  BOREN = OFF           ; Brown Out Reset Selection bits (BOR disabled)
  CONFIG  IESO = OFF            ; Internal External Switchover bit (Internal/External Switchover mode is disabled)
  CONFIG  FCMEN = OFF           ; Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
  CONFIG  LVP = ON              ; Low Voltage Programming Enable bit (RB3/PGM pin has PGM function, low voltage programming enabled)

; CONFIG2
  CONFIG  BOR4V = BOR40V        ; Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
  CONFIG  WRT = OFF             ; Flash Program Memory Self Write Enable bits (Write protection off)

// config statements should precede project file includes.
#include <xc.inc>
  
;--------------------Macros---------------------------
    R_TMR0 MACRO
    BANKSEL TMR0	    ; cambiamos de banco
    MOVLW   197
    MOVWF   TMR0	    ; configuramos tiempo de retardo
    BCF	    T0IF	    ; limpiamos bandera de interrupción
    ENDM

	
;------------------VARIABLES--------------------------
PSECT udata_bank0
    CONTADOR:   DS 2
    PORT_LEDS:	DS 1
    PORT_U:	DS 1
    PORT_D:	DS 1
    
    
PSECT udata_shr
    W_TEMP:	    DS 1
    STATUS_TEMP:    DS 1
    

PSECT resVect, class=CODE, abs, delta=2
ORG 00h	    ; posición 0000h para el reset
;------------ VECTOR RESET --------------
resetVec:
    PAGESEL MAIN	; Cambio de pagina
    GOTO    MAIN
    
;---------------Vector de Interrupciones--------------------
PSECT intVect,class=CODE, abs, delta=2
ORG 04h    ; posición 04h para interrupciones
	
PUSH:
    MOVWF   W_TEMP	; 
    SWAPF   STATUS, W
    MOVWF   STATUS_TEMP	; Guardamos STATUS
    
ISR:
    BTFSC   T0IF
    CALL    INT_TMR0
    
    
    BTFSC   RBIF
    CALL    INT_PORTB
    
POP:
    SWAPF   STATUS_TEMP, W
    MOVWF   STATUS
    SWAPF   W_TEMP, F
    SWAPF   W_TEMP, W
    RETFIE
 
;----------SUBRUTINAS DE INTERRUPCIÓN---------------
INT_TMR0:
    R_TMR0
    INCF	CONTADOR
    MOVF	CONTADOR, W
    SUBLW	50
    BTFSS	ZERO
    GOTO	RETURN_TMR0
    CLRF	CONTADOR
    INCF	PORT_U
    MOVF	PORT_U, W
    CALL	TABLA
    MOVWF	PORTD
    
    MOVF	PORT_U, W
    SUBLW	10
    BTFSC	STATUS, 2
    CALL	INC
    
    MOVF	PORT_D, W
    CALL	TABLA
    MOVWF	PORTC
    
    
    RETURN

INT_PORTB:
    BANKSEL	PORTA
    BTFSS	PORTB, 0
    INCF	PORT_LEDS
    BTFSS	PORTB, 5
    DECF	PORT_LEDS
    MOVF	PORT_LEDS, W
    MOVWF	PORTA
    BCF		RBIF
    
    
    RETURN
    
INC:
    INCF    PORT_D
    CLRF    PORT_U
    MOVF    PORT_U, W
    CALL    TABLA
    MOVWF   PORTD
    
    MOVF    PORT_D, W
    SUBLW   6
    BTFSC   STATUS, 2
    CLRF    PORT_D
    
    RETURN
    
RETURN_TMR0:
    RETURN

ORG 100h
	
;--------------CONFIGURACION------------------------------
    
MAIN:
    CALL    CONFIG_IO
    CALL    CONFIG_RELOJ
    CALL    CONFIG_TMR0
    CALL    CONFIG_INT
    CALL    CONFIG_IOCB
    
LOOP:
    GOTO LOOP
;------------- SUBRUTINAS --------------------------------
CONFIG_RELOJ:
    BANKSEL OSCCON	    ; cambiamos a banco 1
    BSF	    OSCCON, 0	    ; SCS -> 1, Usamos reloj interno
    BSF	    OSCCON, 6
    BSF	    OSCCON, 5
    BCF	    OSCCON, 4	    ; IRCF<2:0> -> 110 4MHz
    return
   
 CONFIG_IO:
    BANKSEL ANSEL
    CLRF    ANSEL
    CLRF    ANSELH	    ; I/O digitales
    
    BANKSEL TRISD
    CLRF    TRISA
    CLRF    TRISC
    CLRF    TRISD	    ; PORTC como salida
    
    BSF	    TRISB, 0
    BSF	    TRISB, 5
    
    BCF	    OPTION_REG, 7
    BSF	    WPUB, 0
    BSF	    WPUB, 5
    
    BANKSEL PORTA
    CLRF    PORTA
    CLRF    PORTC
    CLRF    PORTD
    RETURN
    
    BANKSEL PORTD
    CLRF    PORTD	    ; Apagamos PORTD
    return
    
CONFIG_TMR0:
    BANKSEL OPTION_REG	    ; cambiamos de banco
    BCF	    T0CS	    ; TMR0 como temporizador
    BCF	    PSA		    ; prescaler a TMR0
    BSF	    PS2
    BSF	    PS1
    BSF	    PS0		    ; PS<2:0> -> 111 prescaler 1 : 256
    R_TMR0
    
    RETURN
 
    
CONFIG_IOCB:
    BANKSEL	TRISA
    BSF	    IOCB, 0
    BSF	    IOCB, 5
    
    BANKSEL PORTA
    MOVF    PORTB, W
    BCF	    RBIF
    
    RETURN
    
 CONFIG_INT:
    BANKSEL INTCON
    BSF GIE
    BSF T0IE
    BSF	RBIE
    BCF T0IF
    BCF	RBIF
    
    return
    

    
org  200h   
;-----------------TABLA----------------------------
 TABLA:
    CLRF    PCLATH
    BSF	    PCLATH, 1
    ANDLW   0x0F
    ADDWF   PCL
    RETLW   00111111B
    RETLW   00000110B
    RETLW   01011011B
    RETLW   01001111B
    RETLW   01100110B
    RETLW   01101101B
    RETLW   01111101B
    RETLW   00000111B
    RETLW   01111111B
    RETLW   01101111B
    RETLW   01110111B
    RETLW   01111100B
    RETLW   00111001B
    RETLW   01011110B
    RETLW   01111001B
    RETLW   01110001B
    
END