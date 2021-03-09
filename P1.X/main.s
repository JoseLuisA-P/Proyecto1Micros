;-------------------------------------------------------------------------------
;
;   Autor: Jose Luis Alvarez Pineda
;   Archivo: main.s
;   Fecha de creacion: 9 de marzo de 2021
;   modificacion: 9 de marzo de 2021
;   Dispositivo: PIC16F887
;   Descripcion:
/* 
    */    
;   Hardware:
/*  
     */
;-------------------------------------------------------------------------------
PROCESSOR 16F887
#include <xc.inc>

; CONFIG1
  CONFIG  FOSC = INTRC_NOCLKOUT ; Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
  CONFIG  WDTE = OFF            ; Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
  CONFIG  PWRTE = OFF           ; Power-up Timer Enable bit (PWRT disabled)
  CONFIG  MCLRE = ON            ; RE3/MCLR pin function select bit (RE3/MCLR pin function is MCLR)
  CONFIG  CP = ON               ; Code Protection bit (Program memory code protection is enabled)
  CONFIG  CPD = ON              ; Data Code Protection bit (Data memory code protection is enabled)
  CONFIG  BOREN = OFF           ; Brown Out Reset Selection bits (BOR disabled)
  CONFIG  IESO = OFF            ; Internal External Switchover bit (Internal/External Switchover mode is disabled)
  CONFIG  FCMEN = OFF           ; Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
  CONFIG  LVP = OFF             ; Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

; CONFIG2
  CONFIG  BOR4V = BOR40V        ; Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
  CONFIG  WRT = OFF             ; Flash Program Memory Self Write Enable bits (Write protection off)

;---------------------------------Variables-------------------------------------
Psect	udata_shr		;Variables temporales de W y status para las INT
W_TEMP:		DS  1
STATUS_TEMP:	DS  1

Psect	udata_bank0
BANDERAS:	DS  1	    ;Banderas para manejar los eventos
MUX:		DS  1	    ;Manejar el multiplexado
BANDTIEMPO:	DS  1	    ;Banderas para manejar el tiempo
;----------------------Variables asociadas a los semaforos----------------------
;variables temporales que almacenan el valor luego del cambio
S1TEMP:		DS  1	    ;temporal semaforo 1
S2TEMP:		DS  1	    ;temporal semaforo 2
S3TEMP:		DS  1	    ;temporal semaforo 3
;variables que despliegan los valores al estar en el cambio
S1CAM:		DS  1	    ;Cambio semaforo 1
S2CAM:		DS  1	    ;Cambio semaforo 2
S3CAM:		DS  1	    ;Cambio semaforo 3
;variables que llevan el conteo de cada semaforo en los ciclos
S1TI:		DS  1	    ;conteo semaforo 1
S2TI:		DS  1	    ;conteo semaforo 2
S3TI:		DS  1	    ;conteo semaforo 3
;variable que lleva el tiempo en conteo dde cada semaforo
S1RO:		DS  1	    ;tiempo en rojo de los semaforos 1
S2RO:		DS  1	    ;tiempo en rojo de los semaforos 2
S3RO:		DS  1	    ;tiempo en rojo de los semaforos 3

;--------------Variables para los contadores------------------------------------
CONT0:		DS  1	    ;conteo del timmer 0
CONT2:		DS  1	    ;conteo del timmer 2
;Global para ver las variables en el simulador de MPLAB    
GLOBAL	BANDERAS,MUX,CONT0    
    
;------------------------------------Macros-------------------------------------
configPuertos	MACRO	    ;configurar los puertos
    BANKSEL	ANSEL
    CLRF	ANSEL	;quitar entradas analogicas
    CLRF	ANSELH
    BANKSEL	TRISA	;Configurar entradas y salidas de puertos
    CLRF	TRISA
    CLRF	TRISC
    CLRF	TRISD
    CLRF	TRISE
    MOVLW	0X07
    MOVWF	TRISB	;los primeros 3 como entradas
    BANKSEL	PORTB	;Colocar la salida de los puertos en low todas
    CLRF	PORTA
    CLRF	PORTB
    CLRF	PORTC
    CLRF	PORTD
    CLRF	PORTE
    CLRF	MUX	;limpia el multiplexor
    BSF		MUX,0	;el primer bit encendido
    CLRF	CONT0	; limpiar la variable del timmer 0
    ENDM    
    
configOSC	MACRO	;configurar el oscilador interno
    BANKSEL	TMR0
    CLRWDT
    CLRF    TMR0
    BANKSEL OSCCON  ;configurando la frecuencia y fuente del oscilador
    BSF	    IRCF2
    BSF	    IRCF1
    BCF	    IRCF0   ;Frecuencia de 4MHz---110
    BSF	    SCS	    ;utilizar el oscilador interno
    ;Configurando el prescalador y lafuente
    BCF	    OPTION_REG,5    ;TIMER0 usa el reloj interno
    BCF	    OPTION_REG,3    ;Prescalador al timmer0
    BSF	    OPTION_REG,2
    BCF	    OPTION_REG,1
    BSF	    OPTION_REG,0    ;Usar prescalador de 64
    CALL    CARGAT0
    ENDM 
    
configINT	MACRO	;configurar interrupciones
    BANKSEL	INTCON
    BSF		INTCON,7	;habilitar interrupciones GIE
    ;BSF		RBIE	;habilitar interrupciones de cambio en B
    BSF		INTCON,5	;habilitar interrupcion del TIMER0
    BSF		INTCON,6	;habilita interrupciones perifericas TIMER2
    BCF		INTCON,2	;apagar bandera TIMER0
    ;BCF		RBIF	;apagar bandera de cambios en puerto B
    BANKSEL	PIE1
    BSF		PIE1,1		;habilita interrupciones TIMER2
    BANKSEL	PIR1
    BCF		PIR1,1		;coloca en 0 la bandera del TIMER2
    CLRF	TMR2
    ENDM

configT2    MACRO   ;configurar el timmer2
    MOVLW   0XEF
    MOVWF   T2CON   ;pre en 16, post en 16 y habilitado el timmer2
    ENDM
;--------------------------------INICIO CODIGO----------------------------------
Psect	vectorReset, class = CODE, delta = 2, abs
ORG 0000h
    goto main
;----------------------------VECTOR INTERRUPCION--------------------------------
Psect	vectorINT, class= code, delta = 2, abs
ORG 0004h
    push:
    MOVWF   W_TEMP	    ;Guardar de forma temporal W y STATUS
    SWAPF   STATUS,W	    ;sin alterar sus banderas
    MOVWF   STATUS_TEMP	
    
    BTFSC   T0IF	    ;mira si es interrupcion del timmer0
    GOTO    TIE0
    GOTO    TIE2
    
    TIE0:
    INCF    CONT0
    BCF	    T0IF
    call    CARGAT0
    
    TIE2:
    BTFSS   PIR1,1	;Mira si es interrupcion del timer2
    GOTO    pop
    BCF	    STATUS,0
    RLF	    MUX
    BTFSC   STATUS,0
    call    arrmux
    BCF	    PIR1,1	;limpia bandera timmer2
    
    pop:
    SWAPF   STATUS_TEMP,W
    MOVWF   STATUS
    SWAPF   W_TEMP,F
    SWAPF   W_TEMP,W
    RETFIE			;termina la rutina de interrupcion
    
    arrmux:
    BCF	    STATUS,0
    CLRF    MUX
    BSF	    MUX,0
    return
;--------------------------MAIN, REINCIO Y LOOP PRINCIPAL-----------------------
Psect	loopPrin, class = code, delta = 2, abs
ORG 0100h
 
main:
    configPuertos
    configINT
    configOSC
    configT2
    GOTO    reinicio
    
reinicio:
    GOTO    loop
loop:
    MOVLW   100
    XORWF   CONT0,W	;mira si el timmer ya llego a 1 segundo
    BTFSC   STATUS,2	;mira si la operacion no es cero
    CALL    toogle
    MOVF    MUX,W
    MOVWF   PORTD
    GOTO    loop
    
CARGAT0:
    BANKSEL TMR0    ;Precarga el valor de 217 al TM0
    MOVLW   99
    MOVWF   TMR0
    BCF	    INTCON,2	;Limpiar bandera del TIMER0
    RETURN

toogle:
    CLRF    CONT0
    MOVLW   1
    XORWF   PORTE,F
    RETURN
    
END