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
;---------------DECENAS Y UNIDADES DEL CONTADOR---------------------------------
DIV1:		DS  1	    ;dividendo del semaforo 1
DEC1:		DS  1	    ;decenas del semaforo 1
UN1:		DS  1	    ;unidades del semaforo 1
DIV2:		DS  1	    ;dividendo del semaforo 2
DEC2:		DS  1	    ;decenas del semaforo 2
UN2:		DS  1	    ;unidades del semaforo 2
DIV3:		DS  1	    ;dividendo del semaforo 3
DEC3:		DS  1	    ;decenas del semaforo 3
UN3:		DS  1	    ;unidades del semaforo 3
;Global para ver las variables en el simulador de MPLAB    
GLOBAL	BANDERAS,MUX,CONT0,S1TEMP,S2TEMP,S3TEMP,S1RO,S2RO,S3RO,S1TI,S2TI,S3TI    
GLOBAL	DEC1,UN1    
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
    MOVLW	0XAF
    MOVWF	PR2	;modificar el periodo del TIMMER2
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
    MOVLW   0X0F
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
    BSF	    CONT2,0
    BCF	    PIR1,1	;limpia bandera timmer2
    
    pop:
    SWAPF   STATUS_TEMP,W
    MOVWF   STATUS
    SWAPF   W_TEMP,F
    SWAPF   W_TEMP,W
    RETFIE			;termina la rutina de interrupcion
;--------------------------MAIN, REINCIO Y LOOP PRINCIPAL-----------------------
Psect	loopPrin, class = code, delta = 2, abs
ORG 0100h

tabla:
    CLRF    PCLATH	;Colocar el PCLATH en 01 para seleccionar la
    BSF	    PCLATH,0	;pagina correcta
    ADDWF   PCL		;sumar segmento + PCL para seleccionar el valor adecuado
    retlw   00111111B	;0
    retlw   00000110B	;1
    retlw   01011011B	;2
    retlw   01001111B	;3
    retlw   01100110B	;4
    retlw   01101101B	;5
    retlw   01111101B	;6
    retlw   00000111B	;7
    retlw   01111111B	;8
    retlw   01100111B	;9
    retlw   01110111B	;A
    retlw   01111100B	;b
    retlw   00111001B	;C
    retlw   01011110B	;d
    retlw   01111001B	;E
    retlw   01110001B	;F 
 
main:
    configPuertos   ;configurar los puertos
    configINT	    ;configurar las interrupciones
    configOSC	    ;configurar el oscilador
    configT2	    ;configurar el timmer2
    MOVLW   15	    ;colocar en 10 las variables iniciales de cambio
    MOVWF   S1CAM
    MOVWF   S2CAM
    MOVWF   S3CAM
    GOTO    reinicio
    
reinicio:
    MOVF    S1CAM,W ;Carga el cambio de S1 a W y luego a temporal y conteo	
    MOVWF   S1TI
    MOVWF   S2RO    ;se coloca en los rojos 2 y 3 el valor que tiene 1 en el 
    MOVWF   S3RO    ;cambio
    MOVF    S2CAM,W ;carga el valor de cambio de S2 a W y luego a su temporal	
    MOVWF   S2TI
    MOVWF   S1RO    ;carga este valor en el rojo 1 
    ADDWF   S3RO,F  ;se le suma este valor a el rojo 3
    MOVF    S3CAM,W ;carga el valor de cambio de S3 a W y luego a su temporal
    MOVWF   S3TI
    ADDWF   S1RO,F  ;le suma este valor al rojo del 1
    CLRF    BANDTIEMPO	    ;limpia las banderas de conteo en rojo
    BSF	    BANDTIEMPO,0    ;indica que el semaforo 1 comienza en verde
    BSF	    BANDERAS,7	    ;descartable para el rojo de 2
    GOTO    loop
    
loop:
    BTFSC   CONT2,0
    CALL    ARREMUX
    MOVF    S1TEMP,W	;se mueve el temporal de 1 a W y se mira si es 0
    BTFSC   STATUS,2	
    CALL    CARGARTEMP1	;carga verde o rojo a temp1
    MOVF    S2TEMP,W	;se mueve el temporal de 2 a W y se mira si es 0
    BTFSC   STATUS,2	
    CALL    CARGARTEMP2	;carga verde o rojo a temp2
    MOVF    S3TEMP,W	;se mueve el temporal de 3 a W y se mira si es 0
    BTFSC   STATUS,2	
    CALL    CARGARTEMP3	;carga verde o rojo a temp3
    MOVLW   100
    XORWF   CONT0,W	;mira si el timmer ya llego a 1 segundo
    BTFSC   STATUS,2	;mira si la operacion no es cero
    CALL    REGRESIVO
    ;limpia las variables de division para evitar acumulado
    CLRF    DEC1
    CLRF    UN1
    CLRF    DEC2
    CLRF    UN2
    CLRF    DEC3
    CLRF    UN3
    BTFSC   BANDERAS,0
    CALL    DIVISION	;llama la rutina de division de los numeros
    BTFSC   BANDERAS,0
    CALL    MULTIPLEX	;llama la rutina de multiplexado
    GOTO    loop

CARGAT0:
    BANKSEL TMR0    ;Precarga el valor de 217 al TM0
    MOVLW   99
    MOVWF   TMR0
    BCF	    INTCON,2	;Limpiar bandera del TIMER0
    RETURN

CARGARTEMP1:
    BTFSS   BANDTIEMPO,0
    GOTO    $+8
    MOVF    S1TI,W
    MOVWF   S1TEMP
    BCF	    BANDTIEMPO,0
    BSF	    PORTA,2	;Enciende el verde, apaga los demas
    BCF	    PORTA,1
    BCF	    PORTA,0
    RETURN  
    MOVF    S1RO,W
    MOVWF   S1TEMP
    BSF	    BANDTIEMPO,0
    BCF	    PORTA,2	;Enciende el ROJO, apaga los demas
    BCF	    PORTA,1
    BSF	    PORTA,0
    RETURN
    
CARGARTEMP2:
    BTFSS   BANDTIEMPO,1
    GOTO    $+8
    MOVF    S2TI,W
    MOVWF   S2TEMP
    BCF	    BANDTIEMPO,1
    BSF	    PORTA,5	;Enciende el verde, apaga los demas
    BCF	    PORTA,4
    BCF	    PORTA,3
    RETURN  
    MOVF    S2RO,W
    MOVWF   S2TEMP
    BSF	    BANDTIEMPO,1
    BCF	    PORTA,5	;Enciende el verde, apaga los demas
    BCF	    PORTA,4
    BSF	    PORTA,3
    BTFSC   BANDERAS,7
    CALL    ARR_ROJO2
    RETURN

CARGARTEMP3:
    BTFSS   BANDTIEMPO,2
    GOTO    $+8
    MOVF    S3TI,W
    MOVWF   S3TEMP
    BCF	    BANDTIEMPO,2
    BSF	    PORTE,2	;Enciende el verde, apaga los demas
    BCF	    PORTE,1
    BCF	    PORTE,0
    RETURN  
    MOVF    S3RO,W
    MOVWF   S3TEMP
    BSF	    BANDTIEMPO,2
    BCF	    PORTE,2	;Enciende el verde, apaga los demas
    BCF	    PORTE,1
    BSF	    PORTE,0
    RETURN
    
REGRESIVO:
    DECF    S1TEMP
    DECF    S2TEMP
    DECF    S3TEMP
    CLRF    CONT0
    RETURN
    
MULTIPLEX:
    BTFSC   MUX,0
    CALL    MULTI1
    BTFSC   MUX,1
    CALL    MULTI2
    BTFSC   MUX,2
    CALL    MULTI3
    BTFSC   MUX,3
    CALL    MULTI4
    BTFSC   MUX,4
    CALL    MULTI5
    BTFSC   MUX,5
    CALL    MULTI6
    BTFSC   MUX,6
    CLRF    PORTD
    BTFSC   MUX,7
    CLRF    PORTD
    BCF	    BANDERAS,0
    RETURN
    
MULTI1:
    CLRF    PORTC
    MOVF    UN1,W
    CALL    tabla
    MOVWF   PORTD
    MOVF    MUX,W
    MOVWF   PORTC
    RETURN

MULTI2:
    CLRF    PORTC
    MOVF    DEC1,W
    CALL    tabla
    MOVWF   PORTD
    MOVF    MUX,W
    MOVWF   PORTC
    RETURN

MULTI3:
    CLRF    PORTC
    MOVF    UN2,W
    CALL    tabla
    MOVWF   PORTD
    MOVF    MUX,W
    MOVWF   PORTC
    RETURN    
    
MULTI4:
    CLRF    PORTC
    MOVF    DEC2,W
    CALL    tabla
    MOVWF   PORTD
    MOVF    MUX,W
    MOVWF   PORTC
    RETURN

MULTI5:
    CLRF    PORTC
    MOVF    UN3,W
    CALL    tabla
    MOVWF   PORTD
    MOVF    MUX,W
    MOVWF   PORTC
    RETURN    
    
MULTI6:
    CLRF    PORTC
    MOVF    DEC3,W
    CALL    tabla
    MOVWF   PORTD
    MOVF    MUX,W
    MOVWF   PORTC
    RETURN      
    
DIVISION:
    MOVF    S1TEMP,W	;Se divide primero el valor de semaforo 1
    MOVWF   DIV1
    INCF    DEC1
    MOVLW   10
    SUBWF   DIV1,F
    BTFSC   STATUS,0	;mira si no hay carry
    GOTO    $-4		;si no hay vuelve a restar
    DECF    DEC1	;si hay carry le quita 1 porque se paso
    MOVLW   10		;le suma 10 para que regrese al estado previo
    ADDWF   DIV1,F
    MOVF    DIV1,W
    MOVWF   UN1		;el remanente se queda en las unidades
;DIVISION DEL SEGUNDO SEMAFORO
    MOVF    S2TEMP,W	;Se divide primero el valor de semaforo 2
    MOVWF   DIV2
    INCF    DEC2
    MOVLW   10
    SUBWF   DIV2,F
    BTFSC   STATUS,0	;mira si no hay carry
    GOTO    $-4		;si no hay vuelve a restar
    DECF    DEC2	;si hay carry le quita 1 porque se paso
    MOVLW   10		;le suma 10 para que regrese al estado previo
    ADDWF   DIV2,F
    MOVF    DIV2,W
    MOVWF   UN2		;el remanente se queda en las unidades
;DIVISION DEL TERCER SEMAFORO  
    MOVF    S3TEMP,W	;Se divide primero el valor de semaforo 3
    MOVWF   DIV3
    INCF    DEC3
    MOVLW   10
    SUBWF   DIV3,F
    BTFSC   STATUS,0	;mira si no hay carry
    GOTO    $-4		;si no hay vuelve a restar
    DECF    DEC3	;si hay carry le quita 1 porque se paso
    MOVLW   10		;le suma 10 para que regrese al estado previo
    ADDWF   DIV3,F
    MOVF    DIV3,W
    MOVWF   UN3		;el remanente se queda en las unidades
    RETURN

ARREMUX:
    BSF	    BANDERAS,0	;bandera que indica que se puede multiplexar 
    CLRF    CONT2
    RLF	    MUX	;mueve a la izquierda el bit encendido
    BTFSC   STATUS,0
    GOTO    $+2
    RETURN
    MOVLW   1
    MOVWF   MUX
    RETURN

ARR_ROJO2:
    MOVF    S3CAM,W	;Hasta este momento se carga el tiempo del semaforo 3
    ADDWF   S2RO,F
    BCF	    BANDERAS,7	;se coloca en 0 la descartable
    RETURN
                
END