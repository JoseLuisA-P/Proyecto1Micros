;-------------------------------------------------------------------------------
;
;   Autor: Jose Luis Alvarez Pineda
;   Archivo: main.s
;   Fecha de creacion: 9 de marzo de 2021
;   modificacion: 25 de marzo de 2021
;   Dispositivo: PIC16F887
;   Descripcion:
/* Semaforo de 3 vias el cual se puede controlar en base a tiempos establecidos
    por el usuario. Estos tiempos oscilaran entre 10 y 20 segundos, cada 
    semaforo funciona acorde a 2 estados dando via o en rojo, y el estado de via
    tiene el modo de verde y verde titilante y luego amarillo. El tiempo de 
    verde y verde titilante es independiente del de amarillo y rojo. Todos los
    semaforos van a dar a una sola via de salida, al reinicio siempre comienza 
    en el semaforo 1 y al realizar cambios sucede lo mismo. El semaforo cuenta
    con 5 modos, en 3 de ellos se modifica el valor al cual va a contar los
    semaforos y en 1 de ellos funciona normal, en el ultimo se puede aceptar o 
    cancelar los cambios en el tiempo y en todos los modos los semaforos siguen
    funcionando sin detenerse, solamente se reinician al aceptar los cambios.
    Los tiempos de via y rojo son indicados en displays en pares los cuales
    son individuales para cada semaforo y luego se tiene una pareja de displays 
    los cuales muestran el valor que se modificara y cargara a un semaforo.
    */    
;   Hardware:
/*  Puerto A:
    A0-A2: luz roja, amarilla y verde del semaforo 1
    A3-A5: luz roja, amarilla y verde del semaforo 2
    
    Puerto B:
    B0: boton que cambia el modo
    B1: boton para aumentar el valor/ aceptar el cambio
    B2: boton para disminuir el valor/ rechazar el cambio
    B3-B7: indicador del modo 1 al modo 5 en leds.
     
    Puerto C: Todos estos puertos son utilizados para multiplexar los displays
    C0-C1: displays semaforo 1
    C2-C3: displays semaforo 2
    C4-C5: displays semaforo 3
    C6-C7: displays indicador de valores del semaforo
     
    Puerto D:
    De 0 a 7 se sacan los valores de A a H utilizados por los displays 
    al ser multiplexados
     
    Puerto E:
    E0-E2: luz roja, amarilla y verde del semaforo 3
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
BANDERAS2:	DS  1	    ;Mas banderas para manejar eventos
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
CONT2:		DS  1	    ;conteo del timmer 0 parpadeo 
CONTRE:		DS  1	    ;Variable para el conteo del inicio
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
;-----------------------modos y alterar valores---------------------------------
BANDMODOS:	DS  1	    ;bandera para manejar los distintos modos
CAMBTEMP1:	DS  1	    ;adopta el valor a modificar del semaforo1
CAMBTEMP2:	DS  1	    ;adopta el valor a modificar del semaforo2
CAMBTEMP3:	DS  1	    ;adopta el valor a modificar del semaforo3
DIVTEMPO:	DS  1	    ;dividendo temporal para la variable a cambiar
DECTEMPO:	DS  1	    ;Decena del temporal modificando
UNTEMPO:	DS  1	    ;unidades del temporal modificando
;COLOR AMARILLO AGREGADO
S1AMAR:		DS  1	    ;Temporal de amarillo
S2AMAR:		DS  1	    ;Temporal de amarillo
S3AMAR:		DS  1	    ;Temporal de amarillo    
;Global para ver las variables en el simulador de MPLAB    
GLOBAL	BANDERAS,MUX,CONT0,S1TEMP,S2TEMP,S3TEMP,S1RO,S2RO,S3RO,S1TI,S2TI,S3TI    
GLOBAL	DEC1,UN1,BANDMODOS    
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
    CLRF	BANDMODOS   ;en 0 bandera de modos
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
    BSF		RBIE		;habilitar interrupciones de cambio en B
    BSF		INTCON,5	;habilitar interrupcion del TIMER0
    BSF		INTCON,6	;habilita interrupciones perifericas TIMER2
    BCF		INTCON,2	;apagar bandera TIMER0
    BCF		RBIF		;apagar bandera de cambios en puerto B
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
    
configINTB	MACRO		;configurar pullup e interrupciones en B
    BANKSEL	TRISA
    BCF		OPTION_REG,7	;permite habilitar las pullup en B (RBPU)
    MOVLW	0X07
    MOVWF	WPUB	;Primeros dos pines de B en WeakPullup
    MOVLW	0X07
    MOVWF	IOCB	;primeros dos pines habilitados de intOnChange
    BANKSEL	PORTA
    MOVF	PORTB,W	;eliminar el mismatch
    BCF		RBIF
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
    
    BTFSC   RBIF
    GOTO    INOCB
    BTFSC   T0IF	    ;mira si es interrupcion del timmer0
    GOTO    TIE0
    GOTO    TIE2
    
    TIE0:
    INCF    CONT0	;contador para los segundos
    INCF    CONT2	;contador para el parpadeo
    BCF	    T0IF
    call    CARGAT0
    
    TIE2:
    BTFSS   PIR1,1	;Mira si es interrupcion del timer2
    GOTO    pop
    BSF	    BANDERAS,1	;Bandera que indica que ya es tiempo de multiplexar
    BCF	    PIR1,1	;limpia bandera timmer2
    
    INOCB: 
    BTFSS   PORTB,0	;mira si es cambio de modo
    BSF	    BANDERAS,6	;Permite que se haga el RLF
    BTFSS   PORTB,1	;mira si es aumento/aceptar
    BSF	    BANDERAS2,0	;Se puede aumentar
    BTFSS   PORTB,2	;Mira si es disminucion
    BSF	    BANDERAS2,1	;Se puede disminuir
    MOVF    PORTB,W	;eliminar el mismatch
    BCF	    RBIF	;elimina bandera del IOCB
    
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
    configINTB	    ;configurar IOC del puerto B
    MOVLW   10	    ;colocar en 10 las variables iniciales de cambio
    MOVWF   S1CAM
    MOVWF   S2CAM
    MOVWF   S3CAM
    MOVWF   CAMBTEMP1
    MOVWF   CAMBTEMP2
    MOVWF   CAMBTEMP3
    GOTO    reinicio

modificacion:
    CLRF    S1TEMP	    ;Reinicia las variables necesarias
    CLRF    S2TEMP
    CLRF    S3TEMP
    CLRF    BANDERAS	    ;Reinicia las banderas y los contadores
    CLRF    BANDERAS2
    CLRF    CONT0
    CLRF    CONT2
    MOVF    CAMBTEMP1,W	    ;Precarga los nuevos valores
    MOVWF   S1CAM
    MOVF    CAMBTEMP2,W	    ;Precarga los nuevos valores
    MOVWF   S2CAM
    MOVF    CAMBTEMP3,W	    ;Precarga los nuevos valores
    MOVWF   S3CAM
    CALL    REINICIO	    ;Reinicia los valores
    MOVLW   0XFF
    MOVWF   PORTD
    MOVWF   PORTC
    MOVWF   PORTB
    MOVWF   PORTA
    MOVWF   PORTE
    MOVLW   100
    XORWF   CONT0,W
    BTFSS   STATUS,2	    ;mira si ya paso un segundo
    GOTO    $-3
    CLRF    CONT0
    INCF    CONTRE
    MOVLW   3
    XORWF   CONTRE,W
    BTFSS   STATUS,2	    ;Mira si ya paso 5 segundos
    GOTO    $-9
    CLRF    CONTRE
    CLRF    CONT0
    CLRF    PORTB
    BSF	    PORTB,3	    ;encender luz del modo
    
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
    MOVLW   3
    MOVWF   S1AMAR	    ;Carga el valor de 3 a amarillo 1
    MOVWF   S2AMAR	    ;Carga el valor de 3 a amarillo 2
    MOVWF   S3AMAR
    GOTO    loop
    
loop:
    BTFSC   BANDERAS,1
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
    MOVLW   20
    XORWF   CONT2,W
    BTFSC   STATUS,2
    BSF	    BANDERAS,5	    ;Habilita el toogle cada cierto tiempo
    ;limpia las variables de division para evitar acumulado
    CLRF    DEC1
    CLRF    UN1
    CLRF    DEC2
    CLRF    UN2
    CLRF    DEC3
    CLRF    UN3
    CLRF    DECTEMPO
    CLRF    UNTEMPO
    ;se modifican los valores temporales
    BTFSC   BANDERAS2,0	    ;Mira si es aumento
    CALL    AUMTEMPO	    ;aumenta el valor temporal indicado
    BTFSC   BANDERAS2,1	    ;Mira si es disminucion
    CALL    DISMTEMPO	    ;disminucion de el valor temporal indicado
    ;-----------------------------------
    BTFSC   BANDERAS,0
    CALL    DIVISION	;llama la rutina de division de los numeros
    BTFSC   BANDERAS,0
    CALL    MULTIPLEX	;llama la rutina de multiplexado
    MOVLW   3		;mira si ya es 3 en el semaforo y si esta en verde luego
    XORWF   S1TEMP,W
    BTFSC   STATUS,2
    CALL    S1LAM
    MOVLW   3
    XORWF   S2TEMP,W
    BTFSC   STATUS,2
    CALL    S2LAM
    MOVLW   3
    XORWF   S3TEMP,W
    BTFSC   STATUS,2
    CALL    S3LAM
    MOVLW   6		;mira si es 6 en el semaforo y si este es en verde luego
    XORWF   S1TEMP,W
    BTFSC   STATUS,2
    CALL    S1TOOGE
    BTFSC   BANDERAS,2	
    CALL    S1TOOG
    MOVLW   6
    XORWF   S2TEMP,W
    BTFSC   STATUS,2
    CALL    S2TOOGE
    BTFSC   BANDERAS,3	
    CALL    S2TOOG
    MOVLW   6
    XORWF   S3TEMP,W
    BTFSC   STATUS,2
    CALL    S3TOOGE
    BTFSC   BANDERAS,4	
    CALL    S3TOOG
    BTFSC   BANDERAS,5
    CLRF    CONT2		;reinicia el contador luego de haber contado
    BCF	    BANDERAS,5		;Luego de hacer el toogle las apaga
    BTFSC   BANDERAS,6		;mira si se hacer el cambio de modo
    CALL    INDMODOS		;Colocar el modo en el puerto
    BTFSS   BANDMODOS,4		;mira si es el modo 5
    GOTO    loop
    BTFSC   BANDERAS2,1		;mira si es para cancelar
    CALL    REINICIO
    BTFSC   BANDERAS2,0		;mira si es para aceptar
    GOTO    modificacion
    GOTO    loop

REINICIO:
    MOVLW   10			;coloca todas las temporales de nuevo en 10
    MOVWF   CAMBTEMP1
    MOVWF   CAMBTEMP2
    MOVWF   CAMBTEMP3
    CLRF    BANDMODOS		;Lo regresa al modo 0
    BSF	    BANDMODOS,0
    CLRF    PORTB
    BSF	    PORTB,3
    RETURN
    
INDMODOS:
    BTFSS   PORTB,0
    RETURN
    BCF	    BANDERAS,6	;ya no permite el cambio de modo
    BCF	    STATUS,0	;elimina el carry
    RLF	    BANDMODOS
    BTFSS   BANDMODOS,5	;Para que regrese al modo 1
    GOTO    $+3
    CLRF    BANDMODOS
    BSF	    BANDMODOS,0
    CLRF    PORTB
    MOVLW   0
    XORWF   BANDMODOS
    BTFSS   STATUS,2	;mira si comienza en 0
    GOTO    $+2
    BSF	    BANDMODOS,0	;inicializa el primero en 0
    BTFSC   BANDMODOS,0	;Mira que led indicador de modo tiene que colocar
    BSF	    PORTB,3
    BTFSC   BANDMODOS,1
    BSF	    PORTB,4
    BTFSC   BANDMODOS,2
    BSF	    PORTB,5
    BTFSC   BANDMODOS,3
    BSF	    PORTB,6
    BTFSC   BANDMODOS,4
    BSF	    PORTB,7
    RETURN
    
CARGAT0:
    BANKSEL TMR0    ;Precarga el valor de 217 al TM0
    MOVLW   99
    MOVWF   TMR0
    BCF	    INTCON,2	;Limpiar bandera del TIMER0
    RETURN

S1TOOGE:
    BTFSS   BANDTIEMPO,0    ;mira si esta en verde
    BSF	    BANDERAS,2	    ;Activa la bandera de toogle1
    RETURN

S1TOOG:
    BTFSC   PORTA,2	;revisa si esta apagado o encendido y le hace toogle
    GOTO    $+4
    BTFSC   BANDERAS,5	;apaga el habilitar que haga toogle hasta que vuelva
    BSF	    PORTA,2	   ;a pasar el tiempo estipulado en CONT2
    RETURN
    BTFSC   BANDERAS,5
    BCF	    PORTA,2
    BCF	    PORTC,0	    ;para hacer parpadear al contador
    BCF	    PORTC,1
    RETURN
    
S2TOOGE:
    BTFSS   BANDTIEMPO,1    ;mira si esta en verde
    BSF	    BANDERAS,3	    ;Activa la bandera de toogle1
    RETURN

S2TOOG:
    BTFSC   PORTA,5
    GOTO    $+4
    BTFSC   BANDERAS,5	;apaga el habilitar que haga toogle hasta que vuelva
    BSF	    PORTA,5	    ;a pasar el tiempo estipulado en CONT2
    RETURN
    BTFSC   BANDERAS,5
    BCF	    PORTA,5
    BCF	    PORTC,2	    ;para hacer parpadear al contador
    BCF	    PORTC,3
    RETURN

S3TOOGE:
    BTFSS   BANDTIEMPO,2    ;mira si esta en verde
    BSF	    BANDERAS,4	    ;Activa la bandera de toogle1
    RETURN

S3TOOG:
    BTFSC   PORTE,2
    GOTO    $+4
    BTFSC   BANDERAS,5	    ;apaga el habilitar que haga toogle hasta que vuelva
    BSF	    PORTE,2		;a pasar el tiempo estipulado en CONT2
    RETURN
    BTFSC   BANDERAS,5
    BCF	    PORTE,2
    BCF	    PORTC,4	    ;para hacer parpadear al contador
    BCF	    PORTC,5
    RETURN
    
S1LAM:
    BTFSC   BANDTIEMPO,0    ;mira si esta en VERDE
    RETURN
    BCF	    BANDERAS,2	    ;Desactiva el toogle
    BSF	    BANDTIEMPO,3    ;Activa el amarillo
    BCF	    PORTA,2
    BSF	    PORTA,1
    BCF	    PORTA,0
    RETURN
    
S2LAM:
    BTFSC   BANDTIEMPO,1    ;mira si esta en VERDE
    RETURN
    BCF	    BANDERAS,3	    ;desactiva el toogle
    BSF	    BANDTIEMPO,4    ;Activa el amarillo 2
    BCF	    PORTA,5
    BSF	    PORTA,4
    BCF	    PORTA,3
    RETURN

S3LAM:
    BTFSC   BANDTIEMPO,2    ;mira si esta en VERDE
    RETURN
    BCF	    BANDERAS,4	    ;Desactiva el toogle 3
    BSF	    BANDTIEMPO,5
    BCF	    PORTE,2
    BSF	    PORTE,1
    BCF	    PORTE,0
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
    BCF	    BANDTIEMPO,3    ;apaga el modo amarillo
    MOVLW   3
    MOVWF   S1AMAR
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
    BCF	    BANDTIEMPO,4    ;apaga el modo amarillo
    MOVLW   3
    MOVWF   S2AMAR
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
    BCF	    BANDTIEMPO,5    ;apaga el modo amarillo
    MOVLW   3
    MOVWF   S3AMAR
    RETURN
    
REGRESIVO:
    DECF    S1TEMP
    DECF    S2TEMP
    DECF    S3TEMP
    BTFSC   BANDTIEMPO,3
    DECF    S1AMAR
    BTFSC   BANDTIEMPO,4
    DECF    S2AMAR
    BTFSC   BANDTIEMPO,5
    DECF    S3AMAR
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
    BTFSC   BANDMODOS,0	    ;Mira si es distinto a el modo
    GOTO    $+7
    BTFSC   MUX,6	    ;se muestra el valor que se modificara en el momento
    CALL    MULTI7
    BTFSC   MUX,7
    CALL    MULTI8
    BCF	    BANDERAS,0
    RETURN
    BTFSC   MUX,6	    ;el indicador de los valores de cambio apagado
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
    
MULTI7:
    CLRF    PORTC
    BTFSS   BANDMODOS,4
    MOVF    UNTEMPO,W
    BTFSC   BANDMODOS,4
    MOVLW   0X0C
    CALL    tabla
    MOVWF   PORTD
    MOVF    MUX,W
    MOVWF   PORTC
    RETURN    
    
MULTI8:
    CLRF    PORTC
    BTFSS   BANDMODOS,4
    MOVF    DECTEMPO,W
    BTFSC   BANDMODOS,4
    MOVLW   0X0A
    CALL    tabla
    MOVWF   PORTD
    MOVF    MUX,W
    MOVWF   PORTC
    RETURN      
    
DIVISION:
    BTFSC   BANDTIEMPO,0    ;mira si es rojo o verde
    GOTO    $+4
    MOVLW   3
    SUBWF   S1TEMP,W	;Se divide primero el valor de semaforo 1
    GOTO    $+3
    MOVF    S1TEMP,W
    GOTO    $+3
    BTFSC   BANDTIEMPO,3
    MOVF    S1AMAR,W		;carga el valor de 3 a el semaforo
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
    BTFSC   BANDTIEMPO,1    ;mira si es rojo o verde
    GOTO    $+4
    MOVLW   3
    SUBWF   S2TEMP,W	;Se divide primero el valor de semaforo 1
    GOTO    $+3
    MOVF    S2TEMP,W
    GOTO    $+3
    BTFSC   BANDTIEMPO,4
    MOVF    S2AMAR,W		;carga el valor de 3 a el semaforo
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
    BTFSC   BANDTIEMPO,2    ;mira si es rojo o verde
    GOTO    $+4
    MOVLW   3
    SUBWF   S3TEMP,W	;Se divide primero el valor de semaforo 1
    GOTO    $+3
    MOVF    S3TEMP,W
    GOTO    $+3
    BTFSC   BANDTIEMPO,5
    MOVF    S3AMAR,W		;carga el valor de 3 a el semaforo
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
;DIVISION DEL semaforo indicador
    BTFSC   BANDMODOS,0	    ;si esta en modo 1 no divide nada
    RETURN
    BTFSC   BANDMODOS,1	    ;para cargar el valor del semaforo 1
    MOVF    CAMBTEMP1,W
    BTFSC   BANDMODOS,2	    ;para cargar el valor del semaforo 2
    MOVF    CAMBTEMP2,W
    BTFSC   BANDMODOS,3	    ;para cargar el valor del semaforo 3
    MOVF    CAMBTEMP3,W
    MOVWF   DIVTEMPO
    INCF    DECTEMPO
    MOVLW   10
    SUBWF   DIVTEMPO,F
    BTFSC   STATUS,0	;mira si no hay carry
    GOTO    $-4		;si no hay vuelve a restar
    DECF    DECTEMPO	;si hay carry le quita 1 porque se paso
    MOVLW   10		;le suma 10 para que regrese al estado previo
    ADDWF   DIVTEMPO,F
    MOVF    DIVTEMPO,W
    MOVWF   UNTEMPO		;el remanente se queda en las unidades
    RETURN

ARREMUX:
    BSF	    BANDERAS,0	;bandera que indica que se puede multiplexar 
    BCF	    BANDERAS,1
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

AUMTEMPO:
    BTFSS   PORTB,1
    RETURN
    BTFSS   BANDMODOS,1	    ;Mira si es semaforo 1
    GOTO    $+6
    INCF    CAMBTEMP1,F    ;Incrementa la temporal 1
    MOVLW   21
    XORWF   CAMBTEMP1,W    ;Mira si es 21
    BTFSC   STATUS,2	    ;Mira si el valor es 21
    CALL    ARR20
    BTFSS   BANDMODOS,2	    ;Mira si es semaforo 2
    GOTO    $+6
    INCF    CAMBTEMP2,F    ;Incrementa la temporal 2
    MOVLW   21
    XORWF   CAMBTEMP2,W    ;Mira si es 21
    BTFSC   STATUS,2	    ;Mira si el valor es 21
    CALL    ARR20
    BTFSS   BANDMODOS,3	    ;Mira si es semaforo 2
    GOTO    $+6
    INCF    CAMBTEMP3,F    ;Incrementa la temporal 2
    MOVLW   21
    XORWF   CAMBTEMP3,W    ;Mira si es 21
    BTFSC   STATUS,2	    ;Mira si el valor es 21
    CALL    ARR20
    BCF	    BANDERAS2,0	    ;Elimina la bandera de aumento
    RETURN
    
DISMTEMPO:
    BTFSS   PORTB,2
    RETURN
    BTFSS   BANDMODOS,1	    ;Mira si es semaforo 1
    GOTO    $+6
    DECF    CAMBTEMP1,F    ;DECREMENTA la temporal 1
    MOVLW   9
    XORWF   CAMBTEMP1,W    ;Mira si es 9
    BTFSC   STATUS,2	    ;Mira si el valor es 9
    CALL    ARR10
    BTFSS   BANDMODOS,2	    ;Mira si es semaforo 2
    GOTO    $+6
    DECF    CAMBTEMP2,F    ;DECREMENTA la temporal 2
    MOVLW   9
    XORWF   CAMBTEMP2,W    ;Mira si es 9
    BTFSC   STATUS,2	    ;Mira si el valor es 9
    CALL    ARR10
    BTFSS   BANDMODOS,3	    ;Mira si es semaforo 3
    GOTO    $+6
    DECF    CAMBTEMP3,F    ;DECREMENTA la temporal 3
    MOVLW   9
    XORWF   CAMBTEMP3,W    ;Mira si es 9
    BTFSC   STATUS,2	    ;Mira si el valor es 9
    CALL    ARR10
    BCF	    BANDERAS2,1	    ;Elimina la bandera de decremento
    RETURN

ARR20:
    MOVLW   10
    BTFSC   BANDMODOS,1	;Mira si es el valor del semaforo 1 y lo coloca en 10
    MOVWF   CAMBTEMP1
    BTFSC   BANDMODOS,2	;Mira si es el valor del semaforo 1 y lo coloca en 10
    MOVWF   CAMBTEMP2
    BTFSC   BANDMODOS,3	;Mira si es el valor del semaforo 1 y lo coloca en 10
    MOVWF   CAMBTEMP3
    RETURN
    
ARR10:
    MOVLW   20
    BTFSC   BANDMODOS,1	;Mira si es el valor del semaforo 1 y lo coloca en 10
    MOVWF   CAMBTEMP1
    BTFSC   BANDMODOS,2	;Mira si es el valor del semaforo 1 y lo coloca en 10
    MOVWF   CAMBTEMP2
    BTFSC   BANDMODOS,3	;Mira si es el valor del semaforo 1 y lo coloca en 10
    MOVWF   CAMBTEMP3
    RETURN    
    
END