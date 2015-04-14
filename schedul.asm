; ******************************************************
; BASIC .ASM template file for AVR
; ******************************************************

.include "C:\VMLAB\include\m8def.inc"

; Define here the variables
;
.def  temp  =r16
.equ TASKCOUNT =$0060
.equ CURR_TASK =$0061
.equ TASK_START =$0062

; Define here Reset and interrupt vectors, if any
;
reset:
   rjmp start
   reti      ; Addr $01
   reti      ; Addr $02
   rjmp ocmp2_isr      ; Addr $03
   reti      ; Addr $04
   reti      ; Addr $05
   reti      ; Addr $06        Use 'rjmp myVector'
   reti      ; Addr $07        to define a interrupt vector
   reti      ; Addr $08
   reti      ; Addr $09
   reti      ; Addr $0A
   reti      ; Addr $0B        This is just an example
   reti      ; Addr $0C        Not all MCUs have the same
   reti      ; Addr $0D        number of interrupt vectors
   reti      ; Addr $0E
   reti      ; Addr $0F
   reti      ; Addr $10

; Program starts here after Reset
;
start:
   ;first initialize the stack pointer
	ldi r16,low(RAMEND)
	ldi r17,high(RAMEND)
	out SPL,r16
	out SPH,r17
	rcall memset_data     ; We will use the location present from 0060 (+100 bytes) for our nano kernel
	ldi r24,low(task1)
	ldi r25,high(task1)
	rcall add_task
	ldi r24,low(task2)
	ldi r25,high(task2)
	rcall add_task
	ldi r24,low(task3)
	ldi r25,high(task3)
	rcall add_task
	;Now we can call any subroutines because the stack pointer is initialized
	rcall timer2_init
	
forever:
   nop
   nop       ; Infinite loop.
   nop       ; Define your main system
   nop       ; behaviour here
rjmp forever


;we will use the timer2 to generate a delay of 100 us;clock frequency assumed to be 1MHZ
timer2_init:

	ldi r16,100
	out OCR2,r16
	ldi r16,(1<<CS20|1<<WGM21)	;No prescaler and CTC Mode
	out TCCR2,r16
	ldi r16,1<<OCIE2
	out TIMSK,r16
	sei;global interrupts are enabled
	ret
	
ocmp2_isr:

	;load the taskcount
	
	lds r16,TASKCOUNT
	lds r17,CURR_TASK
	CP r16,r17
	brne comp
	ldi r17,0
comp:
	MOV R18,R17
	ldi R29,HIGH(TASK_START)
	ldi R28,LOW(TASK_START)
	ADD r17,r17
	breq isr2
	isr1:
	   	INC R28
	   	DEC R17
	   	brne isr1
	
isr2:	
	ld R30,Y+
	ld R31,Y
	;We will increment the current task before going into the function
	INC R18
	STS CURR_TASK,R18
	sei  ;we are enabling interrupts before call because there may be an infinite loop inside
	icall
	

	reti
	


;in this subroutine we will clear the whole data memory locations from 0x60 to 0x100
memset_data:
	
	ldi R26,$60
	ldi R27,$00
	ldi R16,$00
	ldi R17,100
loop:	st X+,R16
		dec R17
		brne loop
	
ret

;R24 should contain the lower byte address of the function
;R25 should contain the higher byte address of the function

add_task:
	;do it in atomic operation
	cli
   ;increment the task_count
   lds R16,TASKCOUNT
   ldi R26,LOW(TASK_START)
   ldi R27,HIGH(TASK_START)
	MOV R17,R16
	ADD R17,R17
	breq cmp1
	cmp:INC R26
		 DEC R17
		 brne cmp	
cmp1: INC r16
   STS TASKCOUNT,R16
	ST X+,R24
	ST X,R25
  sei
ret




;function that pushes all the registers on the stack
push_reg:

	PUSH r0
	PUSH r1
	PUSH r2
	PUSH r3
	PUSH r4
	PUSH r5
	PUSH r6
	PUSH r7
	PUSH r8
	PUSH r9
	PUSH r10
	PUSH r11
	PUSH r12
	PUSH r13
	PUSH r14
	PUSH r15
	PUSH r16
	PUSH r17
	PUSH r18
	PUSH r19
	PUSH r20
	PUSH r21
	PUSH r22
	PUSH r23
	PUSH r24
	PUSH r25
	PUSH r26
	PUSH r27
	PUSH r28
	PUSH r29
	PUSH r30
	PUSH r31
	ret


pop_reg:

	POP r31
	POP r30
	POP r29
	POP r28
	POP r27
	POP r26
	POP r25
	POP r24
	POP r23
	POP r22
	POP r21
	POP r20
	POP r19
	POP r18
	POP r17
	POP r16
	POP r15
	POP r14
	POP r13
	POP r12
	POP r11
	POP r10
	POP r9
	POP r8
	POP r7
	POP r6
	POP r5
	POP r4
	POP r3
	POP r2
	POP r1
	POP r0
	ret




	
;Let the task1 performs the reading character function from the uart	and writing it to a function
task1:

	;setting the baud rate to 9600
   ldi r16,0x33
   out UBRRL,r16
   ldi r16,0x00
   out UBRRH,r16

	;enable the receiver and transmitter
	ldi r16,(1<<TXEN|1<<RXEN)
	out UCSRB,r16

	;setting to 8 databits
	ldi r16,  (1<<URSEL|1<<UCSZ1|1<<UCSZ0)
	out UCSRC, r16
	
	;Wait to check whether any character is received
	check : sbis UCSRA,RXC
			brne check
 	in r16,UDR

ret


;let the task2 performs the reading function and writing it to the eeprom location
task2:
	
	nop
	nop
ret


;let the task3 do not perform any operation just do lot of nop's
task3:
	nop	
	nop
	nop
   nop
   nop
ret	





