
; Copyright (c) 2021 ozforester. All rights reserved.
; Use of this source code is goverened by a MIT license
; that can be found in the LICENSE file.

; ATmega8 @4MHz internal osc. (H:D9, L:23)
; scans twi bus for pcf8574 extension
; and instructs "remote" P7 to be toggled on and off rep.
; if 16x2 lcd is connected its backlight led also blinks
;
; If used in quasi-bidirectional environment you must check if port is not pulled
; down by its local periphery before writing him to be high remotely etc..
;
; LED on D7 port will blink in case of TWI error or if device scan failed.
;
; Receiving a button press via extension is the next task
; coming back to quasi-bidirectional cause need reading external
; error detection ; may be counter inside flag wait loop

#include <avr/io.h>

#undef  __SFR_OFFSET
#define __SFR_OFFSET 0
#define START 	  0x08
#define REP_START 0x10
#define SLAW_ACK  0x18
#define SLAW_NACK 0x20
#define DATA_ACK  0x28
#define DATA_NACK 0x30

temp    = 16			; common reg
counter = 17			; delay counter reg
code    = 18			; blink code reg
address = 19			; twi address reg
data    = 20			; twi data reg

.section .text			; code section
.global __vector_default        ; Note [10] nongnu.org assembly
.global TIMER0_OVF_vect         ; doc
.global main                    ;

main:				;
	rcall led_port_init	; D7 out,low
	rcall timer_init	; timer0
	rcall twi_init		; 100kHz
	rcall twi_scan		; downwards 127..0 (0 = none answered)
        cpi address, 0          ; there was no answer?
        brne main1              ;
	rcall error_blink	; error blink
	main1:			;
	rcall pcf_init		;
	rcall pcf_led_on_off	; light led connected to pcf8574
	loop:			;
	rjmp loop		; waste a time
ret				;

pcf_led_on_off:			;
	pcfled:			;
	sbr data, 1<<3		; P3 high
	rcall twi_start		;
	rcall twi_sla_w		;
	rcall twi_data_w	;
        rcall twi_stop          ;
        ldi counter, 5          ; delay
        rcall delay             ;
        rcall twi_start         ;
        rcall twi_sla_w         ;
        cbr data, 1<<3          ; P3 low
        rcall twi_data_w        ;
        rcall twi_stop          ;
        ldi counter, 5          ; delay
        rcall delay             ;
	rjmp pcf_led_on_off	;
ret				;

pcf_init:			;
	nop			;
ret				;

twi_scan:			;
	ldi address, 127	; will step backward
	twscan1:		;
	rcall twi_start		; start/rep_start
	rcall twi_sla_w		; slave addres + w
	cpi temp, 0xFF		; touched?
	brne twscan2		; if not cleared
	clr temp		; (temp=0x00)
	subi address, 1		; else next address
	cpi address, 0		; last address
	breq twscan2		; exit with zero address
	rjmp twscan1		;
	twscan2:		;
	rcall twi_stop		;
ret				;

twi_data_w:					;
	mov temp, data				;
	out TWDR, temp				; data
	ldi temp, ((1<<TWINT)|(1<<TWEN))	;
	out TWCR, temp				; go
	tdw1:					;
	in temp, TWCR				;
	sbrs temp, TWINT			; sent?
	rjmp tdw1				; not yet, repeat
	in temp, TWSR				; ok, read status
	andi temp, 0xF8				;
	cpi temp, DATA_ACK			;
	breq tdw2				; ok
        rcall error_blink                       ; error (blinking loop)
	tdw2:					;
ret						;

twi_sla_w:					;
	mov temp, address			; slave address (e.g. 0x3F as pcf of mine)
	lsl temp				; address<<  + 0, R/W = W
	out TWDR, temp				;
	ldi temp, ((1<<TWINT)|(1<<TWEN))	;
	out TWCR, temp				; go
	slaw1:					;
        in temp, TWCR                           ;
        sbrs temp, TWINT                        ; sent
        rjmp slaw1                              ;
        in temp, TWSR                           ; read status
        andi temp, 0xF8                         ;
        cpi temp, SLAW_ACK                      ;
	breq slaw2				; acked (stay temp untouched)
	;rcall error_blink			; uncomment if used w/o autodetect
	ser temp				; else (temp=0xFF)
	slaw2:					;
ret						;

twi_start:						;
	ldi temp, ((1<<TWINT)|(1<<TWSTA)|(1<<TWEN))	;
	out TWCR, temp					; go
	start1:						;
	in temp, TWCR					;
	sbrs temp, TWINT				; sent
        rjmp start1                                     ;
	in temp, TWSR					; read status
	andi temp, 0xF8					;
	cpi temp, START					;
	breq start2					;
        cpi temp, REP_START                             ;
        breq start2                                     ;
	rcall error_blink				; error (blinking loop)
	start2:						;
ret							; acked

twi_stop:						;
	ldi temp, ((1<<TWINT)|(1<<TWEN)|(1<<TWSTO))	;
        out TWCR, temp                                  ;
ret							;

error_blink:			;
        eb:                     ;
        sbi PORTD, 7            ; D7 high
        ldi counter, 1          ; delay on
        rcall delay             ;
        cbi PORTD, 7            ; D7 low
        ldi counter, 3          ; delay off
        rcall delay             ;
        rjmp eb                 ;
ret				;

delay:                          ; counter itself decrements by t0 isr
        cpi counter, 0          ; just wait 0
        brne delay              ;
ret               		; voila

led_port_init:			;
	sbi DDRD, 7		; D7 output
	cbi PORTD, 7		; D7 low
ret				;

twi_init:			; 100 kHz @4MHz clock
	ldi temp, 0b00001100	; 4000000 / ( 16 + 2*TWBR * 4^TWPS )
	out TWBR, temp		; 12
        in temp, TWSR	        ;
	ori temp, 0b00000011	; 64
	out TWSR, temp		;
ret				;

timer_init:			; TIMER0 (65 ms @4MHz)
        ldi temp, (1<<CS00)     ;
        ori temp, (1<<CS02)     ;
        out TCCR0, temp         ;
        ldi temp, (1<<TOIE0)    ;
        out TIMSK, temp         ;
        sei                     ;
ret				;

TIMER0_OVF_vect:		; counter--
	push temp		;
	in temp, SREG		; z in main loop
	subi counter, 1		;
	out SREG, temp		;
	pop temp		;
reti				;

__vector_default:		;
	reti			;

.end
