.syntax unified
.cpu cortex-m7
.fpu fpv5-d16
.thumb

.global g_pfnVectors
.global Reset_Handler

.section .isr_vector, "a", %progbits
.type g_pfnVectors, %object

g_pfnVectors:
    .word _estack
    .word Reset_Handler
    .word Default_Handler
    .word Default_Handler
    .word Default_Handler
    .word Default_Handler
    .word Default_Handler
    .word 0
    .word 0
    .word 0
    .word 0
    .word SVC_Handler
    .word Default_Handler
    .word 0
    .word PendSV_Handler
    .word SysTick_Handler

.section .text.Reset_Handler, "ax", %progbits
.type Reset_Handler, %function

Reset_Handler:
    ldr r0, =_estack
    mov sp, r0

    ldr r0, =_sdata
    ldr r1, =_edata
    ldr r2, =_sidata
CopyData:
    cmp r0, r1
    bcc CopyDataWord
    b ZeroBss
CopyDataWord:
    ldr r3, [r2], #4
    str r3, [r0], #4
    b CopyData

ZeroBss:
    ldr r0, =_sbss
    ldr r1, =_ebss
    movs r2, #0
ZeroBssLoop:
    cmp r0, r1
    bcc ZeroBssWord
    b CallSystemInit
ZeroBssWord:
    str r2, [r0], #4
    b ZeroBssLoop

CallSystemInit:
    bl SystemInit
    bl __libc_init_array
    bl main
    b .

.size Reset_Handler, .-Reset_Handler
