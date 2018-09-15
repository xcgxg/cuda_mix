;.586              ;Target processor.  Use instructions for Pentium class machines
;.MODEL FLAT, C    ;Use the flat memory model. Use C calling conventions
;.STACK            ;Define a stack segment of 1KB (Not required for this example)
;.DATA             ;Create a near data segment.  Local variables are declared after
                  ;this directive (Not required for this example)
.CODE             ;Indicates the start of a code segment.

_text SEGMENT

plsToXmm5 PROC FRAME
	mov rax, rcx
	;mov eax, dword ptr[ebp+8]
	;mov eax, ecx
	movdqa xmm5,xmmword ptr [rax]
	.endprolog
	ret
plsToXmm5 ENDP

xmm5ToPls PROC FRAME
	mov rax, rcx
	;mov eax, dword ptr[ebp+8]
	;mov eax, ecx
	movdqa xmmword ptr [rax],xmm5
	.endprolog
	ret
xmm5ToPls ENDP

xorXmm5 PROC FRAME
	xorps  xmm5, xmm5
	.endprolog
	ret
xorXmm5 ENDP

xmm5_xmm4 PROC FRAME
	;mov         rax, qword ptr [rcx]
	;mov			rax,rcx
	;mov eax, dword ptr[ebp+8]
	;mov eax, ecx
    movdqa      xmm4,xmmword ptr [rcx]
	;movdqa      xmm4,[rcx]
    psubd       xmm5,xmm4
	.endprolog
	ret
xmm5_xmm4 ENDP

_text ENDS

END