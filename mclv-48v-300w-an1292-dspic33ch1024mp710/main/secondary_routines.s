;The libpic30 versions of __start_secondary and __stop_secondary are located
;in the same section as the 33CH startup code incompatible with this device.
;Calling those functions will cause the compiler to link the incompatible startup
;code after linking the selected startup code, resulting in a multiple definition error.

;Below are the routines used by these library functions.
;Apparently the same names can be used, and including this file in the project will
;prevent the linker from looking for these functions externally?
    
.text    
    .global __start_secondary
__start_secondary:
	mov 	#0x55, w0
	mov 	w0, MSI1KEY
	mov 	#0xAA, w0
	mov 	w0, MSI1KEY
	bset 	MSI1CON, #15
	return


    .global __stop_secondary
__stop_secondary:
	mov 	#0x55, w0
	mov 	w0, MSI1KEY
	mov 	#0xAA, w0
	mov 	w0, MSI1KEY
	bclr 	MSI1CON, #15
	return

;Centaurus cannot use the other routines used for 33CH devices,
;which involve the removed LDSLV/VFSLV instructions.
