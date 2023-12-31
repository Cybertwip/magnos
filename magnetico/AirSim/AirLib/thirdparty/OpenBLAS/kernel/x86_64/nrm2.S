/*********************************************************************/
/* Copyright 2009, 2010 The University of Texas at Austin.           */
/* All rights reserved.                                              */
/*                                                                   */
/* Redistribution and use in source and binary forms, with or        */
/* without modification, are permitted provided that the following   */
/* conditions are met:                                               */
/*                                                                   */
/*   1. Redistributions of source code must retain the above         */
/*      copyright notice, this list of conditions and the following  */
/*      disclaimer.                                                  */
/*                                                                   */
/*   2. Redistributions in binary form must reproduce the above      */
/*      copyright notice, this list of conditions and the following  */
/*      disclaimer in the documentation and/or other materials       */
/*      provided with the distribution.                              */
/*                                                                   */
/*    THIS  SOFTWARE IS PROVIDED  BY THE  UNIVERSITY OF  TEXAS AT    */
/*    AUSTIN  ``AS IS''  AND ANY  EXPRESS OR  IMPLIED WARRANTIES,    */
/*    INCLUDING, BUT  NOT LIMITED  TO, THE IMPLIED  WARRANTIES OF    */
/*    MERCHANTABILITY  AND FITNESS FOR  A PARTICULAR  PURPOSE ARE    */
/*    DISCLAIMED.  IN  NO EVENT SHALL THE UNIVERSITY  OF TEXAS AT    */
/*    AUSTIN OR CONTRIBUTORS BE  LIABLE FOR ANY DIRECT, INDIRECT,    */
/*    INCIDENTAL,  SPECIAL, EXEMPLARY,  OR  CONSEQUENTIAL DAMAGES    */
/*    (INCLUDING, BUT  NOT LIMITED TO,  PROCUREMENT OF SUBSTITUTE    */
/*    GOODS  OR  SERVICES; LOSS  OF  USE,  DATA,  OR PROFITS;  OR    */
/*    BUSINESS INTERRUPTION) HOWEVER CAUSED  AND ON ANY THEORY OF    */
/*    LIABILITY, WHETHER  IN CONTRACT, STRICT  LIABILITY, OR TORT    */
/*    (INCLUDING NEGLIGENCE OR OTHERWISE)  ARISING IN ANY WAY OUT    */
/*    OF  THE  USE OF  THIS  SOFTWARE,  EVEN  IF ADVISED  OF  THE    */
/*    POSSIBILITY OF SUCH DAMAGE.                                    */
/*                                                                   */
/* The views and conclusions contained in the software and           */
/* documentation are those of the authors and should not be          */
/* interpreted as representing official policies, either expressed   */
/* or implied, of The University of Texas at Austin.                 */
/*********************************************************************/

#define ASSEMBLER
#include "common.h"

#define M	ARG1	/* rdi */
#define X	ARG2	/* rsi */
#define INCX	ARG3	/* rdx */

#define I	%rax

#include "l1param.h"

	PROLOGUE
	PROFCODE

#ifdef WINDOWS_ABI
	emms
#endif

	fldz
	testq	M, M
	jle	.L999
	testq	INCX, INCX
	je	.L999

	salq	$BASE_SHIFT, INCX

	fldz
	fldz
	fldz
	cmpq	$SIZE, INCX
	jne	.L40

	movq	M,  I
	sarq	$3, I
	jle	.L20
	ALIGN_4

.L10:
#ifdef PREFETCH
	PREFETCH (PREFETCHSIZE +  0) - PREOFFSET(X)
#endif

	FLD	0 * SIZE(X)
	fmul	%st(0), %st
	FLD	1 * SIZE(X)
	fmul	%st(0), %st
	FLD	2 * SIZE(X)
	fmul	%st(0), %st
	FLD	3 * SIZE(X)
	fmul	%st(0), %st

	faddp	%st, %st(7)
	faddp	%st, %st(5)
	faddp	%st, %st(3)
	faddp	%st, %st(1)

	FLD	4 * SIZE(X)
	fmul	%st(0), %st
	FLD	5 * SIZE(X)
	fmul	%st(0), %st
	FLD	6 * SIZE(X)
	fmul	%st(0), %st
	FLD	7 * SIZE(X)
	fmul	%st(0), %st

	addq	$8 * SIZE, X

	faddp	%st, %st(7)
	faddp	%st, %st(5)
	faddp	%st, %st(3)
	faddp	%st, %st(1)

	decq	I
	jg	.L10
	ALIGN_4

.L20:
	andq	$7,  M
	jle	.L998
	ALIGN_4


.L21:
	FLD	(X)
	fmul	%st(0), %st
	faddp	%st,%st(1)
	addq	$1 * SIZE, X
	decq	M
	jg	.L21
	jmp	.L998
	ALIGN_4

.L40:
	movq	M, I
	sarq	$3,   I
	jle	.L60
	ALIGN_4

.L50:
	FLD	(X)
	addq	INCX, X
	fmul	%st(0), %st
	FLD	(X)
	addq	INCX, X
	fmul	%st(0), %st
	FLD	(X)
	addq	INCX, X
	fmul	%st(0), %st
	FLD	(X)
	addq	INCX, X
	fmul	%st(0), %st

	faddp	%st, %st(7)
	faddp	%st, %st(5)
	faddp	%st, %st(3)
	faddp	%st, %st(1)

	FLD	(X)
	addq	INCX, X
	fmul	%st(0), %st
	FLD	(X)
	addq	INCX, X
	fmul	%st(0), %st
	FLD	(X)
	addq	INCX, X
	fmul	%st(0), %st
	FLD	(X)
	addq	INCX, X
	fmul	%st(0), %st

	faddp	%st, %st(7)
	faddp	%st, %st(5)
	faddp	%st, %st(3)
	faddp	%st, %st(1)

	decq	I
	jg	.L50
	ALIGN_4

.L60:
	andq	$7,  M
	jle	.L998
	ALIGN_4


.L61:
	FLD	(X)
	addq	INCX, X
	fmul	%st(0), %st
	faddp	%st,%st(1)
	decq	M
	jg	.L61
	ALIGN_4

.L998:
	faddp	%st,%st(2)
	faddp	%st,%st(1)
	faddp	%st,%st(1)
	ALIGN_4

.L999:
	fsqrt
#ifndef XDOUBLE
	sub	$2 * SIZE, %rsp
	FST	(%rsp)
	MOVSD	(%rsp), %xmm0
	add	$2 * SIZE, %rsp
#endif
	ret

	EPILOGUE

