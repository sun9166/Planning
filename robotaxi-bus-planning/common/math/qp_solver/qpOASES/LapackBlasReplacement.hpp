/*
 *	This file is part of qpOASES.
 *
 *	qpOASES -- An Implementation of the Online Active Set Strategy.
 *	Copyright (C) 2007-2017 by Hans Joachim Ferreau, Andreas Potschka,
 *	Christian Kirches et al. All rights reserved.
 *
 *	qpOASES is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU Lesser General Public
 *	License as published by the Free Software Foundation; either
 *	version 2.1 of the License, or (at your option) any later version.
 *
 *	qpOASES is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *	See the GNU Lesser General Public License for more details.
 *
 *	You should have received a copy of the GNU Lesser General Public
 *	License along with qpOASES; if not, write to the Free Software
 *	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */


/**
 *	\file include/qpOASES/LapackBlasReplacement.hpp
 *	\author Andreas Potschka, Hans Joachim Ferreau, Christian Kirches
 *	\version 3.2
 *	\date 2009-2017
 *
 *  Declarations for external LAPACK/BLAS functions.
 */



#ifndef QPOASES_LAPACKBLASREPLACEMENT_HPP
#define QPOASES_LAPACKBLASREPLACEMENT_HPP


#ifdef __USE_SINGLE_PRECISION__

	/** Macro for calling level 3 BLAS operation in single precision. */
	#define QPOASES_GEMM qp_oases::blas::qp_oases_sgemm_
	/** Macro for calling level 3 BLAS operation in single precision. */
	#define QPOASES_SYR qp_oases::blas::qp_oases_ssyr_
	/** Macro for calling level 3 BLAS operation in single precision. */
	#define QPOASES_SYR2 qp_oases::blas::qp_oases_ssyr2_
	/** Macro for calling level 3 BLAS operation in single precision. */
	#define QPOASES_POTRF qp_oases::blas::qp_oases_spotrf_

	/** Macro for calling level 3 BLAS operation in single precision. */
	/* #define QPOASES_GEQRF qp_oases::blas::qp_oase_sgeqrf_ */
	/** Macro for calling level 3 BLAS operation in single precision. */
	/* #define QPOASES_ORMQR qp_oases::blas::qp_oase_sormqr_ */
	/** Macro for calling level 3 BLAS operation in single precision. */
	#define QPOASES_TRTRS qp_oases::blas::qp_oases_strtrs_
	/** Macro for calling level 3 BLAS operation in single precision. */
	#define QPOASES_TRCON qp_oases::blas::qp_oases_strcon_

#else

	/** Macro for calling level 3 BLAS operation in double precision. */
	#define QPOASES_GEMM qp_oases::blas::qp_oases_dgemm_
	/** Macro for calling level 3 BLAS operation in double precision. */
	#define QPOASES_SYR  qp_oases::blas::qp_oases_dsyr_
	/** Macro for calling level 3 BLAS operation in double precision. */
	#define QPOASES_SYR2 qp_oases::blas::qp_oases_dsyr2_
	/** Macro for calling level 3 BLAS operation in double precision. */
	#define QPOASES_POTRF qp_oases::blas::qp_oases_dpotrf_

	/** Macro for calling level 3 BLAS operation in double precision. */
	/* #define QPOASES_GEQRF qp_oases::blas::qp_oases_dgeqrf_ */
	/** Macro for calling level 3 BLAS operation in double precision. */
	/* #define QPOASES_ORMQR qp_oases::blas::qp_oases_dormqr_ */
	/** Macro for calling level 3 BLAS operation in double precision. */
	#define QPOASES_TRTRS qp_oases::blas::qp_oases_dtrtrs_
	/** Macro for calling level 3 BLAS operation in double precision. */
	#define QPOASES_TRCON qp_oases::blas::qp_oases_dtrcon_

#endif /* __USE_SINGLE_PRECISION__ */

namespace qp_oases {
namespace blas {

extern "C"
{
	/** Performs one of the matrix-matrix operation in double precision. */
	void qp_oases_dgemm_(	const char*, const char*, const la_uint_t*, const la_uint_t*, const la_uint_t*,
					const double*, const double*, const la_uint_t*, const double*, const la_uint_t*,
					const double*, double*, const la_uint_t* );
	/** Performs one of the matrix-matrix operation in single precision. */
	void qp_oases_sgemm_(	const char*, const char*, const la_uint_t*, const la_uint_t*, const la_uint_t*,
					const float*, const float*, const la_uint_t*, const float*, const la_uint_t*,
					const float*, float*, const la_uint_t* );

	/** Performs a symmetric rank 1 operation in double precision. */
	void qp_oases_dsyr_(		const char*, const la_uint_t*, const double*, const double*,
					const la_uint_t*, double*, const la_uint_t* );
	/** Performs a symmetric rank 1 operation in single precision. */
	void qp_oases_ssyr_(		const char*, const la_uint_t*, const float*, const float*,
					const la_uint_t*, float*, const la_uint_t* );

	/** Performs a symmetric rank 2 operation in double precision. */
	void qp_oases_dsyr2_(	const char*, const la_uint_t*, const double*, const double*,
					const la_uint_t*, const double*, const la_uint_t*, double*, const la_uint_t*);
	/** Performs a symmetric rank 2 operation in single precision. */
	void qp_oases_ssyr2_(	const char*, const la_uint_t*, const float*, const float*,
					const la_uint_t*, const float*, const la_uint_t*, float*, const la_uint_t*);

	/** Calculates the Cholesky factorization of a real symmetric positive definite matrix in double precision. */
	void qp_oases_dpotrf_(	const char*, const la_uint_t*, double*, const la_uint_t*, la_int_t* );
	/** Calculates the Cholesky factorization of a real symmetric positive definite matrix in single precision. */
	void qp_oases_spotrf_(	const char*, const la_uint_t*, float*, const la_uint_t*, la_int_t* );


	/** Computes a QR factorization of a real M-by-N matrix A in double precision */
	/* void qp_oases_dgeqrf_(	const la_uint_t* M, const la_uint_t* N, double* A, const la_uint_t* LDA,
						double* TAU, double* WORK, const la_uint_t* LWORK, int *INFO );*/
	/** Computes a QR factorization of a real M-by-N matrix A in single precision */
	/* void qp_oases_sgeqrf_(	const la_uint_t* M, const la_uint_t* N, float* A, const la_uint_t* LDA,
						float* TAU, float* WORK, const la_uint_t* LWORK, int *INFO );*/

	/** Multiplies C with orthogonal matrix Q**T as returned by geqrf (double precision) */
	/* void qp_oases_dormqr_(	const char* SIDE, const char* TRANS, const la_uint_t* M, const la_uint_t* N, const la_uint_t* K,
						double* A, const la_uint_t* LDA, double* TAU, double* C, const la_uint_t* LDC,
						double* WORK, const la_uint_t* LWORK, int *INFO );*/
	/** Multiplies C with orthogonal matrix Q**T as returned by geqrf (single precision) */
	/* void qp_oases_sormqr_(	const char* SIDE, const char* TRANS, const la_uint_t* M, const la_uint_t* N, const la_uint_t* K,
						float* A, const la_uint_t* LDA, float* TAU, float* C, const la_uint_t* LDC,
						float* WORK, const la_uint_t* LWORK, int *INFO );*/

	/** Solves a triangular system (double precision) */
	void qp_oases_dtrtrs_(	const char* UPLO, const char* TRANS, const char* DIAG, const la_uint_t* N, const la_uint_t* NRHS,
					double* A, const la_uint_t* LDA, double* B, const la_uint_t* LDB, la_int_t* INFO );
	/** Solves a triangular system (single precision) */
	void qp_oases_strtrs_(	const char* UPLO, const char* TRANS, const char* DIAG, const la_uint_t* N, const la_uint_t* NRHS,
					float* A, const la_uint_t* LDA, float* B, const la_uint_t* LDB, la_int_t* INFO );

	/** Estimate the reciprocal of the condition number of a triangular matrix in double precision */
	void qp_oases_dtrcon_(	const char* NORM, const char* UPLO, const char* DIAG, const la_uint_t* N, double* A, const la_uint_t* LDA,
					double* RCOND, double* WORK, const la_uint_t* IWORK, la_int_t* INFO );
	/** Estimate the reciprocal of the condition number of a triangular matrix in single precision */
	void qp_oases_strcon_(	const char* NORM, const char* UPLO, const char* DIAG, const la_uint_t* N, float* A, const la_uint_t* LDA,
					float* RCOND, float* WORK, const la_uint_t* IWORK, la_int_t* INFO );
}

} // namespace qp_oases
} // namespace blas


#endif	/* QPOASES_LAPACKBLASREPLACEMENT_HPP */
