*> \brief \b ZCHKTP
*
*  =========== DOCUMENTATION ===========
*
* Online html documentation available at
*            http://www.netlib.org/lapack/explore-html/
*
*  Definition:
*  ===========
*
*       SUBROUTINE ZCHKTP( DOTYPE, NN, NVAL, NNS, NSVAL, THRESH, TSTERR,
*                          NMAX, AP, AINVP, B, X, XACT, WORK, RWORK,
*                          NOUT )
*
*       .. Scalar Arguments ..
*       LOGICAL            TSTERR
*       INTEGER            NMAX, NN, NNS, NOUT
*       DOUBLE PRECISION   THRESH
*       ..
*       .. Array Arguments ..
*       LOGICAL            DOTYPE( * )
*       INTEGER            NSVAL( * ), NVAL( * )
*       DOUBLE PRECISION   RWORK( * )
*       COMPLEX*16         AINVP( * ), AP( * ), B( * ), WORK( * ), X( * ),
*      $                   XACT( * )
*       ..
*
*
*> \par Purpose:
*  =============
*>
*> \verbatim
*>
*> ZCHKTP tests ZTPTRI, -TRS, -RFS, and -CON, and ZLATPS
*> \endverbatim
*
*  Arguments:
*  ==========
*
*> \param[in] DOTYPE
*> \verbatim
*>          DOTYPE is LOGICAL array, dimension (NTYPES)
*>          The matrix types to be used for testing.  Matrices of type j
*>          (for 1 <= j <= NTYPES) are used for testing if DOTYPE(j) =
*>          .TRUE.; if DOTYPE(j) = .FALSE., then type j is not used.
*> \endverbatim
*>
*> \param[in] NN
*> \verbatim
*>          NN is INTEGER
*>          The number of values of N contained in the vector NVAL.
*> \endverbatim
*>
*> \param[in] NVAL
*> \verbatim
*>          NVAL is INTEGER array, dimension (NN)
*>          The values of the matrix column dimension N.
*> \endverbatim
*>
*> \param[in] NNS
*> \verbatim
*>          NNS is INTEGER
*>          The number of values of NRHS contained in the vector NSVAL.
*> \endverbatim
*>
*> \param[in] NSVAL
*> \verbatim
*>          NSVAL is INTEGER array, dimension (NNS)
*>          The values of the number of right hand sides NRHS.
*> \endverbatim
*>
*> \param[in] THRESH
*> \verbatim
*>          THRESH is DOUBLE PRECISION
*>          The threshold value for the test ratios.  A result is
*>          included in the output file if RESULT >= THRESH.  To have
*>          every test ratio printed, use THRESH = 0.
*> \endverbatim
*>
*> \param[in] TSTERR
*> \verbatim
*>          TSTERR is LOGICAL
*>          Flag that indicates whether error exits are to be tested.
*> \endverbatim
*>
*> \param[in] NMAX
*> \verbatim
*>          NMAX is INTEGER
*>          The leading dimension of the work arrays.  NMAX >= the
*>          maximum value of N in NVAL.
*> \endverbatim
*>
*> \param[out] AP
*> \verbatim
*>          AP is COMPLEX*16 array, dimension (NMAX*(NMAX+1)/2)
*> \endverbatim
*>
*> \param[out] AINVP
*> \verbatim
*>          AINVP is COMPLEX*16 array, dimension (NMAX*(NMAX+1)/2)
*> \endverbatim
*>
*> \param[out] B
*> \verbatim
*>          B is COMPLEX*16 array, dimension (NMAX*NSMAX)
*>          where NSMAX is the largest entry in NSVAL.
*> \endverbatim
*>
*> \param[out] X
*> \verbatim
*>          X is COMPLEX*16 array, dimension (NMAX*NSMAX)
*> \endverbatim
*>
*> \param[out] XACT
*> \verbatim
*>          XACT is COMPLEX*16 array, dimension (NMAX*NSMAX)
*> \endverbatim
*>
*> \param[out] WORK
*> \verbatim
*>          WORK is COMPLEX*16 array, dimension
*>                      (NMAX*max(3,NSMAX))
*> \endverbatim
*>
*> \param[out] RWORK
*> \verbatim
*>          RWORK is DOUBLE PRECISION array, dimension
*>                      (max(NMAX,2*NSMAX))
*> \endverbatim
*>
*> \param[in] NOUT
*> \verbatim
*>          NOUT is INTEGER
*>          The unit number for output.
*> \endverbatim
*
*  Authors:
*  ========
*
*> \author Univ. of Tennessee
*> \author Univ. of California Berkeley
*> \author Univ. of Colorado Denver
*> \author NAG Ltd.
*
*> \ingroup complex16_lin
*
*  =====================================================================
      SUBROUTINE ZCHKTP( DOTYPE, NN, NVAL, NNS, NSVAL, THRESH, TSTERR,
     $                   NMAX, AP, AINVP, B, X, XACT, WORK, RWORK,
     $                   NOUT )
*
*  -- LAPACK test routine --
*  -- LAPACK is a software package provided by Univ. of Tennessee,    --
*  -- Univ. of California Berkeley, Univ. of Colorado Denver and NAG Ltd..--
*
*     .. Scalar Arguments ..
      LOGICAL            TSTERR
      INTEGER            NMAX, NN, NNS, NOUT
      DOUBLE PRECISION   THRESH
*     ..
*     .. Array Arguments ..
      LOGICAL            DOTYPE( * )
      INTEGER            NSVAL( * ), NVAL( * )
      DOUBLE PRECISION   RWORK( * )
      COMPLEX*16         AINVP( * ), AP( * ), B( * ), WORK( * ), X( * ),
     $                   XACT( * )
*     ..
*
*  =====================================================================
*
*     .. Parameters ..
      INTEGER            NTYPE1, NTYPES
      PARAMETER          ( NTYPE1 = 10, NTYPES = 18 )
      INTEGER            NTESTS
      PARAMETER          ( NTESTS = 9 )
      INTEGER            NTRAN
      PARAMETER          ( NTRAN = 3 )
      DOUBLE PRECISION   ONE, ZERO
      PARAMETER          ( ONE = 1.0D+0, ZERO = 0.0D+0 )
*     ..
*     .. Local Scalars ..
      CHARACTER          DIAG, NORM, TRANS, UPLO, XTYPE
      CHARACTER*3        PATH
      INTEGER            I, IDIAG, IMAT, IN, INFO, IRHS, ITRAN, IUPLO,
     $                   K, LAP, LDA, N, NERRS, NFAIL, NRHS, NRUN
      DOUBLE PRECISION   AINVNM, ANORM, RCOND, RCONDC, RCONDI, RCONDO,
     $                   SCALE
*     ..
*     .. Local Arrays ..
      CHARACTER          TRANSS( NTRAN ), UPLOS( 2 )
      INTEGER            ISEED( 4 ), ISEEDY( 4 )
      DOUBLE PRECISION   RESULT( NTESTS )
*     ..
*     .. External Functions ..
      LOGICAL            LSAME
      DOUBLE PRECISION   ZLANTP
      EXTERNAL           LSAME, ZLANTP
*     ..
*     .. External Subroutines ..
      EXTERNAL           ALAERH, ALAHD, ALASUM, ZCOPY, ZERRTR, ZGET04,
     $                   ZLACPY, ZLARHS, ZLATPS, ZLATTP, ZTPCON, ZTPRFS,
     $                   ZTPT01, ZTPT02, ZTPT03, ZTPT05, ZTPT06, ZTPTRI,
     $                   ZTPTRS
*     ..
*     .. Scalars in Common ..
      LOGICAL            LERR, OK
      CHARACTER*32       SRNAMT
      INTEGER            INFOT, IOUNIT
*     ..
*     .. Common blocks ..
      COMMON             / INFOC / INFOT, IOUNIT, OK, LERR
      COMMON             / SRNAMC / SRNAMT
*     ..
*     .. Intrinsic Functions ..
      INTRINSIC          MAX
*     ..
*     .. Data statements ..
      DATA               ISEEDY / 1988, 1989, 1990, 1991 /
      DATA               UPLOS / 'U', 'L' / , TRANSS / 'N', 'T', 'C' /
*     ..
*     .. Executable Statements ..
*
*     Initialize constants and the random number seed.
*
      PATH( 1: 1 ) = 'Zomplex precision'
      PATH( 2: 3 ) = 'TP'
      NRUN = 0
      NFAIL = 0
      NERRS = 0
      DO 10 I = 1, 4
         ISEED( I ) = ISEEDY( I )
   10 CONTINUE
*
*     Test the error exits
*
      IF( TSTERR )
     $   CALL ZERRTR( PATH, NOUT )
      INFOT = 0
*
      DO 110 IN = 1, NN
*
*        Do for each value of N in NVAL
*
         N = NVAL( IN )
         LDA = MAX( 1, N )
         LAP = LDA*( LDA+1 ) / 2
         XTYPE = 'N'
*
         DO 70 IMAT = 1, NTYPE1
*
*           Do the tests only if DOTYPE( IMAT ) is true.
*
            IF( .NOT.DOTYPE( IMAT ) )
     $         GO TO 70
*
            DO 60 IUPLO = 1, 2
*
*              Do first for UPLO = 'U', then for UPLO = 'L'
*
               UPLO = UPLOS( IUPLO )
*
*              Call ZLATTP to generate a triangular test matrix.
*
               SRNAMT = 'ZLATTP'
               CALL ZLATTP( IMAT, UPLO, 'No transpose', DIAG, ISEED, N,
     $                      AP, X, WORK, RWORK, INFO )
*
*              Set IDIAG = 1 for non-unit matrices, 2 for unit.
*
               IF( LSAME( DIAG, 'N' ) ) THEN
                  IDIAG = 1
               ELSE
                  IDIAG = 2
               END IF
*
*+    TEST 1
*              Form the inverse of A.
*
               IF( N.GT.0 )
     $            CALL ZCOPY( LAP, AP, 1, AINVP, 1 )
               SRNAMT = 'ZTPTRI'
               CALL ZTPTRI( UPLO, DIAG, N, AINVP, INFO )
*
*              Check error code from ZTPTRI.
*
               IF( INFO.NE.0 )
     $            CALL ALAERH( PATH, 'ZTPTRI', INFO, 0, UPLO // DIAG, N,
     $                         N, -1, -1, -1, IMAT, NFAIL, NERRS, NOUT )
*
*              Compute the infinity-norm condition number of A.
*
               ANORM = ZLANTP( 'I', UPLO, DIAG, N, AP, RWORK )
               AINVNM = ZLANTP( 'I', UPLO, DIAG, N, AINVP, RWORK )
               IF( ANORM.LE.ZERO .OR. AINVNM.LE.ZERO ) THEN
                  RCONDI = ONE
               ELSE
                  RCONDI = ( ONE / ANORM ) / AINVNM
               END IF
*
*              Compute the residual for the triangular matrix times its
*              inverse.  Also compute the 1-norm condition number of A.
*
               CALL ZTPT01( UPLO, DIAG, N, AP, AINVP, RCONDO, RWORK,
     $                      RESULT( 1 ) )
*
*              Print the test ratio if it is .GE. THRESH.
*
               IF( RESULT( 1 ).GE.THRESH ) THEN
                  IF( NFAIL.EQ.0 .AND. NERRS.EQ.0 )
     $               CALL ALAHD( NOUT, PATH )
                  WRITE( NOUT, FMT = 9999 )UPLO, DIAG, N, IMAT, 1,
     $               RESULT( 1 )
                  NFAIL = NFAIL + 1
               END IF
               NRUN = NRUN + 1
*
               DO 40 IRHS = 1, NNS
                  NRHS = NSVAL( IRHS )
                  XTYPE = 'N'
*
                  DO 30 ITRAN = 1, NTRAN
*
*                 Do for op(A) = A, A**T, or A**H.
*
                     TRANS = TRANSS( ITRAN )
                     IF( ITRAN.EQ.1 ) THEN
                        NORM = 'O'
                        RCONDC = RCONDO
                     ELSE
                        NORM = 'I'
                        RCONDC = RCONDI
                     END IF
*
*+    TEST 2
*                 Solve and compute residual for op(A)*x = b.
*
                     SRNAMT = 'ZLARHS'
                     CALL ZLARHS( PATH, XTYPE, UPLO, TRANS, N, N, 0,
     $                            IDIAG, NRHS, AP, LAP, XACT, LDA, B,
     $                            LDA, ISEED, INFO )
                     XTYPE = 'C'
                     CALL ZLACPY( 'Full', N, NRHS, B, LDA, X, LDA )
*
                     SRNAMT = 'ZTPTRS'
                     CALL ZTPTRS( UPLO, TRANS, DIAG, N, NRHS, AP, X,
     $                            LDA, INFO )
*
*                 Check error code from ZTPTRS.
*
                     IF( INFO.NE.0 )
     $                  CALL ALAERH( PATH, 'ZTPTRS', INFO, 0,
     $                               UPLO // TRANS // DIAG, N, N, -1,
     $                               -1, -1, IMAT, NFAIL, NERRS, NOUT )
*
                     CALL ZTPT02( UPLO, TRANS, DIAG, N, NRHS, AP, X,
     $                            LDA, B, LDA, WORK, RWORK,
     $                            RESULT( 2 ) )
*
*+    TEST 3
*                 Check solution from generated exact solution.
*
                     CALL ZGET04( N, NRHS, X, LDA, XACT, LDA, RCONDC,
     $                            RESULT( 3 ) )
*
*+    TESTS 4, 5, and 6
*                 Use iterative refinement to improve the solution and
*                 compute error bounds.
*
                     SRNAMT = 'ZTPRFS'
                     CALL ZTPRFS( UPLO, TRANS, DIAG, N, NRHS, AP, B,
     $                            LDA, X, LDA, RWORK, RWORK( NRHS+1 ),
     $                            WORK, RWORK( 2*NRHS+1 ), INFO )
*
*                 Check error code from ZTPRFS.
*
                     IF( INFO.NE.0 )
     $                  CALL ALAERH( PATH, 'ZTPRFS', INFO, 0,
     $                               UPLO // TRANS // DIAG, N, N, -1,
     $                               -1, NRHS, IMAT, NFAIL, NERRS,
     $                               NOUT )
*
                     CALL ZGET04( N, NRHS, X, LDA, XACT, LDA, RCONDC,
     $                            RESULT( 4 ) )
                     CALL ZTPT05( UPLO, TRANS, DIAG, N, NRHS, AP, B,
     $                            LDA, X, LDA, XACT, LDA, RWORK,
     $                            RWORK( NRHS+1 ), RESULT( 5 ) )
*
*                    Print information about the tests that did not pass
*                    the threshold.
*
                     DO 20 K = 2, 6
                        IF( RESULT( K ).GE.THRESH ) THEN
                           IF( NFAIL.EQ.0 .AND. NERRS.EQ.0 )
     $                        CALL ALAHD( NOUT, PATH )
                           WRITE( NOUT, FMT = 9998 )UPLO, TRANS, DIAG,
     $                        N, NRHS, IMAT, K, RESULT( K )
                           NFAIL = NFAIL + 1
                        END IF
   20                CONTINUE
                     NRUN = NRUN + 5
   30             CONTINUE
   40          CONTINUE
*
*+    TEST 7
*                 Get an estimate of RCOND = 1/CNDNUM.
*
               DO 50 ITRAN = 1, 2
                  IF( ITRAN.EQ.1 ) THEN
                     NORM = 'O'
                     RCONDC = RCONDO
                  ELSE
                     NORM = 'I'
                     RCONDC = RCONDI
                  END IF
                  SRNAMT = 'ZTPCON'
                  CALL ZTPCON( NORM, UPLO, DIAG, N, AP, RCOND, WORK,
     $                         RWORK, INFO )
*
*                 Check error code from ZTPCON.
*
                  IF( INFO.NE.0 )
     $               CALL ALAERH( PATH, 'ZTPCON', INFO, 0,
     $                            NORM // UPLO // DIAG, N, N, -1, -1,
     $                            -1, IMAT, NFAIL, NERRS, NOUT )
*
                  CALL ZTPT06( RCOND, RCONDC, UPLO, DIAG, N, AP, RWORK,
     $                         RESULT( 7 ) )
*
*                 Print the test ratio if it is .GE. THRESH.
*
                  IF( RESULT( 7 ).GE.THRESH ) THEN
                     IF( NFAIL.EQ.0 .AND. NERRS.EQ.0 )
     $                  CALL ALAHD( NOUT, PATH )
                     WRITE( NOUT, FMT = 9997 ) 'ZTPCON', NORM, UPLO,
     $                  DIAG, N, IMAT, 7, RESULT( 7 )
                     NFAIL = NFAIL + 1
                  END IF
                  NRUN = NRUN + 1
   50          CONTINUE
   60       CONTINUE
   70    CONTINUE
*
*        Use pathological test matrices to test ZLATPS.
*
         DO 100 IMAT = NTYPE1 + 1, NTYPES
*
*           Do the tests only if DOTYPE( IMAT ) is true.
*
            IF( .NOT.DOTYPE( IMAT ) )
     $         GO TO 100
*
            DO 90 IUPLO = 1, 2
*
*              Do first for UPLO = 'U', then for UPLO = 'L'
*
               UPLO = UPLOS( IUPLO )
               DO 80 ITRAN = 1, NTRAN
*
*                 Do for op(A) = A, A**T, or A**H.
*
                  TRANS = TRANSS( ITRAN )
*
*                 Call ZLATTP to generate a triangular test matrix.
*
                  SRNAMT = 'ZLATTP'
                  CALL ZLATTP( IMAT, UPLO, TRANS, DIAG, ISEED, N, AP, X,
     $                         WORK, RWORK, INFO )
*
*+    TEST 8
*                 Solve the system op(A)*x = b.
*
                  SRNAMT = 'ZLATPS'
                  CALL ZCOPY( N, X, 1, B, 1 )
                  CALL ZLATPS( UPLO, TRANS, DIAG, 'N', N, AP, B, SCALE,
     $                         RWORK, INFO )
*
*                 Check error code from ZLATPS.
*
                  IF( INFO.NE.0 )
     $               CALL ALAERH( PATH, 'ZLATPS', INFO, 0,
     $                            UPLO // TRANS // DIAG // 'N', N, N,
     $                            -1, -1, -1, IMAT, NFAIL, NERRS, NOUT )
*
                  CALL ZTPT03( UPLO, TRANS, DIAG, N, 1, AP, SCALE,
     $                         RWORK, ONE, B, LDA, X, LDA, WORK,
     $                         RESULT( 8 ) )
*
*+    TEST 9
*                 Solve op(A)*x = b again with NORMIN = 'Y'.
*
                  CALL ZCOPY( N, X, 1, B( N+1 ), 1 )
                  CALL ZLATPS( UPLO, TRANS, DIAG, 'Y', N, AP, B( N+1 ),
     $                         SCALE, RWORK, INFO )
*
*                 Check error code from ZLATPS.
*
                  IF( INFO.NE.0 )
     $               CALL ALAERH( PATH, 'ZLATPS', INFO, 0,
     $                            UPLO // TRANS // DIAG // 'Y', N, N,
     $                            -1, -1, -1, IMAT, NFAIL, NERRS, NOUT )
*
                  CALL ZTPT03( UPLO, TRANS, DIAG, N, 1, AP, SCALE,
     $                         RWORK, ONE, B( N+1 ), LDA, X, LDA, WORK,
     $                         RESULT( 9 ) )
*
*                 Print information about the tests that did not pass
*                 the threshold.
*
                  IF( RESULT( 8 ).GE.THRESH ) THEN
                     IF( NFAIL.EQ.0 .AND. NERRS.EQ.0 )
     $                  CALL ALAHD( NOUT, PATH )
                     WRITE( NOUT, FMT = 9996 )'ZLATPS', UPLO, TRANS,
     $                  DIAG, 'N', N, IMAT, 8, RESULT( 8 )
                     NFAIL = NFAIL + 1
                  END IF
                  IF( RESULT( 9 ).GE.THRESH ) THEN
                     IF( NFAIL.EQ.0 .AND. NERRS.EQ.0 )
     $                  CALL ALAHD( NOUT, PATH )
                     WRITE( NOUT, FMT = 9996 )'ZLATPS', UPLO, TRANS,
     $                  DIAG, 'Y', N, IMAT, 9, RESULT( 9 )
                     NFAIL = NFAIL + 1
                  END IF
                  NRUN = NRUN + 2
   80          CONTINUE
   90       CONTINUE
  100    CONTINUE
  110 CONTINUE
*
*     Print a summary of the results.
*
      CALL ALASUM( PATH, NOUT, NFAIL, NRUN, NERRS )
*
 9999 FORMAT( ' UPLO=''', A1, ''', DIAG=''', A1, ''', N=', I5,
     $      ', type ', I2, ', test(', I2, ')= ', G12.5 )
 9998 FORMAT( ' UPLO=''', A1, ''', TRANS=''', A1, ''', DIAG=''', A1,
     $      ''', N=', I5, ''', NRHS=', I5, ', type ', I2, ', test(',
     $      I2, ')= ', G12.5 )
 9997 FORMAT( 1X, A, '( ''', A1, ''', ''', A1, ''', ''', A1, ''',',
     $      I5, ', ... ), type ', I2, ', test(', I2, ')=', G12.5 )
 9996 FORMAT( 1X, A, '( ''', A1, ''', ''', A1, ''', ''', A1, ''', ''',
     $      A1, ''',', I5, ', ... ), type ', I2, ', test(', I2, ')=',
     $      G12.5 )
      RETURN
*
*     End of ZCHKTP
*
      END
