/* ========================================================================== */
/* === colamd mexFunction =================================================== */
/* ========================================================================== */

/* Usage:

	P = colamd2 (A) ;
	[ P, stats ] = colamd2 (A, knobs) ;

    see colamd.m for a description.

    Authors:

	The authors of the code itself are Stefan I. Larimore and Timothy A.
	Davis (DrTimothyAldenDavis@gmail.com).  The algorithm was
	developed in collaboration with John Gilbert, Xerox PARC, and Esmond
	Ng, Oak Ridge National Laboratory.

    Acknowledgements:

	This work was supported by the National Science Foundation, under
	grants DMS-9504974 and DMS-9803599.

    Notice:

	Copyright (c) 1998-2007, Timothy A. Davis, All Rights Reserved.

    Availability:

	The colamd/symamd library is available at http://www.suitesparse.com

*/

/* ========================================================================== */
/* === Include files ======================================================== */
/* ========================================================================== */

#include "colamd.h"
#include "mex.h"
#include "matrix.h"
#include <stdlib.h>
#include <string.h>
#define Long SuiteSparse_long

/* ========================================================================== */
/* === colamd mexFunction =================================================== */
/* ========================================================================== */

void mexFunction
(
    /* === Parameters ======================================================= */

    int nlhs,			/* number of left-hand sides */
    mxArray *plhs [],		/* left-hand side matrices */
    int nrhs,			/* number of right--hand sides */
    const mxArray *prhs []	/* right-hand side matrices */
)
{
    /* === Local variables ================================================== */

    Long *A ;                   /* colamd's copy of the matrix, and workspace */
    Long *p ;                   /* colamd's copy of the column pointers */
    Long Alen ;                 /* size of A */
    Long n_col ;                /* number of columns of A */
    Long n_row ;                /* number of rows of A */
    Long nnz ;                  /* number of entries in A */
    Long full ;                 /* TRUE if input matrix full, FALSE if sparse */
    double knobs [COLAMD_KNOBS] ; /* colamd user-controllable parameters */
    double *out_perm ;          /* output permutation vector */
    double *out_stats ;         /* output stats vector */
    double *in_knobs ;          /* input knobs vector */
    Long i ;                    /* loop counter */
    mxArray *Ainput ;           /* input matrix handle */
    Long spumoni ;              /* verbosity variable */
    Long stats [COLAMD_STATS] ; /* stats for colamd */

    colamd_printf = mexPrintf ; /* COLAMD printf routine */

    /* === Check inputs ===================================================== */

    if (nrhs < 1 || nrhs > 2 || nlhs < 0 || nlhs > 2)
    {
	mexErrMsgTxt (
	"colamd: incorrect number of input and/or output arguments") ;
    }

    /* === Get knobs ======================================================== */

    colamd_l_set_defaults (knobs) ;
    spumoni = 0 ;

    /* check for user-passed knobs */
    if (nrhs == 2)
    {
	in_knobs = mxGetPr (prhs [1]) ;
	i = mxGetNumberOfElements (prhs [1]) ;
	if (i > 0) knobs [COLAMD_DENSE_ROW] = in_knobs [0] ;
	if (i > 1) knobs [COLAMD_DENSE_COL] = in_knobs [1] ;
	if (i > 2) spumoni = (Long) (in_knobs [2] != 0) ;
    }

    /* print knob settings if spumoni is set */
    if (spumoni)
    {
	mexPrintf ("\ncolamd version %d.%d, %s:\n",
	    COLAMD_MAIN_VERSION, COLAMD_SUB_VERSION, COLAMD_DATE) ;
	if (knobs [COLAMD_DENSE_ROW] >= 0)
	{
	    mexPrintf ("knobs(1): %g, rows with > max(16,%g*sqrt(size(A,2)))"
		" entries removed\n", in_knobs [0], knobs [COLAMD_DENSE_ROW]) ;
	}
	else
	{
	    mexPrintf ("knobs(1): %g, only completely dense rows removed\n",
		in_knobs [0]) ;
	}
	if (knobs [COLAMD_DENSE_COL] >= 0)
	{
	    mexPrintf ("knobs(2): %g, cols with > max(16,%g*sqrt(min(size(A)))"
		" entries removed\n", in_knobs [1], knobs [COLAMD_DENSE_COL]) ;
	}
	else
	{
	    mexPrintf ("knobs(2): %g, only completely dense columns removed\n",
		in_knobs [1]) ;
	}
	mexPrintf ("knobs(3): %g, statistics and knobs printed\n",
	    in_knobs [2]) ;
    }

    /* === If A is full, convert to a sparse matrix ========================= */

    Ainput = (mxArray *) prhs [0] ;
    if (mxGetNumberOfDimensions (Ainput) != 2)
    {
	mexErrMsgTxt ("colamd: input matrix must be 2-dimensional") ;
    }
    full = !mxIsSparse (Ainput) ;
    if (full)
    {
	mexCallMATLAB (1, &Ainput, 1, (mxArray **) prhs, "sparse") ;
    }

    /* === Allocate workspace for colamd ==================================== */

    /* get size of matrix */
    n_row = mxGetM (Ainput) ;
    n_col = mxGetN (Ainput) ;

    /* get column pointer vector so we can find nnz */
    p = (Long *) mxCalloc (n_col+1, sizeof (Long)) ;
    (void) memcpy (p, mxGetJc (Ainput), (n_col+1)*sizeof (Long)) ;
    nnz = p [n_col] ;
    Alen = (Long) colamd_l_recommended (nnz, n_row, n_col) ;
    if (Alen == 0)
    {
    	mexErrMsgTxt ("colamd: problem too large") ;
    }

    /* === Copy input matrix into workspace ================================= */

    A = (Long *) mxCalloc (Alen, sizeof (Long)) ;
    (void) memcpy (A, mxGetIr (Ainput), nnz*sizeof (Long)) ;

    if (full)
    {
	mxDestroyArray (Ainput) ;
    }

    /* === Order the columns (destroys A) =================================== */

    if (!colamd_l (n_row, n_col, Alen, A, p, knobs, stats))
    {
	colamd_l_report (stats) ;
	mexErrMsgTxt ("colamd error!") ;
    }
    mxFree (A) ;

    /* === Return the permutation vector ==================================== */

    plhs [0] = mxCreateDoubleMatrix (1, n_col, mxREAL) ;
    out_perm = mxGetPr (plhs [0]) ;
    for (i = 0 ; i < n_col ; i++)
    {
	/* colamd is 0-based, but MATLAB expects this to be 1-based */
	out_perm [i] = p [i] + 1 ;
    }
    mxFree (p) ;

    /* === Return the stats vector ========================================== */

    /* print stats if spumoni is set */
    if (spumoni)
    {
	colamd_l_report (stats) ;
    }

    if (nlhs == 2)
    {
	plhs [1] = mxCreateDoubleMatrix (1, COLAMD_STATS, mxREAL) ;
	out_stats = mxGetPr (plhs [1]) ;
	for (i = 0 ; i < COLAMD_STATS ; i++)
	{
	    out_stats [i] = stats [i] ;
	}

	/* fix stats (5) and (6), for 1-based information on jumbled matrix. */
	/* note that this correction doesn't occur if symamd returns FALSE */
	out_stats [COLAMD_INFO1] ++ ; 
	out_stats [COLAMD_INFO2] ++ ; 
    }
}
