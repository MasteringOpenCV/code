// -*- C++ -*-
#ifndef V3D_LDL_PRIVATE_H
#define V3D_LDL_PRIVATE_H

namespace
{

   using namespace V3D;

   inline double
   squaredResidual(VectorArray<double> const& e)
   {
      int const N = e.count();
      int const M = e.size();

      double res = 0.0;

      for (int n = 0; n < N; ++n)
         for (int m = 0; m < M; ++m)
            res += e[n][m] * e[n][m];

      return res;
   } // end squaredResidual()

// Since we permute the columns/rows on our own, we can simplify (and
// accelerate) LDL slightly by removing the permutation paramater.
/* LDL Version 1.3, Copyright (c) 2006 by Timothy A Davis,
 * University of Florida.  All Rights Reserved.  Developed while on sabbatical
 * at Stanford University and Lawrence Berkeley National Laboratory.  Refer to
 * the README file for the License.  Available at
 * http://www.cise.ufl.edu/research/sparse.
 */

   typedef int LDL_int;

   inline void
   LDL_symbolic
   (
      LDL_int n,		/* A and L are n-by-n, where n >= 0 */
      LDL_int Ap [ ],	/* input of size n+1, not modified */
      LDL_int Ai [ ],	/* input of size nz=Ap[n], not modified */
      LDL_int Lp [ ],	/* output of size n+1, not defined on input */
      LDL_int Parent [ ],	/* output of size n, not defined on input */
      LDL_int Lnz [ ],	/* output of size n, not defined on input */
      LDL_int Flag [ ]	/* workspace of size n, not defn. on input or output */
      )
   {
      LDL_int i, k, p, kk, p2 ;
      for (k = 0 ; k < n ; k++)
      {
         /* L(k,:) pattern: all nodes reachable in etree from nz in A(0:k-1,k) */
         Parent [k] = -1 ;	    /* parent of k is not yet known */
         Flag [k] = k ;		    /* mark node k as visited */
         Lnz [k] = 0 ;		    /* count of nonzeros in column k of L */
         kk = k ;  /* kth original, or permuted, column */
         p2 = Ap [kk+1] ;
         for (p = Ap [kk] ; p < p2 ; p++)
         {
	    /* A (i,k) is nonzero (original or permuted A) */
	    i = Ai [p] ;
	    if (i < k)
	    {
               /* follow path from i to root of etree, stop at flagged node */
               for ( ; Flag [i] != k ; i = Parent [i])
               {
                  /* find parent of i if not yet determined */
                  if (Parent [i] == -1) Parent [i] = k ;
                  Lnz [i]++ ;				/* L (k,i) is nonzero */
                  Flag [i] = k ;			/* mark i as visited */
               }
	    }
         }
      }
      /* construct Lp index array from Lnz column counts */
      Lp [0] = 0 ;
      for (k = 0 ; k < n ; k++)
      {
         Lp [k+1] = Lp [k] + Lnz [k] ;
      }
   }


/* ========================================================================== */
/* === ldl_numeric ========================================================== */
/* ========================================================================== */

/* Given a sparse matrix A (the arguments n, Ap, Ai, and Ax) and its symbolic
 * analysis (Lp and Parent, and optionally P and Pinv), compute the numeric LDL'
 * factorization of A or PAP'.  The outputs of this routine are arguments Li,
 * Lx, and D.  It also requires three size-n workspaces (Y, Pattern, and Flag).
 */

   inline LDL_int
   LDL_numeric	/* returns n if successful, k if D (k,k) is zero */
   (
      LDL_int n,		/* A and L are n-by-n, where n >= 0 */
      LDL_int Ap [ ],	/* input of size n+1, not modified */
      LDL_int Ai [ ],	/* input of size nz=Ap[n], not modified */
      double Ax [ ],	/* input of size nz=Ap[n], not modified */
      LDL_int Lp [ ],	/* input of size n+1, not modified */
      LDL_int Parent [ ],	/* input of size n, not modified */
      LDL_int Lnz [ ],	/* output of size n, not defn. on input */
      LDL_int Li [ ],	/* output of size lnz=Lp[n], not defined on input */
      double Lx [ ],	/* output of size lnz=Lp[n], not defined on input */
      double D [ ],	/* output of size n, not defined on input */
      double Y [ ],	/* workspace of size n, not defn. on input or output */
      LDL_int Pattern [ ],/* workspace of size n, not defn. on input or output */
      LDL_int Flag [ ]	/* workspace of size n, not defn. on input or output */
      )
   {
      double yi, l_ki ;
      LDL_int i, k, p, kk, p2, len, top ;
      for (k = 0 ; k < n ; k++)
      {
         /* compute nonzero Pattern of kth row of L, in topological order */
         Y [k] = 0.0 ;		    /* Y(0:k) is now all zero */
         top = n ;		    /* stack for pattern is empty */
         Flag [k] = k ;		    /* mark node k as visited */
         Lnz [k] = 0 ;		    /* count of nonzeros in column k of L */
         kk = k ;  /* kth original, or permuted, column */
         p2 = Ap [kk+1] ;
         for (p = Ap [kk] ; p < p2 ; p++)
         {
	    i = Ai [p] ;	/* get A(i,k) */
	    if (i <= k)
	    {
               Y [i] += Ax [p] ;  /* scatter A(i,k) into Y (sum duplicates) */
               for (len = 0 ; Flag [i] != k ; i = Parent [i])
               {
                  Pattern [len++] = i ;   /* L(k,i) is nonzero */
                  Flag [i] = k ;	    /* mark i as visited */
               }
               while (len > 0) Pattern [--top] = Pattern [--len] ;
	    }
         }
         /* compute numerical values kth row of L (a sparse triangular solve) */
         D [k] = Y [k] ;		    /* get D(k,k) and clear Y(k) */
         Y [k] = 0.0 ;
         for ( ; top < n ; top++)
         {
	    i = Pattern [top] ;	    /* Pattern [top:n-1] is pattern of L(:,k) */
	    yi = Y [i] ;	    /* get and clear Y(i) */
	    Y [i] = 0.0 ;
	    p2 = Lp [i] + Lnz [i] ;
	    for (p = Lp [i] ; p < p2 ; p++)
	    {
               Y [Li [p]] -= Lx [p] * yi ;
	    }
	    l_ki = yi / D [i] ;	    /* the nonzero entry L(k,i) */
	    D [k] -= l_ki * yi ;
	    Li [p] = k ;	    /* store L(k,i) in column form of L */
	    Lx [p] = l_ki ;
	    Lnz [i]++ ;		    /* increment count of nonzeros in col i */
         }
         if (D [k] == 0.0) return (k) ;	    /* failure, D(k,k) is zero */
      }
      return (n) ;	/* success, diagonal of D is all nonzero */
   }


/* ========================================================================== */
/* === ldl_lsolve:  solve Lx=b ============================================== */
/* ========================================================================== */

   inline void
   LDL_lsolve
   (
      LDL_int n,		/* L is n-by-n, where n >= 0 */
      double X [ ],	/* size n.  right-hand-side on input, soln. on output */
      LDL_int Lp [ ],	/* input of size n+1, not modified */
      LDL_int Li [ ],	/* input of size lnz=Lp[n], not modified */
      double Lx [ ]	/* input of size lnz=Lp[n], not modified */
      )
   {
      LDL_int j, p, p2 ;
      for (j = 0 ; j < n ; j++)
      {
         p2 = Lp [j+1] ;
         for (p = Lp [j] ; p < p2 ; p++)
         {
	    X [Li [p]] -= Lx [p] * X [j] ;
         }
      }
   }


/* ========================================================================== */
/* === ldl_dsolve:  solve Dx=b ============================================== */
/* ========================================================================== */

   inline void
   LDL_dsolve
   (
      LDL_int n,		/* D is n-by-n, where n >= 0 */
      double X [ ],	/* size n.  right-hand-side on input, soln. on output */
      double D [ ]	/* input of size n, not modified */
      )
   {
      LDL_int j ;
      for (j = 0 ; j < n ; j++)
      {
         X [j] /= D [j] ;
      }
   }


/* ========================================================================== */
/* === ldl_ltsolve: solve L'x=b  ============================================ */
/* ========================================================================== */

   inline void
   LDL_ltsolve
   (
      LDL_int n,		/* L is n-by-n, where n >= 0 */
      double X [ ],	/* size n.  right-hand-side on input, soln. on output */
      LDL_int Lp [ ],	/* input of size n+1, not modified */
      LDL_int Li [ ],	/* input of size lnz=Lp[n], not modified */
      double Lx [ ]	/* input of size lnz=Lp[n], not modified */
      )
   {
      int j, p, p2 ;
      for (j = n-1 ; j >= 0 ; j--)
      {
         p2 = Lp [j+1] ;
         for (p = Lp [j] ; p < p2 ; p++)
         {
	    X [j] -= Lx [p] * X [Li [p]] ;
         }
      }
   }


/* ========================================================================== */
/* === ldl_perm: permute a vector, x=Pb ===================================== */
/* ========================================================================== */

   inline void
   LDL_perm
   (
      LDL_int n,		/* size of X, B, and P */
      double X [ ],	/* output of size n. */
      double B [ ],	/* input of size n. */
      LDL_int P [ ]	/* input permutation array of size n. */
      )
   {
      LDL_int j ;
      for (j = 0 ; j < n ; j++)
      {
         X [j] = B [P [j]] ;
      }
   }


/* ========================================================================== */
/* === ldl_permt: permute a vector, x=P'b =================================== */
/* ========================================================================== */

   inline void
   LDL_permt
   (
      LDL_int n,		/* size of X, B, and P */
      double X [ ],	/* output of size n. */
      double B [ ],	/* input of size n. */
      LDL_int P [ ]	/* input permutation array of size n. */
      )
   {
      LDL_int j ;
      for (j = 0 ; j < n ; j++)
      {
         X [P [j]] = B [j] ;
      }
   }

} // end namespace <>

#endif
