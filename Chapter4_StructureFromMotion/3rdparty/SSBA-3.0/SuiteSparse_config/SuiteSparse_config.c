/* ========================================================================== */
/* === SuiteSparse_config =================================================== */
/* ========================================================================== */

/* Copyright (c) 2012, Timothy A. Davis.  No licensing restrictions
 * apply to this file or to the SuiteSparse_config directory.
 * Author: Timothy A. Davis.
 */

#include "SuiteSparse_config.h"

/* -------------------------------------------------------------------------- */
/* SuiteSparse_malloc: malloc wrapper */
/* -------------------------------------------------------------------------- */

void *SuiteSparse_malloc    /* pointer to allocated block of memory */
(
    size_t nitems,          /* number of items to malloc (>=1 is enforced) */
    size_t size_of_item,    /* sizeof each item */
    int *ok,                /* TRUE if successful, FALSE otherwise */
    SuiteSparse_config *config      /* SuiteSparse-wide configuration */
)
{
    void *p ;
    if (nitems < 1) nitems = 1 ;
    if (nitems * size_of_item != ((double) nitems) * size_of_item)
    {
        /* Int overflow */
        *ok = 0 ;
        return (NULL) ;
    }
    if (!config || config->malloc_memory == NULL)
    {
        /* use malloc by default */
        p = (void *) malloc (nitems * size_of_item) ;
    }
    else
    {
        /* use the pointer to malloc in the config */
        p = (void *) (config->malloc_memory) (nitems * size_of_item) ;
    }
    *ok = (p != NULL) ;
    return (p) ;
}


/* -------------------------------------------------------------------------- */
/* SuiteSparse_free: free wrapper */
/* -------------------------------------------------------------------------- */

void *SuiteSparse_free      /* always returns NULL */
(
    void *p,                /* block to free */
    SuiteSparse_config *config        /* SuiteSparse-wide configuration */
)
{
    if (p)
    {
        if (!config || config->free_memory == NULL)
        {
            /* use free by default */
            free (p) ;
        }
        else
        {
            /* use the pointer to free in the config */
            (config->free_memory) (p) ;
        }
    }
    return (NULL) ;
}


/* -------------------------------------------------------------------------- */
/* SuiteSparse_tic: return current wall clock time */
/* -------------------------------------------------------------------------- */

/* Returns the number of seconds (tic [0]) and nanoseconds (tic [1]) since some
 * unspecified but fixed time in the past.  If no timer is installed, zero is
 * returned.  A scalar double precision value for 'tic' could be used, but this
 * might cause loss of precision because clock_getttime returns the time from
 * some distant time in the past.  Thus, an array of size 2 is used.
 *
 * The timer is enabled by default.  To disable the timer, compile with
 * -DNTIMER.  If enabled on a POSIX C 1993 system, the timer requires linking
 * with the -lrt library.
 *
 * example:
 *
 *      double tic [2], r, s, t ;
 *      SuiteSparse_tic (tic) ;     // start the timer
 *      // do some work A
 *      t = SuiteSparse_toc (tic) ; // t is time for work A, in seconds
 *      // do some work B
 *      s = SuiteSparse_toc (tic) ; // s is time for work A and B, in seconds
 *      SuiteSparse_tic (tic) ;     // restart the timer
 *      // do some work C
 *      r = SuiteSparse_toc (tic) ; // s is time for work C, in seconds
 *
 * A double array of size 2 is used so that this routine can be more easily
 * ported to non-POSIX systems.  The caller does not rely on the POSIX
 * <time.h> include file.
 */

#ifdef SUITESPARSE_TIMER_ENABLED

#include <time.h>

void SuiteSparse_tic
(
    double tic [2]      /* output, contents undefined on input */
)
{
    /* POSIX C 1993 timer, requires -librt */
    struct timespec t ;
    clock_gettime (CLOCK_MONOTONIC, &t) ;
    tic [0] = (double) (t.tv_sec) ;
    tic [1] = (double) (t.tv_nsec) ;
}

#else

void SuiteSparse_tic
(
    double tic [2]      /* output, contents undefined on input */
)
{
    /* no timer installed */
    tic [0] = 0 ;
    tic [1] = 0 ;
}

#endif


/* -------------------------------------------------------------------------- */
/* SuiteSparse_toc: return time since last tic */
/* -------------------------------------------------------------------------- */

/* Assuming SuiteSparse_tic is accurate to the nanosecond, this function is
 * accurate down to the nanosecond for 2^53 nanoseconds since the last call to
 * SuiteSparse_tic, which is sufficient for SuiteSparse (about 104 days).  If
 * additional accuracy is required, the caller can use two calls to
 * SuiteSparse_tic and do the calculations differently.
 */

double SuiteSparse_toc  /* returns time in seconds since last tic */
(
    double tic [2]  /* input, not modified from last call to SuiteSparse_tic */
)
{
    double toc [2] ;
    SuiteSparse_tic (toc) ;
    return ((toc [0] - tic [0]) + 1e-9 * (toc [1] - tic [1])) ;
}


/* -------------------------------------------------------------------------- */
/* SuiteSparse_time: return current wallclock time in seconds */
/* -------------------------------------------------------------------------- */

/* This function might not be accurate down to the nanosecond. */

double SuiteSparse_time  /* returns current wall clock time in seconds */
(
    void
)
{
    double toc [2] ;
    SuiteSparse_tic (toc) ;
    return (toc [0] + 1e-9 * toc [1]) ;
}

