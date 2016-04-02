function [p, stats] = symamd2 (S, knobs)
%SYMAMD Symmetric approximate minimum degree permutation.
%    P = SYMAMD2(S) for a symmetric positive definite matrix S, returns the
%    permutation vector p such that S(p,p) tends to have a sparser Cholesky
%    factor than S.  Sometimes SYMAMD works well for symmetric indefinite
%    matrices too.  The matrix S is assumed to be symmetric; only the
%    strictly lower triangular part is referenced.   S must be square.
%    Note that p = amd(S) is much faster and generates comparable orderings.
%    The ordering is followed by an elimination tree post-ordering.
%
%    Note that this function is source code for the built-in MATLAB symamd
%    function.  It has been renamed here to symamd2 to avoid a filename clash.
%    symamd and symamd2 are identical.
%
%    See also SYMAMD, AMD, COLAMD, COLAMD2.
%
%    Example:
%            P = symamd2 (S)
%            [P, stats] = symamd2 (S, knobs)
%
%    knobs is an optional one- to two-element input vector.  If S is n-by-n,
%    then rows and columns with more than max(16,knobs(1)*sqrt(n)) entries are
%    removed prior to ordering, and ordered last in the output permutation P.
%    No rows/columns are removed if knobs(1)<0.  If knobs(2) is nonzero, stats
%    and knobs are printed.  The default is knobs = [10 0].  Note that knobs
%    differs from earlier versions of symamd.

%    Copyright 1998-2007, Timothy A. Davis, and Stefan Larimore
%    Developed in collaboration with J. Gilbert and E. Ng.
%    Acknowledgements: This work was supported by the National Science
%       Foundation, under grants DMS-9504974 and DMS-9803599.

%-------------------------------------------------------------------------------
% perform the symamd ordering:
%-------------------------------------------------------------------------------

if (nargout <= 1 & nargin == 1)						    %#ok
    p = symamd2mex (S) ;
elseif (nargout <= 1 & nargin == 2)					    %#ok
    p = symamd2mex (S, knobs) ;
elseif (nargout == 2 & nargin == 1)					    %#ok
    [p, stats] = symamd2mex (S) ;
elseif (nargout == 2 & nargin == 2)					    %#ok
    [p, stats] = symamd2mex (S, knobs) ;
else
    error('symamd:  incorrect number of input and/or output arguments.') ;
end

%-------------------------------------------------------------------------------
% symmetric elimination tree post-ordering:
%-------------------------------------------------------------------------------

[ignore, q] = etree (S (p,p)) ;
p = p (q) ;


%    stats is an optional 20-element output vector that provides data about the
%    ordering and the validity of the input matrix S.  Ordering statistics are
%    in stats (1:3).  stats (1) = stats (2) is the number of dense or empty
%    rows and columns ignored by SYMAMD and stats (3) is the number of
%    garbage collections performed on the internal data structure used by
%    SYMAMD (roughly of size 8.4*nnz(tril(S,-1)) + 9*n integers).
%
%    MATLAB built-in functions are intended to generate valid sparse matrices,
%    with no duplicate entries, with ascending row indices of the nonzeros
%    in each column, with a non-negative number of entries in each column (!)
%    and so on.  If a matrix is invalid, then SYMAMD may or may not be able
%    to continue.  If there are duplicate entries (a row index appears two or
%    more times in the same column) or if the row indices in a column are out
%    of order, then SYMAMD can correct these errors by ignoring the duplicate
%    entries and sorting each column of its internal copy of the matrix S (the
%    input matrix S is not repaired, however).  If a matrix is invalid in other
%    ways then SYMAMD cannot continue, an error message is printed, and no
%    output arguments (P or stats) are returned.  SYMAMD is thus a simple way
%    to check a sparse matrix to see if it's valid.
%
%    stats (4:7) provide information if SYMAMD was able to continue.  The
%    matrix is OK if stats (4) is zero, or 1 if invalid.  stats (5) is the
%    rightmost column index that is unsorted or contains duplicate entries,
%    or zero if no such column exists.  stats (6) is the last seen duplicate
%    or out-of-order row index in the column index given by stats (5), or zero
%    if no such row index exists.  stats (7) is the number of duplicate or
%    out-of-order row indices.
%
%    stats (8:20) is always zero in the current version of SYMAMD (reserved
%    for future use).
