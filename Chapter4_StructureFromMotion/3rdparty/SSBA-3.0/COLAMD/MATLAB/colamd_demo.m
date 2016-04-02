%COLAMD_DEMO demo for colamd, column approx minimum degree ordering algorithm
%
% Example:
%   colamd_demo
% 
% The following m-files and mexFunctions provide alternative sparse matrix
% ordering methods for MATLAB.  They are typically faster (sometimes much
% faster) and typically provide better orderings than their MATLAB counterparts:
% 
%       colamd          a replacement for colmmd.
%
%                       Typical usage:  p = colamd (A) ;
%
%       symamd          a replacement for symmmd.  Based on colamd.
%
%                       Typical usage:  p = symamd (A) ;
%
% For a description of the methods used, see the colamd.c file.
% http://www.suitesparse.com
%
% See also colamd, symamd

% Minor changes:  in MATLAB 7, symmmd and colmmd are flagged as "obsolete".
% This demo checks if they exist, so it should still work when they are removed.

%    Copyright 1998-2007, Timothy A. Davis, and Stefan Larimore
%    Developed in collaboration with J. Gilbert and E. Ng.

%-------------------------------------------------------------------------------
% Print the introduction, the help info, and compile the mexFunctions
%-------------------------------------------------------------------------------

fprintf (1, '\n-----------------------------------------------------------\n') ;
fprintf (1, 'Colamd2/symamd2 demo.') ;
fprintf (1, '\n-----------------------------------------------------------\n') ;
help colamd_demo ;

fprintf (1, '\n-----------------------------------------------------------\n') ;
fprintf (1, 'Colamd help information:') ;
fprintf (1, '\n-----------------------------------------------------------\n') ;
help colamd2 ;

fprintf (1, '\n-----------------------------------------------------------\n') ;
fprintf (1, 'Symamd help information:') ;
fprintf (1, '\n-----------------------------------------------------------\n') ;
help symamd2 ;

%-------------------------------------------------------------------------------
% Solving Ax=b
%-------------------------------------------------------------------------------

n = 100 ;
fprintf (1, '\n-----------------------------------------------------------\n') ;
fprintf (1, 'Solving Ax=b for a small %d-by-%d random matrix:', n, n) ;
fprintf (1, '\n-----------------------------------------------------------\n') ;
fprintf (1, '\nNote: Random sparse matrices are AWFUL test cases.\n') ;
fprintf (1, 'They''re just easy to generate in a demo.\n') ;

% set up the system

rand ('state', 0) ;
randn ('state', 0) ;
spparms ('default') ;
A = sprandn (n, n, 5/n) + speye (n) ;
b = (1:n)' ;

fprintf (1, '\n\nSolving via lu (PAQ = LU), where Q is from colamd2:\n') ;
q = colamd2 (A) ;
I = speye (n) ;
Q = I (:, q) ;
[L,U,P] = lu (A*Q) ;
fl = luflops (L, U) ;
x = Q * (U \ (L \ (P * b))) ;
fprintf (1, '\nFlop count for [L,U,P] = lu (A*Q):          %d\n', fl) ;
fprintf (1, 'residual:                                     %e\n', norm (A*x-b));

try
    fprintf (1, '\n\nSolving via lu (PAQ = LU), where Q is from colmmd:\n') ;
    q = colmmd (A) ;
    I = speye (n) ;
    Q = I (:, q) ;
    [L,U,P] = lu (A*Q) ;
    fl = luflops (L, U) ;
    x = Q * (U \ (L \ (P * b))) ;
    fprintf (1, '\nFlop count for [L,U,P] = lu (A*Q):          %d\n', fl) ;
    fprintf (1, 'residual:                                     %e\n', ...
	norm (A*x-b)) ;
catch
    fprintf (1, 'colmmd is obsolete; test skipped\n') ;
end

fprintf (1, '\n\nSolving via lu (PA = LU), without regard for sparsity:\n') ;
[L,U,P] = lu (A) ;
fl = luflops (L, U) ;
x = U \ (L \ (P * b)) ;
fprintf (1, '\nFlop count for [L,U,P] = lu (A*Q):          %d\n', fl) ;
fprintf (1, 'residual:                                     %e\n', norm (A*x-b));

%-------------------------------------------------------------------------------
% Large demo for colamd2
%-------------------------------------------------------------------------------

fprintf (1, '\n-----------------------------------------------------------\n') ;
fprintf (1, 'Large demo for colamd2 (symbolic analysis only):') ;
fprintf (1, '\n-----------------------------------------------------------\n') ;

rand ('state', 0) ;
randn ('state', 0) ;
spparms ('default') ;
n = 1000 ;
fprintf (1, 'Generating a random %d-by-%d sparse matrix.\n', n, n) ;
A = sprandn (n, n, 5/n) + speye (n) ;

fprintf (1, '\n\nUnordered matrix:\n') ;
lnz = symbfact (A, 'col') ;
fprintf (1, 'nz in Cholesky factors of A''A:            %d\n', sum (lnz)) ;
fprintf (1, 'flop count for Cholesky of A''A:           %d\n', sum (lnz.^2)) ;

tic ;
p = colamd2 (A) ;
t = toc ;
lnz = symbfact (A (:,p), 'col') ;
fprintf (1, '\n\nColamd run time:                          %f\n', t) ;
fprintf (1, 'colamd2 ordering quality: \n') ;
fprintf (1, 'nz in Cholesky factors of A(:,p)''A(:,p):  %d\n', sum (lnz)) ;
fprintf (1, 'flop count for Cholesky of A(:,p)''A(:,p): %d\n', sum (lnz.^2)) ;

try
    tic ;
    p = colmmd (A) ;
    t = toc ;
    lnz = symbfact (A (:,p), 'col') ;
    fprintf (1, '\n\nColmmd run time:                          %f\n', t) ;
    fprintf (1, 'colmmd ordering quality: \n') ;
    fprintf (1, 'nz in Cholesky factors of A(:,p)''A(:,p):  %d\n', sum (lnz)) ;
    fprintf (1, 'flop count for Cholesky of A(:,p)''A(:,p): %d\n', ...
	sum (lnz.^2)) ;
catch
    fprintf (1, 'colmmd is obsolete; test skipped\n') ;
end

%-------------------------------------------------------------------------------
% Large demo for symamd2
%-------------------------------------------------------------------------------

fprintf (1, '\n-----------------------------------------------------------\n') ;
fprintf (1, 'Large demo for symamd2 (symbolic analysis only):') ;
fprintf (1, '\n-----------------------------------------------------------\n') ;

fprintf (1, 'Generating a random symmetric %d-by-%d sparse matrix.\n', n, n) ;
A = A+A' ;

fprintf (1, '\n\nUnordered matrix:\n') ;
lnz = symbfact (A, 'sym') ;
fprintf (1, 'nz in Cholesky factors of A:       %d\n', sum (lnz)) ;
fprintf (1, 'flop count for Cholesky of A:      %d\n', sum (lnz.^2)) ;

tic ;
p = symamd2 (A) ;
t = toc ;
lnz = symbfact (A (p,p), 'sym') ;
fprintf (1, '\n\nSymamd run time:                   %f\n', t) ;
fprintf (1, 'symamd2 ordering quality: \n') ;
fprintf (1, 'nz in Cholesky factors of A(p,p):  %d\n', sum (lnz)) ;
fprintf (1, 'flop count for Cholesky of A(p,p): %d\n', sum (lnz.^2)) ;

try
    tic ;
    p = symmmd (A) ;
    t = toc ;
    lnz = symbfact (A (p,p), 'sym') ;
    fprintf (1, '\n\nSymmmd run time:                   %f\n', t) ;
    fprintf (1, 'symmmd ordering quality: \n') ;
    fprintf (1, 'nz in Cholesky factors of A(p,p):  %d\n', sum (lnz)) ;
    fprintf (1, 'flop count for Cholesky of A(p,p): %d\n', sum (lnz.^2)) ;
catch
    fprintf (1, 'symmmd is obsolete\n') ;
end
