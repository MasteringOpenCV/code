function colamd_install
%COLAMD_MAKE to compile and install the colamd2 and symamd2 mexFunction.
%   Your current directory must be COLAMD/MATLAB for this function to work.
%
% Example:
%   colamd_install
%
% See also colamd2, symamd2.

%    Copyright 1998-2007, Timothy A. Davis, and Stefan Larimore
%    Developed in collaboration with J. Gilbert and E. Ng.

colamd_make
addpath (pwd)
fprintf ('\nThe following path has been added.  You may wish to add it\n') ;
fprintf ('permanently, using the MATLAB pathtool command.\n') ;
fprintf ('%s\n\n', pwd) ;
colamd_demo
