function rmserr = rmse (estimate, true)
% rmse: rooted mean squarred error between two vectors.
%
% INPUT
%   estimate, Nx1 estimate values.
%   true,     Nx1 true values, reference.
%
% OUTPUT
%   rmserr, rooted mean squarred error.


if ( length (estimate) ~= length (true) )   
    error('rmse: vectors must have the same length.')
end

if( any(isnan(true)) )
    error('rmse: true vector with at least one NaN value.');
end

if( any(isnan(estimate)) )
    error('rmse: estimate vector with at least one NaN value.');
end

rmserr =  sqrt ( mean ( ( estimate - true ).^2 ) ) ;

end
