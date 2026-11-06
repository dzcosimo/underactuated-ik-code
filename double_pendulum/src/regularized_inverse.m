function Xinv = regularized_inverse(X)
%REGULARIZED_INVERSE  Inverse with the small diagonal loading used near singularities
%   (Sec. 5.3: "a small diagonal loading of the matrix that becomes singular").
%   If |det(X)| < 1e-3 the inverse of X + 1e-3 I is returned, otherwise inv(X).
if abs(det(X)) < 1e-3
    Xinv = inv(X + 1e-3*eye(size(X)));
else
    Xinv = inv(X);
end
end
