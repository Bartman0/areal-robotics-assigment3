function [elements]=polynom(length,derivative,value)
%returns a vector with the coefficients of the derivatives of a n grade
%polynomial. n= number of polonomials, t= time we want to compute it at.

    coefficients = ones(1, length);
    exponents = zeros(1, length);
    for i = 1:length
        exponents(1,i) = i-1;
    end
    for d = 1:derivative
        for i = 1:length
            coefficients(1,i) = coefficients(1,i) * exponents(1,i);
            exponents(1,i) = max(0, exponents(1,i) - 1);
        end
    end
    elements = coefficients .* (value .^ exponents);

end