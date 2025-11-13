function [w] = custom_neg_sqrt(ww)

w = zeros(length(ww),1);
for i = 1:length(ww)
    if ww(i) < 0
        % For negative x, return the negative square root of its absolute value.
        w(i) = -sqrt(abs(ww(i)));
    else
        % For non-negative x, return the standard square root.
        w(i) = sqrt(ww(i));
    end
end

end