function padded_ylim(ax, values)
%PADDED_YLIM  Set the y-limits to the range of VALUES plus 5% padding (as in the original
%   plot_results_* scripts); a flat signal gets +-1 around its value.
lo = min(values(:)); hi = max(values(:));
if lo == hi, lo = lo - 1; hi = hi + 1; end
pad = (hi - lo)*0.05;
ylim(ax, [lo - pad, hi + pad]);
end
