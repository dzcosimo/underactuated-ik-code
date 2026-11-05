function v = bal_sat(v, vmax, mode)
%BAL_SAT  Velocity saturation of the CLIK update.
%
%   v = bal_sat(v, vmax, mode)
%     v     CLIK update (rad/s), any length
%     vmax  bound (default 1 rad/s)
%     mode  'clip'  (default, used for all results): component-wise clipping to [-vmax, vmax]
%           'scale' : the whole vector is scaled so that max|v_i| <= vmax (direction preserving)
%   With the released scenarios the two modes give practically the same results (README).

if nargin < 2, vmax = 1; end
if nargin < 3, mode = 'clip'; end
if strcmp(mode, 'clip'), v = min(max(v, -vmax), vmax);
else, v = v/max(1, max(abs(v))/vmax); end
end
