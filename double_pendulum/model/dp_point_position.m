function ppe = dp_point_position(q, s, p)
%DP_POINT_POSITION  Cartesian position of a point along the double pendulum.
%
%   q : joint angles (2-vector)   s : normalized arc length in [0,1] (0 = base, 1 = tip)
%   ppe = [x, y, orientation]. Frame: y points up, gravity along -y; q = 0 is the hanging
%   configuration (angles are measured from the downward vertical, hence the -pi/2 below).

q(1) = q(1) - pi/2;
effectiveLength = s*(p.l1 + p.l2);
if s <= p.l1/(p.l1 + p.l2)          % point on the first link
    x = effectiveLength*cos(q(1));
    y = effectiveLength*sin(q(1));
    orientation = q(1);
else                                % point on the second link
    x = p.l1*cos(q(1)) + (effectiveLength - p.l1)*cos(q(1) + q(2));
    y = p.l1*sin(q(1)) + (effectiveLength - p.l1)*sin(q(1) + q(2));
    orientation = q(1) + q(2);
end
ppe = [x, y, orientation];
end
