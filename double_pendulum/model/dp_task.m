function x = dp_task(q, p)
%DP_TASK  Task h(q) of Eq. (7): horizontal position of the end effector,
%   x = l1 sin(q1) + l2 sin(q1 + q2) (angles from the downward vertical).
%   q: 2-vector of joint angles.   See also DP_POINT_POSITION.
ppe = dp_point_position(q, 1, p);
x = ppe(1);
end
