waypoints = [0    0   0;
             1    1   1;
             2    0   2;
             3    -1  1;
             4    0   0]'
          ;
[n_coords, n] = size(waypoints);

T_max = 8;
t = max(0, min(t, T_max));
T_i = T_max/(n-1);

out = waypoints(:,1:end-1);

d = waypoints(:,2:end) - waypoints(:,1:end-1);
d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
traj_time = [0, cumsum(d0)];

n = n - 1;
A = zeros(8*n,8*n);
b = zeros(8*n, 1);

Cx Cy Cz;
