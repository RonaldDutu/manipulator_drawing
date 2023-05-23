import ETS3.*
mdl_puma560
load hershey
B = hershey{'B'};
B.stroke
path = [ 0.25*B.stroke; zeros(1,numcols(B.stroke))];
k = find(isnan(path(1,:)));
path(:,k) = path(:,k-1); path(3,k) = 0.2;
traj = mstraj(path(:,2:end)', [0.5 0.5 0.5], [], path(:,1)',0.02, 0.2);
about(traj)
numrows(traj) * 0.02
plot3(traj(:,1), traj(:,2), traj(:,3))
Tp = SE3(0.6, 0, 0) * SE3(traj) * SE3.oa( [0 1 0], [0 0 -1]);
q = p560.ikine6s(Tp);
p560.plot(q)