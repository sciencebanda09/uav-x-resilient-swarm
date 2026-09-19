function replay_3d(path, output)
%REPLAY_3D Compatibility wrapper for the MATLAB-native UAV-X 3D scene.
if nargin < 2
    uav_x_3d_replay(path);
else
    uav_x_3d_replay(path, output);
end
end
