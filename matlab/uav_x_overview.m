function uav_x_overview(logPath, outputPath)
%UAV_X_OVERVIEW Export the first frame of the MATLAB 3D mission scene.
if nargin < 2, outputPath = 'artifacts/matlab_uavx_overview.png'; end
uav_x_3d_replay(logPath, outputPath);
end
