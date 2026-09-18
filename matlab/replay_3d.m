function replay_3d(path, output)
%REPLAY_3D Optional MATLAB replay for uav-x-log/v1.
records = load_log(path);
ticks = records(cellfun(@(r) strcmp(r.record_type, 'tick'), records));
figure; grid on; axis equal; view(3); hold on;
heightmap_path = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'viewer', 'public', 'scene', 'terrain_heightmap.json');
if isfile(heightmap_path)
    terrain = jsondecode(fileread(heightmap_path));
    stride = max(1, floor(terrain.grid / 28));
    z = double(terrain.elevation_m(1:stride:end, 1:stride:end));
    axis_m = linspace(0, terrain.arena_m, size(z, 1));
    [X, Y] = meshgrid(axis_m, axis_m);
    surf(X, Y, z, 'EdgeColor', 'none', 'FaceAlpha', .5);
end
if nargin >= 2, gif_path = output; else, gif_path = ''; end
for k = 1:numel(ticks)
    cla; if exist('X','var'), hold on; surf(X, Y, z, 'EdgeColor', 'none', 'FaceAlpha', .5); end; tick = ticks{k};
    for i = 1:numel(tick.uavs)
        p = tick.uavs(i).position_m;
        scatter3(p(1), p(2), p(3), 80, 'filled');
        text(p(1), p(2), p(3), [' ' char(tick.uavs(i).id)]);
    end
    for i = 1:numel(tick.pois)
        p = tick.pois(i).position_m; scatter3(p(1), p(2), p(3) + 2, 100, 'y', '*', 'filled');
    end
    title(sprintf('UAV-X replay t = %.1f s | mapped %.1f%%', tick.time_s, get_coverage(tick)));
    xlabel('East (m)'); ylabel('North (m)'); zlabel('Altitude (m)'); drawnow;
    if strlength(string(gif_path)) > 0
        frame = getframe(gcf); [im, map] = rgb2ind(frame2im(frame), 256);
        if k == 1, imwrite(im, map, gif_path, 'gif', 'LoopCount', inf, 'DelayTime', .1);
        else, imwrite(im, map, gif_path, 'gif', 'WriteMode', 'append', 'DelayTime', .1); end
    end
end
end

function value = get_coverage(tick)
if isfield(tick, 'survey_coverage_fraction'), value = 100 * tick.survey_coverage_fraction; else, value = 0; end
end
