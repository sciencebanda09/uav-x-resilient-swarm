function replay_3d(path)
%REPLAY_3D Optional MATLAB replay for uav-x-log/v1.
records = load_log(path);
ticks = records(cellfun(@(r) strcmp(r.record_type, 'tick'), records));
figure; grid on; axis equal; view(3); hold on;
for k = 1:numel(ticks)
    cla; tick = ticks{k};
    for i = 1:numel(tick.uavs)
        p = tick.uavs(i).position_m;
        scatter3(p(1), p(2), p(3), 80, 'filled');
        text(p(1), p(2), p(3), [' ' char(tick.uavs(i).id)]);
    end
    for i = 1:numel(tick.pois)
        p = tick.pois(i).position_m; scatter3(p(1), p(2), p(3), 70, 'r', '*');
    end
    title(sprintf('UAV-X replay t = %.1f s', tick.time_s));
    xlabel('East (m)'); ylabel('North (m)'); zlabel('Altitude (m)'); drawnow;
end
end
