function operations_2d(path)
%OPERATIONS_2D Optional MATLAB operations view for uav-x-log/v1.
records = load_log(path);
tick = records{find(cellfun(@(r) strcmp(r.record_type, 'tick'), records), 1, 'last')};
clf; hold on; grid on; axis equal;
for i = 1:numel(tick.uavs)
    u = tick.uavs(i); p = u.position_m;
    scatter(p(1), p(2), 70, 'filled');
    text(p(1), p(2), [' ' char(u.id) ' ' char(u.role)]);
end
for i = 1:numel(tick.pois)
    p = tick.pois(i); xy = p.position_m;
    if strcmp(p.status, 'SURVEYED'), c = [0 0.6 0];
    elseif p.priority == 1, c = [0.85 0.1 0.1];
    else, c = [0.95 0.55 0.05]; end
    scatter(xy(1), xy(2), 110, c, '*');
end
title('UAV-X resilient swarm operations'); xlabel('East (m)'); ylabel('North (m)');
end
