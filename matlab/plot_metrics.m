function plot_metrics(path, output)
%PLOT_METRICS Plot basic metrics from the final summary record.
records = load_log(path);
summary = records{find(cellfun(@(r) strcmp(r.record_type, 'summary'), records), 1, 'last')};
names = {'Mission completion', 'Connectivity availability'};
values = [summary.mission_completion_rate, summary.connectivity_availability];
bar(1:numel(values), values, 0.55); ylim([0 1]); xlim([0.5 numel(values)+.5]);
set(gca, 'XTick', 1:numel(values), 'XTickLabel', names); ylabel('Fraction'); grid on;
xtickangle(15);
title('UAV-X challenge metrics');
if nargin >= 2 && strlength(string(output)) > 0
    exportgraphics(gcf, output, 'Resolution', 150);
end
end
