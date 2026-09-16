function plot_metrics(path)
%PLOT_METRICS Plot basic metrics from the final summary record.
records = load_log(path);
summary = records{find(cellfun(@(r) strcmp(r.record_type, 'summary'), records), 1, 'last')};
names = {'Mission completion', 'Connectivity availability'};
values = [summary.mission_completion_rate, summary.connectivity_availability];
bar(values); ylim([0 1]); set(gca, 'XTickLabel', names); ylabel('Fraction'); grid on;
title('UAV-X challenge metrics');
end
