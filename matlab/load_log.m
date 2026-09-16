function records = load_log(path)
%LOAD_LOG Read UAV-X JSONL records. MATLAB is optional for UAV-X execution.
lines = readlines(path);
records = cell(0, 1);
for i = 1:numel(lines)
    line = strtrim(lines(i));
    if strlength(line) > 0
        records{end+1, 1} = jsondecode(char(line)); %#ok<AGROW>
    end
end
end
