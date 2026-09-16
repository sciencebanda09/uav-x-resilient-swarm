function validate_log(path)
%VALIDATE_LOG Validate required fields for uav-x-log/v1 before visualization.
records = load_log(path);
assert(~isempty(records), 'Log is empty');
assert(strcmp(records{1}.schema_version, 'uav-x-log/v1'), 'Unsupported schema');
for i = 1:numel(records)
    assert(isfield(records{i}, 'record_type'), 'record_type missing');
    if strcmp(records{i}.record_type, 'tick')
        assert(isfield(records{i}, 'uavs') && isfield(records{i}, 'pois'), 'tick state missing');
        assert(isfield(records{i}, 'links') && isfield(records{i}, 'packets'), 'tick network state missing');
    end
end
disp('UAV-X log schema valid');
end
