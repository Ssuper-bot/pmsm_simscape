function results = validate_pmsm_foc_modules(run_all_in)
%VALIDATE_PMSM_FOC_MODULES Build and validate each module plus the all-in model.

if nargin < 1
    run_all_in = true;
end

module_specs = {
    'signal_in', 'pmsm_module_signal_in';
    'foc_controller', 'pmsm_module_foc_controller';
    'three_invertor', 'pmsm_module_three_invertor';
    'motor', 'pmsm_module_motor';
    'measure', 'pmsm_module_measure';
    'scope', 'pmsm_module_scope';
};

[~, ~, ~, sim_params, ~] = pmsm_foc_builder('default_parameters');
results = repmat(struct('model_name', '', 'saved', false, 'compiled', false, 'simulated', false, 'message', ''), 0, 1);

for module_index = 1:size(module_specs, 1)
    module_name = module_specs{module_index, 1};
    model_name = module_specs{module_index, 2};
    fprintf('Building module: %s\n', module_name);
    pmsm_foc_builder('create_module_model', module_name, model_name);
    results(end + 1, 1) = pmsm_foc_builder('validate_model', model_name, sim_params.validation_t_end); %#ok<AGROW>
end

if run_all_in
    all_in_name = 'pmsm_foc_model';
    fprintf('Building all-in model: %s\n', all_in_name);
    create_pmsm_foc_all_in_model(all_in_name);
    results(end + 1, 1) = pmsm_foc_builder('validate_model', all_in_name, sim_params.validation_t_end); %#ok<AGROW>
end

fprintf('\nValidation summary:\n');
for result_index = 1:numel(results)
    status = 'FAIL';
    if results(result_index).saved && results(result_index).compiled && results(result_index).simulated
        status = 'PASS';
    end
    fprintf('  [%s] %s - %s\n', status, results(result_index).model_name, results(result_index).message);
end
end