function model_path = create_measure_module(model_name)
%CREATE_MEASURE_MODULE Build the standalone measurement module model.

if nargin < 1 || isempty(model_name)
    model_name = 'pmsm_module_measure';
end

model_path = pmsm_foc_builder('create_module_model', 'measure', model_name);
end