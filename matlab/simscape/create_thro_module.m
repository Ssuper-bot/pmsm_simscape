function model_path = create_thro_module(model_name)
%CREATE_THRO_MODULE Build the standalone throttle-to-iq feedforward module.

if nargin < 1 || isempty(model_name)
    model_name = 'pmsm_module_thro';
end

model_path = pmsm_foc_builder('create_module_model', 'thro', model_name);
end