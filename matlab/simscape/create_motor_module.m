function model_path = create_motor_module(model_name)
%CREATE_MOTOR_MODULE Build the standalone motor module model.

if nargin < 1 || isempty(model_name)
    model_name = 'pmsm_module_motor';
end

model_path = pmsm_foc_builder('create_module_model', 'motor', model_name);
end