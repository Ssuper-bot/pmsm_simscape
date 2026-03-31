function model_path = create_foc_controller_module(model_name)
%CREATE_FOC_CONTROLLER_MODULE Build the standalone FOC controller module model.

if nargin < 1 || isempty(model_name)
    model_name = 'pmsm_module_foc_controller';
end

model_path = pmsm_foc_builder('create_module_model', 'foc_controller', model_name);
end