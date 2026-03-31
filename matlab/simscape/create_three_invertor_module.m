function model_path = create_three_invertor_module(model_name)
%CREATE_THREE_INVERTOR_MODULE Build the standalone three-invertor module model.

if nargin < 1 || isempty(model_name)
    model_name = 'pmsm_module_three_invertor';
end

model_path = pmsm_foc_builder('create_module_model', 'three_invertor', model_name);
end