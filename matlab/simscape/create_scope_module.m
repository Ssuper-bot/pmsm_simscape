function model_path = create_scope_module(model_name)
%CREATE_SCOPE_MODULE Build the standalone scope module model.

if nargin < 1 || isempty(model_name)
    model_name = 'pmsm_module_scope';
end

model_path = pmsm_foc_builder('create_module_model', 'scope', model_name);
end