function model_path = create_signal_in_module(model_name)
%CREATE_SIGNAL_IN_MODULE Build the standalone signal-in module model.

if nargin < 1 || isempty(model_name)
    model_name = 'pmsm_module_signal_in';
end

model_path = pmsm_foc_builder('create_module_model', 'signal_in', model_name);
end