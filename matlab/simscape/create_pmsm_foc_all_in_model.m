function model_path = create_pmsm_foc_all_in_model(model_name, motor_params, inv_params, ctrl_params, sim_params, ref_params)
%CREATE_PMSM_FOC_ALL_IN_MODEL Build the assembled PMSM FOC Simulink model.

if nargin == 0
    [motor_params, inv_params, ctrl_params, sim_params, ref_params] = pmsm_foc_builder('default_parameters');
    model_name = 'pmsm_foc_model';
elseif nargin == 1
    [motor_params, inv_params, ctrl_params, sim_params, ref_params] = pmsm_foc_builder('default_parameters');
elseif nargin ~= 6
    error('Expected either 0 inputs or 6 inputs.');
end

model_path = pmsm_foc_builder( ...
    'create_all_in_model', model_name, motor_params, inv_params, ctrl_params, sim_params, ref_params);
end