function stage_example_drive_blocks(example_model_path, wrapper_model_name)
%STAGE_EXAMPLE_DRIVE_BLOCKS Copy key blocks from PMSMDrive example to wrapper.
%   stage_example_drive_blocks
%   stage_example_drive_blocks(example_model_path, wrapper_model_name)
%
% This script copies three blocks from the official example model into
% my_blue_plant_wrapper/PlantCore/ExampleBorrowed for reference:
%   - Three-phase inverter
%   - Encoder
%   - Permanent Magnet Synchronous Motor

if nargin < 1 || isempty(example_model_path)
    example_model_path = '/Users/wangguandi/Documents/MATLAB/Examples/R2024b/simscapeelectrical/PMSMDriveExample/PMSMDrive.slx';
end
if nargin < 2 || isempty(wrapper_model_name)
    wrapper_model_name = 'my_blue_plant_wrapper';
end

if ~exist(example_model_path, 'file')
    error('Example model not found: %s', example_model_path);
end

script_dir = fileparts(mfilename('fullpath'));
project_root = fullfile(script_dir, '..', '..');
wrapper_path = fullfile(project_root, 'matlab', 'models', [wrapper_model_name '.slx']);

if ~exist(wrapper_path, 'file')
    error('Wrapper model not found: %s', wrapper_path);
end

load_system(example_model_path);
[~, src_model, ~] = fileparts(example_model_path);
load_system(wrapper_path);

plant_core = [wrapper_model_name '/PlantCore'];
if isempty(find_system(wrapper_model_name, 'SearchDepth', 1, 'Name', 'PlantCore'))
    error('PlantCore subsystem not found in wrapper model: %s', wrapper_model_name);
end

staging = [plant_core '/ExampleBorrowed'];
if ~isempty(find_system(plant_core, 'SearchDepth', 1, 'Name', 'ExampleBorrowed'))
    delete_block(staging);
end

add_block('built-in/Subsystem', staging, 'Position', [360 10 950 320]);

src_inv = [src_model '/' sprintf('Three-phase\ninverter')];
src_enc = [src_model '/Encoder'];
src_pmsm = [src_model '/' sprintf('Permanent Magnet\nSynchronous Motor')];

dst_inv = [staging '/' sprintf('Three-phase\ninverter')];
dst_enc = [staging '/Encoder'];
dst_pmsm = [staging '/' sprintf('Permanent Magnet\nSynchronous Motor')];

add_block(src_inv, dst_inv, 'Position', [40 70 180 170]);
add_block(src_pmsm, dst_pmsm, 'Position', [260 70 420 170]);
add_block(src_enc, dst_enc, 'Position', [460 70 610 170]);

try
    ann = Simulink.Annotation([staging '/Reference blocks copied from PMSMDrive example.']);
    ann.Position = [20 15 420 35];
catch
end

save_system(wrapper_model_name, wrapper_path);
open_system(wrapper_model_name);
open_system(staging);

fprintf('Staging completed in: %s\n', staging);
fprintf('Copied blocks:\n');
fprintf('  - %s\n', dst_inv);
fprintf('  - %s\n', dst_pmsm);
fprintf('  - %s\n', dst_enc);
end
