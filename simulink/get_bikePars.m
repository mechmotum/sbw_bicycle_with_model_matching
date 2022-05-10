function [ bikePar ] = get_bikePars(bike_name)
% GET_BIKEPARS function reads the bicycle parameters from a text file.
%   bikePar = GET_BIKEPARS(BIKE_NAME)
% Returns a struct with all the bicycle parameters. Input is a bicycle name
% corresponding to a bicycle in /bicycles/ folder, but without
% '_bicycle.txt'
    
%% Read the file contents
    call_stack = dbstack('-completenames');
    [current_dir, ~, ~] = fileparts(call_stack(1).file);
    full_path = strcat(current_dir, filesep, 'bicycles', ...
        filesep, bike_name, '_bicycle.txt');
    fid = fopen(full_path, 'rt');
    C = textscan(fid,'%s%f');
    fclose(fid);
%% Put everything to a struct
    for i = 1:length(C{1})
        bikePar.(string(C{1,1}(i))) = C{1,2}(i);
    end

end