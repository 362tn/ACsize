function xflr5_save(file_name,output_name)
%XFLR5_SAVE Takes exported XFLR5 txt file and converts data to a .mat file
% 1st argument in (file_name) is the name of the .txt file
% 2nd argument in (output_name) is the name of the .mat file.
%
% Enter file names as strings
%
% Returns data in following format:
% alpha CL CD


% File Open
XFile = fopen(file_name,'r');
if XFile == -1
    disp('File not successfully opened')
    return
end

% File Read and Save
Airfoil_Temp = textscan(XFile,'%f %f %f %f %f %f %f %f %f %f','HeaderLines',11);

Airfoil_Data = zeros(length(Airfoil_Temp{1}),3);
for i = 1:3
    Airfoil_Data(:,i) = Airfoil_Temp{i}; 
end

% File Close
close = fclose(XFile);
if close ~= 0
    disp('File not successfully closed')
    return
end

% Saving Data
save(output_name,'Airfoil_Data');

end

% xflr5_save.m
% Trey Green
% treybgreen@gmail.com
% 9/8/2017



