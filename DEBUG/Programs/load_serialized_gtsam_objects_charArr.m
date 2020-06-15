function [charArr] = load_serialized_gtsam_objects_charArr(filename, varargin)
%%  import GTSAM serialized data from the text file
%   Author: Andrej Kitanov
%   ANPL, Technion

%% Initialize variables.
delimiter = {''};

%% Read columns of data as text:
% For more information, see the TEXTSCAN documentation.
formatSpec = '%s%[^\n\r]';

%% Open the text file.
fileID = fopen(filename,'r');

%% Read columns of data according to the format.
% This call is based on the structure of the file used to generate this
% code. If an error occurs for a different file, try regenerating the code
% from the Import Tool.
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'TextType', 'string',  'ReturnOnError', false);

%% Close the text file.
fclose(fileID);

%% Convert the contents of columns containing numeric text to numbers.
% Replace non-numeric text with NaN.
raw = repmat({''},length(dataArray{1}),length(dataArray)-1);
for col=1:length(dataArray)-1
    raw(1:length(dataArray{col}),col) = mat2cell(dataArray{col}, ones(length(dataArray{col}), 1));
end
numericData = NaN(size(dataArray{1},1),size(dataArray,2));


%% Split data into numeric and string columns.
rawNumericColumns = {};
rawStringColumns = string(raw(:, 1));


%% Create output variable
cppstr = join(rawStringColumns,newline);

%% Replace boost archive version
if nargin > 1
    cppstr = replace(cppstr,"serialization::archive "+ varargin{1}(1),"serialization::archive " + varargin{1}(2));
end


%% Convert to character array
%charArr = convertStringsToChars(cppstr);
charArr = char(cppstr);
% print first 28 chars to see the archive version
charArr(1:28)
%% Clear temporary variables
clearvars filename delimiter formatSpec fileID dataArray ans raw col numericData rawNumericColumns rawStringColumns;

end

