function data = import_csv_as_table(filename, headers, dataLines)
%IMPORT_CSV_AS_TABLE Import data from a csv file
%  DATA = IMPORT_CSV_AS_TABLE(FILENAME) reads data from csv file FILENAME
%  for the default selection.  Returns the data as a table.
%
%  DATA = IMPORT_CSV_AS_TABLE(FILE, DATALINES) reads data for the
%  specified row interval(s) of text file FILENAME. Specify DATALINES as
%  a positive scalar integer or a N-by-2 array of positive scalar
%  integers for dis-contiguous row intervals.
%
%  See also READTABLE.

%% Input handling

% If dataLines is not specified, define defaults
if nargin < 2
    headers = {'index', 'relative_time', 'ExactoKnife_x', 'ExactoKnife_y', 'ExactoKnife_z', 'ExactoKnife_qx', 'ExactoKnife_qy', 'ExactoKnife_qz', 'ExactoKnife_qw', 'force_x', 'force_y', 'force_z', 'displacement'};
    dataLines = [2, Inf];
end

if nargin < 3
    dataLines = [2, Inf];
end

%% Set up the Import Options and import the data
opts = delimitedTextImportOptions("NumVariables", 13);

% Specify range and delimiter
opts.DataLines = dataLines;
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["index", "relative_time", "ExactoKnife_x", "ExactoKnife_y", "ExactoKnife_z", "ExactoKnife_qx", "ExactoKnife_qy", "ExactoKnife_qz", "ExactoKnife_qw", "force_x", "force_y", "force_z", "displacement"];
opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double"];

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Import the data
data = readtable(filename, opts);
data = data(:, ismember(data.Properties.VariableNames, headers));

end