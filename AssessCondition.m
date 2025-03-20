function [dataToExport] = AssessCondition(folderName)
%% Assess Condition
% PURPOSE: To loop through data of a single condition consisting of 5 (or more) 
% trials. 

% OUTPUT: An xlsx file titled "ConditionSummary.xlsx"., containing all of
% the results

%% Load data 

if nargin < 1
    % Query user for a folder name
    folderName = uigetdir('', 'Select a Folder');


    % Check if the user selected a folder or cancelled the dialog
    if folderName == 0
        disp('User cancelled the folder selection.');
        return;
    end
end
    
    % Get a list of all files in the selected folder
    fileData = dir(folderName);
    
    % Filter out directories (keep only files)
    fileNames = {fileData(~[fileData.isdir]).name};
    
    % Display the file names
    if isempty(fileNames)
        disp('No files found in the selected folder.');
        return;
    else
        disp('Files in the selected folder:');
        disp(fileNames);
    end
    

%% Loop through each file and extract the necessary data
completionTime = [];
numberOfCollisions = [];
cumulativeTimeOfCollisions = [];


%take the last row of the following columns: 
    
    numberOfFiles = length(fileNames);

    for i = 1:numberOfFiles
        %read in file 
        fileName = fileNames(i);
        if fileName == "ConditionSummary.xlsx"
            continue;
        end

        file = strcat( strcat(folderName,"/"), string(fileName));
        dataTable = readtable(file);
        
        %extract relevant data
        completionTime = [completionTime; dataTable.TaskDuration_s_(end)];
        numberOfCollisions = [numberOfCollisions; dataTable.Collisions_increments_(end)];
        cumulativeTimeOfCollisions = [cumulativeTimeOfCollisions; dataTable.CumulativeCollisionTime_s_(end)];
    end
    

    %write to file
    dataToExport = table(completionTime, numberOfCollisions, cumulativeTimeOfCollisions, ...
             'VariableNames', {'completionTime(s)', 'numberOfCollisions', 'cumulativeTimeOfCollisions(s)'});

    % Write to an Excel file
    writetable(dataToExport,folderName+"/"+"ConditionSummary.xlsx" , 'Sheet', 1);

end

