%% Assess Participant
% PURPOSE: Loop through all condition folders and generate ConditionSummaryFiles in a participant and compute the average scores for each condition and 
% place them in the file ParticipantXXMasterFile.xlsx for easy viewing.
% Include the raw data as well. Difference here is that each trial is split
% into a forward and a backwards pass


% OUTPUT: An xlsx file titled "ParticipantMasterFile.xlsx"., containing all of
% the results

%% Load data 

% Query user for a folder name
folderName = uigetdir('', 'Select a Folder');

% Check if the user selected a folder or cancelled the dialog
if folderName == 0
    disp('User cancelled the folder selection.');
else
    % Get a list of all files in the selected folder
    fileData = dir(folderName);
    
    % Filter out directories (keep only files)
    fileNames = {fileData(~[fileData.isdir]).name};
    
    % Display the file names
    if isempty(fileNames)
        disp('No files found in the selected folder.');
    else
        disp('Files in the selected folder:');
        disp(fileNames);
    end
end

%% Loop through each condition and extract the necessary data

conditions = ["STAT", "AUT", "ENDO"];

totalMeanCompletionTime = [];
totalStdCompletionTime = [];
totalMeanNumberOfCollisions = [];
totalStdNumberOfCollisions = [];
totalMeanCumulativeTimeOfCollisions = [];
totalStdCumulativeTimeOfCollisions = [];

%take the last row of the following columns: 
if folderName ~= 0
    
    for i = 1:length(conditions)
        %read in file 
        condition = conditions(i);


        conditionFolder = strcat( strcat(folderName,"/"), string(condition));
        dataTable = AssessCondition(conditionFolder);

        %extract relevant data
        completionTime = dataTable.("completionTime(s)");
        numberOfCollisions = dataTable.("numberOfCollisions");
        cumulativeTimeOfCollisions = dataTable.("cumulativeTimeOfCollisions(s)");

        %computeSummaryStatistics
        meanCompletionTime = mean(completionTime); stdCompletionTime = std(completionTime);
        meanNumberOfCollisions = mean(numberOfCollisions); stdNumberOfCollisions = std(numberOfCollisions);
        meanCumulativeTimeOfCollisions = mean(cumulativeTimeOfCollisions); stdCumulativeTimeOfCollisions = std(cumulativeTimeOfCollisions);
        
        totalMeanCompletionTime = [totalMeanCompletionTime; meanCompletionTime];
        totalStdCompletionTime = [totalStdCompletionTime; stdCompletionTime];
        totalMeanNumberOfCollisions = [totalMeanNumberOfCollisions; meanNumberOfCollisions];
        totalStdNumberOfCollisions = [totalStdNumberOfCollisions; stdNumberOfCollisions];
        totalMeanCumulativeTimeOfCollisions =[totalMeanCumulativeTimeOfCollisions; meanCumulativeTimeOfCollisions]; 
        totalStdCumulativeTimeOfCollisions = [totalStdCumulativeTimeOfCollisions; stdCumulativeTimeOfCollisions];

    end

    %% Write to file
    
    dataToExport = table(totalMeanCompletionTime, totalStdCompletionTime,totalMeanNumberOfCollisions,totalStdNumberOfCollisions, totalMeanCumulativeTimeOfCollisions, totalStdCumulativeTimeOfCollisions,...
                  'VariableNames', {'meanCompletionTime', 'stdCompletionTime', 'meanNumberOfCollisions','stdNumberOfCollisions','meanCumulativeTimeOfCollisions','stdCumulativeTimeOfCollisions'}, ...
                  'RowNames', conditions);
    % Convert row names to a column
    dataToExport = addvars(dataToExport, dataToExport.Properties.RowNames, ...
                                    'Before', 1, 'NewVariableNames', 'Condition');


    % Write to an Excel file
    writetable(dataToExport,folderName+"/"+"ParticipantSummary.xlsx" , 'Sheet', 1);
end



