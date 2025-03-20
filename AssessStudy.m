%% Assess Study
% PURPOSE: Loop through all particpants to generate a cell array for each
% participant that contains three tables (one for each condition). After,
% combines all participant data into one table (columns are outcome
% metrics). Generates mean and std for this combined table and then
% computes, for each participant, z-scores for each condition and saves in
% table_f.


% OUTPUT: An xlsx file titled "ParticipantMasterFile.xlsx"., containing all of
% the results
close all; 
clear all;

%% Load data 

dataDir = "C:\Users\alexa\OneDrive\Documents\UBC_Thesis\Code\RCL_autocam\dataCollection\ParticipantData";

studyData = []; %contains multiple participant structs

% Iterate through the following participants
participantsToInclude = ["P02", "P03","P04","P05","P06","P07"];
for x = 1:length(participantsToInclude)
    participant = participantsToInclude(x);
    folderName = dataDir + "/" + participant;

    conditions = ["STAT", "AUT", "ENDO"];
    %for each participant, struct containing {participantName, STATdata, AUTdata, ENDOdata}
    participantData = struct("participant", participant, conditions(1), table(), conditions(2), table(), conditions(3), table(), "summary", table());
    % Loop through each condition and extract the necessary data
        for i = 1:length(conditions)
            %load table 
            condition = conditions(i);
            conditionFolder = strcat( strcat(folderName,"/"), string(condition));
            %add table to participant data
            participantData.(condition) = readtable(conditionFolder +"/ConditionSummary.xlsx");

            %change how many trials are used
            participantData.(condition) = participantData.(condition)(1:end, :); %Using all trials
        end
        % Append struct to list
        studyData = [studyData; participantData];
end
    % Write to an Excel file
    % writetable(dataToExport,folderName+"/"+"ParticipantSummary.xlsx" , 'Sheet', 1);

%% Generate Global Dataframe 
globalData = [];
for i = 1:length(studyData)
    globalData = [globalData; studyData(i).STAT;studyData(i).AUT;studyData(i).ENDO];
end

%% Generate mean and std dev for all variables
means = [];
stdDevs = [];
maxi = [];
mini = [];

for i = 1:length(globalData.Properties.VariableNames)
    means = [ means mean(globalData.(string(globalData.Properties.VariableNames(i)))) ];
    stdDevs = [ stdDevs std(globalData.(string(globalData.Properties.VariableNames(i)))) ];
    maxi = [maxi;max(globalData.(string(globalData.Properties.VariableNames(i))))];
    mini = [mini; min(globalData.(string(globalData.Properties.VariableNames(i))))];
end

%% Compute Z Score Table for each participant 

%copy data to new data frame 
normalizedData = studyData;

%iterate through all participant tables and standardize respective variables 
for i = 1:length(normalizedData)
    
    %iterate through the various data tables 
    for j = 1:length(conditions)
        condition = conditions(j);

        %iterate through the variable names
        targetTable = normalizedData(i).(condition);
        for p = 1:length(globalData.Properties.VariableNames)
            %Zscore normalization
            % targetTable.(string(globalData.Properties.VariableNames(p))) = ( targetTable.(string(globalData.Properties.VariableNames(p))) - means(p) ) / stdDevs(p);

            %min max normalization
             targetTable.(string(globalData.Properties.VariableNames(p))) = ( targetTable.(string(globalData.Properties.VariableNames(p))) - mini(p) ) / (maxi(p) - mini(p));
        end
        %save table 
        normalizedData(i).(condition) = targetTable;
    end
end

%Generate a Z-score dataframe
zScore_Data = normalizedData;

%iterate through all participant tables and compute z score
for i = 1:length(normalizedData)
    
    %iterate through the various data tables 
    for j = 1:length(conditions)
        condition = conditions(j);

        %iterate through the variable names to find the z score
        targetTable = normalizedData(i).(condition);
        zScore = [];
        for p = 1:height(targetTable)
            zScore = [zScore; table2array(targetTable(p,"completionTime_s_")) + table2array(targetTable(p,"cumulativeTimeOfCollisions_s_")) + table2array(targetTable(p,"numberOfCollisions"))];
        end
        %save table 
        targetTable.Zscore = zScore;
        zScore_Data(i).(condition) = targetTable;
    end
end

%% Compute Mean and Std for Each condition in each participant
summaryStatistics = [];

%create n x 6 table that contains variables [STATmean, STATstd, AUTmean, AUTstd, ENDOmean, ENDOstd]
for i = 1:length(zScore_Data)
    participant = [];
    %iterate through the various data tables 
    for j = 1:length(conditions)
        condition = conditions(j);

        %iterate through the variable names to find the z score
        targetTable = zScore_Data(i).(condition);
        
        
        participant = [ participant, mean(targetTable.("Zscore")), std(targetTable).("Zscore") ];
       
        
    end
    summaryStatistics = [summaryStatistics; participant];
end

%% Computer percent error (collision duration/task duration)

%Creates nx6 array, where n=number of participants, and cols=[STATCollisionDurationMean,
% STATTaskDurationMean,AUTCollisionDurationMean,AUTTaskDurationMean,ENDOCollisionDurationMean,ENDOTaskDurationMean]
durationMatrixMean=[];
durationMatrixStd=[];

for i=1:length(studyData)
    durationMatrixMean=[durationMatrixMean;mean(studyData(i).STAT.cumulativeTimeOfCollisions_s_(:)),...
        mean(studyData(i).STAT.completionTime_s_(:)),...
    mean(studyData(i).AUT.cumulativeTimeOfCollisions_s_(:)),...
    mean(studyData(i).AUT.completionTime_s_(:)),...
    mean(studyData(i).ENDO.cumulativeTimeOfCollisions_s_(:)),...
    mean(studyData(i).ENDO.completionTime_s_(:))];

    durationMatrixStd=[durationMatrixStd;std(studyData(i).STAT.cumulativeTimeOfCollisions_s_(:)),...
        std(studyData(i).STAT.completionTime_s_(:)),...
    std(studyData(i).AUT.cumulativeTimeOfCollisions_s_(:)),...
    std(studyData(i).AUT.completionTime_s_(:)),...
    std(studyData(i).ENDO.cumulativeTimeOfCollisions_s_(:)),...
    std(studyData(i).ENDO.completionTime_s_(:))];

end

perc_error_mean=[durationMatrixMean(:,1)./durationMatrixMean(:,2),durationMatrixMean(:,3)./durationMatrixMean(:,4),durationMatrixMean(:,5)./durationMatrixMean(:,6)];
perc_error_combined_std=[];

for i=1:3
    perc_error_combined_std=[perc_error_combined_std,...
        sqrt((durationMatrixStd(:,i*2-1)./durationMatrixMean(:,i*2-1)).^2+(durationMatrixStd(:,i*2)./durationMatrixMean(:,i*2)).^2).*abs(perc_error_mean(:,i))];

end

%% Computer Difference for each person and take population mean and std of difference

%Finds an nx6 array where n=number of participants,
%columns=[task dur stat-aut,task dur endo-aut,
%num collisions stat-aut,num collisions endo-aut,
%collision duration stat-aut,num collisions endo-au]

differenceMatrix=[];

for i=1:length(studyData)

    differenceMatrix=[differenceMatrix;mean(studyData(i).STAT.completionTime_s_(:))-mean(studyData(i).AUT.completionTime_s_(:)),...
        mean(studyData(i).ENDO.completionTime_s_(:))-mean(studyData(i).AUT.completionTime_s_(:)),...
        mean(studyData(i).STAT.numberOfCollisions(:))-mean(studyData(i).AUT.numberOfCollisions(:)),...
        mean(studyData(i).ENDO.numberOfCollisions(:))-mean(studyData(i).AUT.numberOfCollisions(:)),...
        mean(studyData(i).STAT.cumulativeTimeOfCollisions_s_(:))-mean(studyData(i).AUT.cumulativeTimeOfCollisions_s_(:)),...
        mean(studyData(i).ENDO.cumulativeTimeOfCollisions_s_(:))-mean(studyData(i).AUT.cumulativeTimeOfCollisions_s_(:))];

end

%% Boxplot of cumulitive difference measures
%Grouping Variable
group=[repmat({'task dur stat-aut'},size(differenceMatrix,1),1);...
    repmat({'task dur endo-aut'},size(differenceMatrix,1),1);...
    repmat({'num collisions stat-aut'},size(differenceMatrix,1),1);...
    repmat({'num collisions endo-aut'},size(differenceMatrix,1),1);...
    repmat({'collision duration stat-aut'},size(differenceMatrix,1),1);...
    repmat({'num collisions endo-aut'},size(differenceMatrix,1),1)];

figure;
h=boxplot(differenceMatrix,group,'MedianStyle','line','BoxStyle','outline');
title('Population Errors in Outcome Vars','FontName', 'Arial', 'FontSize', 14,'FontWeight','bold')
ylabel('Population Error','FontName', 'Arial', 'FontSize', 14);
set(h, 'LineWidth', 2);


%% Generate bar plot where each participant has a zscore for each condition



%-------------------------------


% Number of clusters
n = length(participantsToInclude); % Adjust for your number of clusters

% Example data: Randomly generate data for each bar in the clusters
data = perc_error_mean; %(:, [1,3,5]); % n rows for clusters, 3 columns for bars in each cluster
std_dev = perc_error_combined_std; %(:,[2,4,6]); % Random standard deviation values

% Generate the grouped bar plot
figure;
b = bar(data, 'grouped'); % Generate grouped bars
hold on;

% Add error bars
% Get the x-coordinates of the bars for each group
[~, numBars] = size(data);
x = nan(size(data)); % Initialize matrix for bar positions
for i = 1:numBars
    x(:, i) = b(i).XEndPoints; % Get x positions of bars
end

% Add error bars for each bar
for i = 1:numBars
    errorbar(x(:, i), data(:, i), std_dev(:, i), 'k', 'linestyle', 'none', 'LineWidth', 1);
end

% Add labels for clusters and bars
xticks(1:n); % Set x-axis ticks to match the number of clusters
% xticklabels({'Pilot01', 'Pilot02', 'Pilot03', 'Pilot04', 'Pilot05', 'Pilot06'}); % Adjust labels as needed
xticklabels({'P02', 'P03','P04','P05','P06','P07'});
xlabel('Participants');
ylabel('Percent Error (%/100)');
title('Percent Error for Each Participant and Condition');

% Add a legend for the bars in each cluster
legend({'STAT', 'AUT', 'ENDO'}, 'Location', 'northeast');

hold off;



%% Plot NASA TLX
NASATLX =  [25	9.5	18;
            20	20	28.5;
            25	9.5	18;
            20	20	28.5;
            25	9.5	18;
            20	20	28.5
            ];

% Generate the grouped bar plot
figure;
b = bar(NASATLX, 'grouped'); % Generate grouped bars
hold on;

% Add error bars
% Get the x-coordinates of the bars for each group
[~, numBars] = size(data);
x = nan(size(data)); % Initialize matrix for bar positions
for i = 1:numBars
    x(:, i) = b(i).XEndPoints; % Get x positions of bars
end

% Add error bars for each bar
for i = 1:numBars
    errorbar(x(:, i), data(:, i), std_dev(:, i), 'k', 'linestyle', 'none', 'LineWidth', 1);
end

% Add labels for clusters and bars
xticks(1:n); % Set x-axis ticks to match the number of clusters
% xticklabels({'Pilot01', 'Pilot02', 'Pilot03', 'Pilot04', 'Pilot05', 'Pilot06'}); % Adjust labels as needed
xticklabels({'P02', 'P03','P04','P05','P06','P07'});

xlabel('Participants');
ylabel('NASA TLX');
title('NASA TLX for Each Participant and Condition');

% Add a legend for the bars in each cluster
legend({'STAT', 'AUT', 'ENDO'}, 'Location', 'northeast');

hold off;


%% Generate plots of all variables 
% For each variable, plot the mean and std for all participants 

%loop through data frame
%create summary table that is appended to the struct for each participant
for i = 1:length(studyData)
    %iterate through the various data tables 
    participantSummary = table();
    for j = 1:length(conditions)
        condition = conditions(j);
        %iterate through the variable names to find the z score
        targetTable = studyData(i).(condition);
        newTable = table();
        newTable.completionTime_s__mean = mean(targetTable.completionTime_s_);
        newTable.completionTime_s__std = std(targetTable.completionTime_s_);

        newTable.numberOfCollisions_mean = mean(targetTable.numberOfCollisions);
        newTable.numberOfCollisions_std = std(targetTable.numberOfCollisions);

        newTable.cumulativeTimeOfCollisions_s__mean = mean(targetTable.cumulativeTimeOfCollisions_s_);
        newTable.cumulativeTimeOfCollisions_s__std = std(targetTable.cumulativeTimeOfCollisions_s_);
        participantSummary = [newTable; participantSummary];
    end
    %save table 
    participantSummary.Properties.RowNames = conditions;
    studyData(i).summary = participantSummary;
end

%combine all summaries into different groups
combinedAUT = table();
combinedSTAT = table();
combinedENDO = table();


for i = 1:length(studyData)
    AUT = studyData(i).summary("AUT",:);
    AUT.Properties.RowNames = {};
    STAT = studyData(i).summary("STAT",:);
    STAT.Properties.RowNames = {};
    ENDO = studyData(i).summary("ENDO",:);
    ENDO.Properties.RowNames = {};
    combinedAUT = [combinedAUT; AUT];
    combinedSTAT = [combinedSTAT; STAT];
    combinedENDO = [combinedENDO; ENDO];
end

%plot each variable
figure;
for i = 1:length(studyData(1).AUT.Properties.VariableNames)
    variable = string(studyData(1).AUT.Properties.VariableNames(i));
    mean_data = [];
    std_data = [];
    for j = 1:height(combinedAUT)
        row = [combinedSTAT.(variable+"_mean")(j)  combinedAUT.(variable+"_mean")(j)  combinedENDO.(variable+"_mean")(j)]; 
        mean_data = [mean_data; row];

        row = [combinedSTAT.(variable+"_std")(j)  combinedAUT.(variable+"_std")(j)  combinedENDO.(variable+"_std")(j)]; 
        std_data = [std_data; row];
    end
    subplot(3,1,i); 
    % Number of clusters
    n = length(participantsToInclude); % Adjust for your number of clusters
    
    data = mean_data; % n rows for clusters, 3 columns for bars in each cluster
    std_dev = std_data; % Random standard deviation values
    
    % Generate the grouped bar plot
    b = bar(data, 'grouped'); % Generate grouped bars
    hold on;
    
    % Add error bars
    % Get the x-coordinates of the bars for each group
    [~, numBars] = size(data);
    x = nan(size(data)); % Initialize matrix for bar positions
    for i = 1:numBars
        x(:, i) = b(i).XEndPoints; % Get x positions of bars
    end
    
    % Add error bars for each bar
    for i = 1:numBars
        errorbar(x(:, i), data(:, i), std_dev(:, i), 'k', 'linestyle', 'none', 'LineWidth', 1);
    end
    
    % Add labels for clusters and bars
    xticks(1:n); % Set x-axis ticks to match the number of clusters
    % xticklabels({'Pilot01', 'Pilot02', 'Pilot03', 'Pilot04', 'Pilot05', 'Pilot06'}); % Adjust labels as needed
    xticklabels({'P02', 'P03','P04','P05','P06','P07'});

    xlabel('Participants');
    ylabel(variable);
    title(variable + ' for Each Participant and Condition');
    
    % Add a legend for the bars in each cluster
    legend({'STAT', 'AUT', 'ENDO'}, 'Location', 'northeast');
    
    hold off;

end

%% Generate any necessary data to compute statistical significance 


