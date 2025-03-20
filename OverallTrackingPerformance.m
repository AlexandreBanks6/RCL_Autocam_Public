clear
close all
clc


%% %%%%%%%%%%%%%%%%%%%%%%1. Read in data%%%%%%%%%%%%%%%%%%%%%%%%%
DATA_ROOT='TrackingValidationData_March8';

%Gets data for multiple and single scene registration for left/right cams
data_p1=xlsread([DATA_ROOT,'\p1_trials.csv']);
data_p2=xlsread([DATA_ROOT,'\p2_trials.csv']);

data_p1_table=readtable([DATA_ROOT,'\p1_trials.csv'],'TextType','string');
data_p2_table=readtable([DATA_ROOT,'\p2_trials.csv'],'TextType','string');
n_p1=length(data_p1(7:end,1));
combined_data=[data_p1(7:end,:);data_p2(7:end,:)];
p1_time=string(data_p1_table{2:end,2});
p2_time=string(data_p2_table{2:end,2});
p1_time=datetime(p1_time,'InputFormat','HH:mm:ss.SSSSSS');
p2_time=datetime(p2_time,'InputFormat','HH:mm:ss.SSSSSS');

p1_time=seconds(p1_time-p1_time(1));
p2_time=seconds(p2_time-p2_time(1));

time_vector=[p1_time;p1_time(end)+0.006+p2_time];

psm3_T_cam=convertCSVRowToHomo(data_p1(3,2:13));
offset=data_p1(1,2:4);
time_vector(n_p1+1)=[]; %Get rid of sample when we change participants
time_vector(end)=[]; %Get rid of last time

NoGo_Points=[-0.02562659  0.19977891 -0.09145149;
    -0.10775786  0.19640249 -0.08759377;
    -0.10963935  0.13565024 -0.1298436;
    -0.0273852   0.13670756 -0.13512043;
    -0.07120856  0.2261429  -0.12999408];
 floor_offset = 0.04;

%% %%%%%%%%%%%%%%%%%Calcuating Errors%%%%%%%%%%%%%%%%%%%%%%%%%%%%
orientation_errors=[]; %Vector of orientation error for every entry
centroid_viewingvector_angle_errors=[];
perp_to_floor_errors=[];
distance_to_ring_errors=[];
distances_from_desired=[];
ecm_T_cam_desired_vec=[];
ecm_T_cam_actual_vec=[];
ecm_T_ring_vec=[];

[row,col]=size(combined_data);
for i=[1:row-1]
    %%%%%Extracting transforms
    if(i==n_p1) %New participant
        continue;
    end
    ecm_T_ring_target=combined_data(i,101:112);
    ecm_T_ring_target=convertCSVRowToHomo(ecm_T_ring_target);

    ecm_T_world=combined_data(i+1,114:125);
    ecm_T_world=convertCSVRowToHomo(ecm_T_world);

    ecm_T_psm3_raw=combined_data(i+1,36:47);
    ecm_T_psm3_raw=convertCSVRowToHomo(ecm_T_psm3_raw);
    

    %Computes Ideal desired Pose
    ecm_T_world_curr=combined_data(i,114:125);
    ecm_T_world_curr=convertCSVRowToHomo(ecm_T_world_curr);

    ecm_T_psm3_raw_curr=combined_data(i,36:47);
    ecm_T_psm3_raw_curr=convertCSVRowToHomo(ecm_T_psm3_raw_curr);
    
    z_i=ecm_T_psm3_raw_curr(1:3,3);
    ecm_T_psm3_desired = orientCamera(psm3_T_cam, ecm_T_ring_target, ecm_T_world_curr, z_i, 0.11, offset');
    [interpolationRequired, intermediatePose] = interpolatePose(ecm_T_psm3_desired, ecm_T_psm3_raw_curr);
    if(interpolationRequired)
        ecm_T_psm3_desired=intermediatePose;
    end
    ecm_T_cam_desired=ecm_T_psm3_desired*psm3_T_cam;
    ecm_T_cam_actual=ecm_T_psm3_raw_curr*psm3_T_cam;
    ecm_T_cam_desired_vec=[ecm_T_cam_desired_vec;ecm_T_cam_desired(1:3,4)'];
    ecm_T_cam_actual_vec=[ecm_T_cam_actual_vec;ecm_T_cam_actual(1:3,4)']; %Trajectory, for jerk computation
    ecm_T_ring_vec=[ecm_T_ring_vec;ecm_T_ring_target(1:3,4)'];

    % ecm_T_psm3_des_proximity=combined_data(i,75:86); %If proximity flag is triggered, desired pose prior
    % ecm_T_psm3_des=combined_data(i,62:73); %If any other flag triggered, desired pose prior    
    % ecm_T_psm3_des_noflags=combined_data(i,88:99); %Desired if not flags triggered
    % if(~(ecm_T_psm3_des_proximity(1)==-1))
    %     ecm_T_psm3_desired=convertCSVRowToHomo(ecm_T_psm3_des_proximity);
    % elseif(~(ecm_T_psm3_des(1)==-1))
    %     ecm_T_psm3_desired=convertCSVRowToHomo(ecm_T_psm3_des);
    % else
    %     ecm_T_psm3_desired=convertCSVRowToHomo(ecm_T_psm3_des_noflags);
    % end

    
    %%%%%Computing errors
    orientation_error=orientationError(ecm_T_ring_target,ecm_T_psm3_raw,psm3_T_cam);
    centroid_viewingvector_angle_error=centroidTrackingAngleError(ecm_T_ring_target,ecm_T_psm3_raw,psm3_T_cam,offset');
    perp_to_floor_error=perpendicularToFloorError(ecm_T_psm3_raw,psm3_T_cam,ecm_T_world);
    distance_to_ring_error=distanceToRingError(ecm_T_psm3_raw,psm3_T_cam,ecm_T_ring_target,offset',0.11)*1000; %in mm
    distance_from_desired=distanceFromDesired(ecm_T_psm3_raw,ecm_T_psm3_desired,psm3_T_cam).*1000; %in mm

    %%%%Updating errors
    orientation_errors=[orientation_errors;orientation_error];
    centroid_viewingvector_angle_errors=[centroid_viewingvector_angle_errors;centroid_viewingvector_angle_error];
    perp_to_floor_errors=[perp_to_floor_errors;perp_to_floor_error];
    distance_to_ring_errors=[distance_to_ring_errors;distance_to_ring_error];
    distances_from_desired=[distances_from_desired;distance_from_desired];

end

%% Computes the jerk and does jerk stats
actual_trajectory_jerk=computeJerkMag(ecm_T_cam_actual_vec,time_vector);
desired_trajectory_jerk=computeJerkMag(ecm_T_cam_desired_vec,time_vector);
ring_trajectory_jerk=computeJerkMag(ecm_T_ring_vec,time_vector);

disp(['Actual Traj Jerk: ',num2str(mean(actual_trajectory_jerk)),' STD=',num2str(std(actual_trajectory_jerk))]);

%% %%%%%%%%%%%%Running Stats%%%%%%%%%%%%%%%%%

flags_for_plot=[combined_data(1:end,5:8),combined_data(1:end,3)];
flags_for_plot(n_p1+1,:)=[]; %Get rid of sample when we change participants
flags_for_plot(end,:)=[]; %Get rid of last time
elapsed_time_vec=[combined_data(1:n_p1,1);combined_data(n_p1+2:end-1,1)];

disp('********************All STATS*************************')
disp(['Centroid View Vector Error Mean=',num2str(mean(centroid_viewingvector_angle_errors)), ...
    ' STD=',num2str(std(centroid_viewingvector_angle_errors)),' (deg)']);

disp(['Perp To Floor Error Mean=',num2str(mean(perp_to_floor_errors)), ...
    ' STD=',num2str(std(perp_to_floor_errors)),' (deg)']);

disp(['Distance To Ring Error Mean=',num2str(mean(distance_to_ring_errors)), ...
    ' STD=',num2str(std(distance_to_ring_errors)),' (mm)']);

disp(['Orientation From Optimal Error Mean=',num2str(mean(orientation_errors)), ...
    ' STD=',num2str(std(orientation_errors)),' (deg)']);

disp(['Distance From Optimal Error Mean=',num2str(mean(distances_from_desired)), ...
    ' STD=',num2str(std(distances_from_desired)),' (mm)']);
disp(['Num Samples: ',num2str(length(orientation_errors))]);
disp(['Loop Time: ',num2str(mean(abs(elapsed_time_vec))),' STD=',num2str(std(abs(elapsed_time_vec))),' (s)']);

%Removing when flags are raised
isnotflags=~any(flags_for_plot,2); %If any flag has been raised per row, notted
disp('*************STATS Without Flag Conditions***************')
disp(['Centroid View Vector Error Mean=',num2str(mean(centroid_viewingvector_angle_errors(isnotflags))), ...
    ' STD=',num2str(std(centroid_viewingvector_angle_errors(isnotflags))),' (deg)']);

disp(['Perp To Floor Error Mean=',num2str(mean(perp_to_floor_errors(isnotflags))), ...
    ' STD=',num2str(std(perp_to_floor_errors(isnotflags))),' (deg)']);

disp(['Distance To Ring Error Mean=',num2str(mean(distance_to_ring_errors(isnotflags))), ...
    ' STD=',num2str(std(distance_to_ring_errors(isnotflags))),' (mm)']);

disp(['Orientation From Optimal Error Mean=',num2str(mean(orientation_errors(isnotflags))), ...
    ' STD=',num2str(std(orientation_errors(isnotflags))),' (deg)']);

disp(['Distance From Optimal Error Mean=',num2str(mean(distances_from_desired(isnotflags))), ...
    ' STD=',num2str(std(distances_from_desired(isnotflags))),' (mm)']);
disp(['Num Samples: ',num2str(sum(isnotflags))]);
disp(['Loop Time: ',num2str(mean(abs(elapsed_time_vec(isnotflags)))),' STD=',num2str(std(abs(elapsed_time_vec(isnotflags)))),' (s)']);

%% %%%%%%%%%%%%%%%%%Displaying Results%%%%%%%%%%%%%%%%%%%%%%%%%%%
save_root='PaperFigures/';
%Orientation error
legend_labels={'Orientation From Naive','No-Go Zone Flag', 'Below Floor Flag', 'Orientation Flag', 'Proximity Flag','Joint Limit Flag'};
showErrorAndFlags(time_vector,orientation_errors,flags_for_plot,'Orientation Error (deg)',5,legend_labels,[save_root,'OrientationFromNaiveError.svg']);

%Centroid Tracking Error
legend_labels={'Viewing Vec Angle Error','No-Go Zone Flag', 'Below Floor Flag', 'Orientation Flag', 'Proximity Flag','Joint Limit Flag'};
showErrorAndFlags(time_vector,centroid_viewingvector_angle_errors,flags_for_plot,'Viewing Vec Angle Error (deg)',5,legend_labels,[save_root,'ViewingVectorAngleError.svg']);

%Perpendicular to Floor Error
legend_labels={'Parallel To Floor Angle Error','No-Go Zone Flag', 'Below Floor Flag', 'Orientation Flag', 'Proximity Flag','Joint Limit Flag'};
showErrorAndFlags(time_vector,perp_to_floor_errors,flags_for_plot,'Parallel To Floor Angle Error (deg)',5,legend_labels,[save_root,'PerpToFloorError.svg']);

%Distance to Ring Error
legend_labels={'Distance To Feature Error','No-Go Zone Flag', 'Below Floor Flag', 'Orientation Flag', 'Proximity Flag','Joint Limit Flag'};
showErrorAndFlags(time_vector,distance_to_ring_errors,flags_for_plot,'Distance To Feature Error (mm)',5,legend_labels,[save_root,'DistanceToRingError.svg']);

%Distance from desired
legend_labels={'Distance From Naive','No-Go Zone Flag', 'Below Floor Flag', 'Orientation Flag', 'Proximity Flag','Joint Limit Flag'};
showErrorAndFlags(time_vector,distances_from_desired,flags_for_plot,'Distance Error (mm)',5,legend_labels,[save_root,'DistanceFromNaiveError.svg']);


%% %Displaying the trajectories
[row,col]=size(combined_data);
ecm_T_cam_actual_trajectory=[];
ecm_T_cam_desired_trajectory=[];
ecm_T_ring_target_trajectory=[];
rotation_for_viewing=[-1,0,0;
                    0,1,0;
                    0,0,-1];
%Rotating the no-go zone points
NoGo_Points_New=[];
for i=1:5
    NoGo_Points_New=[NoGo_Points_New;NoGo_Points(i,:)*rotation_for_viewing];


end

for i=[1200:8000] %23000-27000
    %%%%%Extracting transforms
    if(i==n_p1) %New participant
        continue;
    end
    %Stored data reading in
    ecm_T_ring_target=combined_data(i,101:112);
    ecm_T_ring_target=convertCSVRowToHomo(ecm_T_ring_target);

    ecm_T_world=combined_data(i+1,114:125);
    ecm_T_world=convertCSVRowToHomo(ecm_T_world);

    ecm_T_psm3_raw=combined_data(i+1,36:47);
    ecm_T_psm3_raw=convertCSVRowToHomo(ecm_T_psm3_raw);
    

    %Computes Ideal desired Pose
    ecm_T_world_curr=combined_data(i,114:125);
    ecm_T_world_curr=convertCSVRowToHomo(ecm_T_world_curr);

    ecm_T_psm3_raw_curr=combined_data(i,36:47);
    ecm_T_psm3_raw_curr=convertCSVRowToHomo(ecm_T_psm3_raw_curr);
    
    z_i=ecm_T_psm3_raw_curr(1:3,3);
    ecm_T_psm3_desired = orientCamera(psm3_T_cam, ecm_T_ring_target, ecm_T_world_curr, z_i, 0.11, offset');
    [interpolationRequired, intermediatePose] = interpolatePose(ecm_T_psm3_desired, ecm_T_psm3_raw_curr);
    if(interpolationRequired)
        ecm_T_psm3_desired=intermediatePose;
    end

    ecm_T_cam_desired=ecm_T_psm3_desired*psm3_T_cam;
    ecm_T_psm3_actual=ecm_T_psm3_raw*psm3_T_cam;
    ecm_T_ring_target=ecm_T_ring_target;

    %Updates trajectories
    ecm_T_cam_actual_trajectory=[ecm_T_cam_actual_trajectory;(ecm_T_psm3_actual(1:3,4)'+offset)*rotation_for_viewing]; %By adding offset, we essentially put it in PSM1 reference frame
    ecm_T_cam_desired_trajectory=[ecm_T_cam_desired_trajectory;(ecm_T_cam_desired(1:3,4)'+offset)*rotation_for_viewing];
    ecm_T_ring_target_trajectory=[ecm_T_ring_target_trajectory;(ecm_T_ring_target(1:3,4)')*rotation_for_viewing];

end

%Plotting trajectories
figure;
set(gcf, 'Color', 'w');

plot3(ecm_T_cam_actual_trajectory(:,1), ecm_T_cam_actual_trajectory(:,2), ecm_T_cam_actual_trajectory(:,3), 'r', 'LineWidth', 2);    % Actual (Red)
hold on
plot3(ecm_T_cam_desired_trajectory(:,1), ecm_T_cam_desired_trajectory(:,2), ecm_T_cam_desired_trajectory(:,3), 'g', 'LineWidth', 2); % Desired (green)
plot3(ecm_T_ring_target_trajectory(:,1), ecm_T_ring_target_trajectory(:,2), ecm_T_ring_target_trajectory(:,3), 'b', 'LineWidth', 2);    % Target (blue)

%Shows the no-go zone
showNoGoZone(NoGo_Points_New(1,:),NoGo_Points_New(2,:),NoGo_Points_New(3,:),NoGo_Points_New(4,:),NoGo_Points_New(5,:));

showFloor(NoGo_Points_New(1,:),NoGo_Points_New(2,:),NoGo_Points_New(3,:),NoGo_Points_New(4,:),NoGo_Points_New(5,:),floor_offset);
set(gca, 'FontSize', 12);
xlabel('X (m)','FontSize', 16, 'FontName', 'Times', 'FontWeight', 'bold');
ylabel('Y (m)','FontSize', 16, 'FontName', 'Times', 'FontWeight', 'bold');
zlabel('Z (m)','FontSize', 16, 'FontName', 'Times', 'FontWeight', 'bold');

%Make the legend
new_colors=[1,0,0;0,1,0;0,0,1;0.6,0.6,0.6;0.3,0.3,0.3];
for j = 1:5
    hLegend(j) = plot(nan, nan, 's', 'MarkerFaceColor', new_colors(j,:), 'MarkerEdgeColor', 'k');
end

legend(hLegend, {"Actual Trajectory","Naive Trajectory","Feature Trajectory","No-Go Zone Cube","No-Go Zone Floor"}, 'Location', 'Best','FontSize', 16, 'FontName', 'Times');

set(gcf, 'Position', [100, 100, 1300, 800]);  % (left, bottom, width, height in pixels)
hold off;



%% %Displaying the Pose Plot
[row,col]=size(combined_data);

%Starts the figure
figure;
hold on
grid on;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Trajectories of Optimal, Target, and Actual');
view(3);
axis equal;


for i=[18500:50:27000] %27000
    %%%%%Extracting transforms
    if(i==n_p1) %New participant
        continue;
    end
    %Stored data reading in
    ecm_T_ring_target=combined_data(i,101:112);
    ecm_T_ring_target=convertCSVRowToHomo(ecm_T_ring_target);

    ecm_T_world=combined_data(i+1,114:125);
    ecm_T_world=convertCSVRowToHomo(ecm_T_world);

    ecm_T_psm3_raw=combined_data(i+1,36:47);
    ecm_T_psm3_raw=convertCSVRowToHomo(ecm_T_psm3_raw);
    

    %Computes Ideal desired Pose
    ecm_T_world_curr=combined_data(i,114:125);
    ecm_T_world_curr=convertCSVRowToHomo(ecm_T_world_curr);

    ecm_T_psm3_raw_curr=combined_data(i,36:47);
    ecm_T_psm3_raw_curr=convertCSVRowToHomo(ecm_T_psm3_raw_curr);
    
    z_i=ecm_T_psm3_raw_curr(1:3,3);
    ecm_T_psm3_desired = orientCamera(psm3_T_cam, ecm_T_ring_target, ecm_T_world_curr, z_i, 0.11, offset');
    [interpolationRequired, intermediatePose] = interpolatePose(ecm_T_psm3_desired, ecm_T_psm3_raw_curr);
    if(interpolationRequired)
        ecm_T_psm3_desired=intermediatePose;
    end

    ecm_T_cam_desired=invHomo(ecm_T_world)*ecm_T_psm3_desired*psm3_T_cam;
    ecm_T_psm3_actual=invHomo(ecm_T_world)*ecm_T_psm3_raw*psm3_T_cam;
    ecm_T_ring_target=invHomo(ecm_T_world)*ecm_T_ring_target;

    %Plots optimal (desired) pose
    %poseplot(ecm_T_cam_desired(1:3,1:3),ecm_T_cam_desired(1:3,4),'PatchFaceColor','b','ScaleFactor',0.01);
    poseplot(ecm_T_psm3_actual(1:3,1:3),ecm_T_psm3_actual(1:3,4),'PatchFaceColor','r','ScaleFactor',0.01);
    poseplot(ecm_T_ring_target(1:3,1:3),ecm_T_ring_target(1:3,4),'PatchFaceColor','g','ScaleFactor',0.01);



end

hold off;




%% %%%%%%%%%%%%%%%%%%%%%FUNCTIONS%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%Helper functions
%Converts CSV row to homogeneous transformation
function [frame]=convertCSVRowToHomo(csv_row)
    frame=[csv_row(4),csv_row(5),csv_row(6),csv_row(1);
        csv_row(7),csv_row(8),csv_row(9),csv_row(2);
        csv_row(10),csv_row(11),csv_row(12),csv_row(3);
        0,0,0,1];
end

%Computes inverse of homogeneous transform
function T_inv = invHomo(T)
    % Extract the rotation part (3x3) and the translation part (3x1)
    R = T(1:3, 1:3);
    t = T(1:3, 4);
    
    % Compute the inverse rotation (which is the transpose of R)
    R_inv = R';
    
    % Compute the inverse translation
    t_inv = -R_inv * t;
    
    % Construct the inverse transformation matrix
    T_inv = eye(4);
    T_inv(1:3, 1:3) = R_inv;
    T_inv(1:3, 4) = t_inv;
end

%Rotation error (in degree)
function rotationError=quaternionDistance(T1,T2)
q1 = rotm2quat(T1(1:3,1:3));
q2 = rotm2quat(T2(1:3,1:3));
q_error=quatmultiply(q2,quatinv(q1));
rotationError=2*acos(q_error(1));
rotationError=rad2deg(rotationError);
end

%%%%%Error Calculation Functions

%Orientation Error
function orientation_error=orientationError(ecm_T_ring,ecm_T_psm3,psm3_T_cam)
%{
ecm_T_ring: target frame
ecm_T_psm3: camera arm position
psm3_T_cam: rigid transform from camera arm to cam
%}
ecm_T_cam=ecm_T_psm3*psm3_T_cam;

%Get ring normal vector and camera z-vector
y_ring=ecm_T_ring(1:3,2);
z_cam=ecm_T_cam(1:3,3);

%Normalize
y_ring=y_ring./norm(y_ring);
z_cam=z_cam./norm(z_cam);

%Flip z_cam orientation
z_cam=-1*z_cam;

orientation_error=angleErrorBetweenTwoVectors(z_cam,y_ring);



end

%Centroid Tracking Angle Error
function angle_error=centroidTrackingAngleError(ecm_T_ring,ecm_T_psm3,psm3_T_cam,offset)
ecm_T_cam=ecm_T_psm3*psm3_T_cam;
c=ecm_T_ring(1:3,4)-(ecm_T_cam(1:3,4)+offset);
z=ecm_T_cam(1:3,3);
angle_error=angleErrorBetweenTwoVectors(c,z);
end

function perp_angle_error=perpendicularToFloorError(ecm_T_psm3,psm3_T_cam,ecm_T_w)
ecm_T_cam=ecm_T_psm3*psm3_T_cam;
x_cam=ecm_T_cam(1:3,1);
z_w=ecm_T_w(1:3,3);
perp_angle_error=abs(90-angleErrorBetweenTwoVectors(x_cam,z_w));

end

function distance_error=distanceToRingError(ecm_T_psm3,psm3_T_cam,ecm_T_ring,offset,df)
ecm_T_cam=ecm_T_psm3*psm3_T_cam;
target_location=ecm_T_ring(1:3,4);
cam_location=ecm_T_cam(1:3,4);
dist=target_location-(cam_location+offset);
distance_error=abs(norm(dist)-df);
end

function angle_error=angleErrorBetweenTwoVectors(a,b)
angle_error=dot(a,b)./(norm(a)*norm(b));
angle_error=acos(angle_error);
angle_error=rad2deg(angle_error);
end

function distance_error=distanceFromDesired(ecm_T_psm3,ecm_T_psm3_desired,psm3_T_cam)
ecm_T_cam=ecm_T_psm3*psm3_T_cam;
ecm_T_cam_des=ecm_T_psm3_desired*psm3_T_cam;
distance_error=norm(ecm_T_cam(1:3,4)-ecm_T_cam_des(1:3,4));

end

%%%%Functions to calculate raw desired pose:
function ecm_T_psm3_desired = orientCamera(psm3_T_cam, ecm_T_R, ecm_T_w, z_i, df, offset)
    
    y_R = ecm_T_R(1:3, 2);
    y_R = y_R / norm(y_R);
    z_i = z_i / norm(z_i);
    z_w = ecm_T_w(1:3, 3) / norm(ecm_T_w(1:3, 3));
    
    sgn2 = sign(dot(z_w, y_R));
    
    % This avoids the flipping issue noted in the Python code
    z_cam = -1*y_R;
    
    x_cam = cross(z_w, z_cam) / norm(cross(z_w, z_cam));
    y_cam = cross(z_cam, x_cam) / norm(cross(z_cam, x_cam));

    ecm_R_cam = [x_cam, y_cam, z_cam];

    ecm_p_cam = ecm_T_R(1:3, 4) - df * z_cam;

    ecm_T_cam_desired = [ecm_R_cam, ecm_p_cam; 0 0 0 1];
    
    ecm_T_psm3_desired = ecm_T_cam_desired * invHomo(psm3_T_cam);
    ecm_T_psm3_desired(1:3, 4) = ecm_T_psm3_desired(1:3, 4) - offset;

end

function [interpolationRequired, intermediatePose] = interpolatePose(desiredPose, currentPose)

    interpolationRequired = false;
    
    displacement = desiredPose(1:3, 4) - currentPose(1:3, 4);
    distance = norm(displacement);
    
    direction = displacement / distance;
    intermediatePose = eye(4);

    if distance > 0.015
        interpolationRequired = true;
        intermediatePosition = currentPose(1:3, 4) + direction * distance * 0.5;
        
        intermediatePose(1:3, 1:3) = desiredPose(1:3, 1:3);
        intermediatePose(1:3, 4) = intermediatePosition;
    end

end



%%%%Display Functions

function showErrorAndFlags(time_vector,error_vec,orientation_flag_mat,error_type_ylabel,N_flags,legend_labels,savefile)
figure;
set(gcf, 'Color', 'w');
hold on
plot(time_vector, error_vec, '-k', 'LineWidth', 2);
set(gca, 'FontSize', 12);
ylabel(error_type_ylabel,'FontSize', 16, 'FontName', 'Times', 'FontWeight', 'bold');
xlabel('Time (s)')
y_max=max(error_vec);
% ylim([0,y_max+4]);

%Flag line spacing
fl_spacing=0.025*y_max;

flag_colors = [[1 0 0]; [0 1 0];[0 0 1];[0.9290 0.6940 0.1250];[1 0 1]];
flag_positions = 0- [fl_spacing:fl_spacing:N_flags*fl_spacing];  % Place flags above the error signal
ylim([min(flag_positions)-fl_spacing,y_max]);
xlim([0,max(time_vector)+1]);

%Iterate over each flag
for i=1:N_flags
    flag_data=orientation_flag_mat(:,i);
    flag_diff=diff([0;flag_data;0]);
    flag_start=find(flag_diff==1);
    flag_end=find(flag_diff==-1);
    
    %Plot horizontal line segments when flags are triggered
    for j=1:length(flag_start)
        x_start_idx=flag_start(j);
        if(x_start_idx<0)
            x_start_idx=0;
        end
        x_start=time_vector(x_start_idx);

        x_end_idx=flag_end(j);
        if(x_end_idx>length(time_vector))
            x_end_idx=length(time_vector);
        end
        x_end=time_vector(x_end_idx);
        

        %Horizontal line for flag segment
        plot([x_start,x_end],[flag_positions(i),flag_positions(i)], ...
            '-','Color',flag_colors(i,:),'LineWidth',2.5);
        %Circles at start/end
        scatter([x_start,x_end],[flag_positions(i),flag_positions(i)], ...
            20,flag_colors(i,:),'filled');

    
    end


end

new_colors=[[0,0,0];flag_colors];
for j = 1:(N_flags+1)
    hLegend(j) = plot(nan, nan, 's', 'MarkerFaceColor', new_colors(j,:), 'MarkerEdgeColor', 'k');
end

legend(hLegend, legend_labels, 'Location', 'NorthWest','FontSize', 16, 'FontName', 'Times');

set(gcf, 'Position', [100, 100, 1300, 800]);  % (left, bottom, width, height in pixels)

hold off
saveas(gcf, savefile);


end


function showNoGoZone(A,B,C,D,E)
%Face colours
face_color=[0.7,0.7,0.7];
face_alpha=0.3;

% Compute the base edges
AB = B - A;
BC = C - B;
AC = C - A;
AD = D - A;
CD = D - C;
DA = A - D;

% Normal vector of the base plane
n_base = cross(AB, AC);
n_base = n_base / norm(n_base);

% Ensure the normal vector is pointing outwards
if dot(n_base, A) > dot(n_base, A + [0, 0, -1])
    n_base = -n_base;
end

% Compute the height vector
height_vector = dot((E - A), n_base) * n_base;

% Compute top face vertices
A_top = A + height_vector;
B_top = B + height_vector;
C_top = C + height_vector;
D_top = D + height_vector;

% Vertices for patch function
vertices = [A; B; C; D; A_top; B_top; C_top; D_top];

% Define faces (each row represents a face of the cube)
faces = [
    1 2 3 4; % Base
    5 6 7 8; % Top
    1 2 6 5; % Side (AB)
    2 3 7 6; % Side (BC)
    3 4 8 7; % Side (CD)
    4 1 5 8  % Side (DA)
];

patch('Vertices', vertices, 'Faces', faces, 'FaceColor', face_color, 'FaceAlpha', face_alpha, 'EdgeColor', 'k');


end


function showFloor(A,B,C,D,E,floor_offset)
%Face colours
face_color=[0.3,0.3,0.3];
face_alpha=0.7;
% Define expansion factor for visualization (increase to extend more)
floor_expansion_factor = 0.1; 

% Compute the base edges
AB = B - A;
AC = C - A;

% Normal vector of the base plane
n_base = cross(AB, AC);
n_base = n_base / norm(n_base);

% Ensure the normal vector is pointing outwards
if dot(n_base, A) > dot(n_base, A + [0, 0, -1])
    n_base = -n_base;
end

% Compute the height vector
height_vector = dot((E - A), n_base) * n_base;
 

% Compute the base plane vectors
base_vector1 = (B - A); % Edge along one side
base_vector2 = (D - A); % Edge along another side

% Normalize these vectors
base_vector1 = base_vector1 / norm(base_vector1);
base_vector2 = base_vector2 / norm(base_vector2);

% Extend the floor outward in both directions along these base vectors
expansion1 = floor_expansion_factor * base_vector1;
expansion2 = floor_expansion_factor * base_vector2;

A_floor = A + (floor_offset * height_vector / norm(height_vector))- expansion1 - expansion2;
B_floor = B + (floor_offset * height_vector / norm(height_vector))+ expansion1 - expansion2;
C_floor = C + (floor_offset * height_vector / norm(height_vector))+ expansion1 + expansion2;
D_floor = D + (floor_offset * height_vector / norm(height_vector))- expansion1 + expansion2;

% Store vertices for patch function
floor_vertices = [A_floor; B_floor; C_floor; D_floor];

% Define faces (only one face for the floor)
floor_faces = [1 2 3 4];


patch('Vertices', floor_vertices, 'Faces', floor_faces, 'FaceColor', face_color, 'FaceAlpha', face_alpha, 'EdgeColor', 'k');


end

function jerk_mag=computeJerkMag(position_array,time_vector)
    valid_indices=~any(isnan(position_array), 2) & ~isnan(time_vector);
    position_array=position_array(valid_indices,:);
    position_array(1,:)=[];
    time_vector=time_vector(valid_indices);
    
    time_delta=diff(time_vector);
    N=length(time_delta); 

    jerk_vec=[];

    %Computes jerk using central difference formula for third derivative
    for i=[3:N-2]
        avg_timedelta=mean(time_delta(i-2:i+2));
        jerk=(position_array(i+2,:)-2*position_array(i+1,:)+2*position_array(i-1,:)-position_array(i-2,:))./(2*(avg_timedelta^3));
        jerk_vec=[jerk_vec;jerk];

    end
    jerk_mag=vecnorm(jerk_vec,2,2);
end

