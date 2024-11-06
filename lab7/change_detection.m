%% Task 1: work on the videosurveillance sequence using a simple background obtained as an 
% average between two empty frames

% load two empty images
B1 = double(rgb2gray(imread('EmptyScene01.jpg')));
B2 = double(rgb2gray(imread('EmptyScene02.jpg')));

% compute a simple background model
B = 0.5*(B1 + B2);

% load each image in the sequence, perform the change detection
% show the frame, the background and the binary map
% Observe how the results change as you vary the threshold

tau = 60;

FIRST_IDX = 250; %index of first image
LAST_IDX = 320; % index of last image

for t = FIRST_IDX : LAST_IDX
    
    filename = sprintf('optical_flow/Data/videosurveillance/frame%4.4d.jpg', t);
    It = imread(filename);
    Ig = rgb2gray(It);
    
    Mt = (abs(double(Ig) - B) > tau);
    
    subplot(1, 3, 1), imshow(It);
    subplot(1, 3, 2), imshow(uint8(B));
    subplot(1, 3, 3), imshow(uint8(Mt*255));
    pause(0.5)

end

%% Task 2: working again on the videosurveillance sequence, use now a background model based 
% on running average to incorporate scene changes

% Let's use the first N  frames to initialize the background

FIRST_IDX = 250; %index of first image
LAST_IDX = 320; % index of last image

N = 5;

filename = sprintf('optical_flow/Data/videosurveillance/frame%4.4d.jpg', FIRST_IDX);
B = double(rgb2gray(imread(filename)));
for t = FIRST_IDX+1 : FIRST_IDX + N-1
    
    filename = sprintf('optical_flow/Data/videosurveillance/frame%4.4d.jpg', t);
    B = B + double(rgb2gray(imread(filename)));
    
end

B = B / N;

% Play with these parameters
TAU = 40;
TAU1 = 2;
ALPHA = 0.9;

% Now start the change detection while updating the background with the
% running average. For that you have to set the values for TAU and ALPHA
Bprev = B;
prevIt = imread(filename);

for t = FIRST_IDX+N : LAST_IDX
    
    filename = sprintf('optical_flow/Data/videosurveillance/frame%4.4d.jpg', t);
    
    It = imread(filename);
    Ig = rgb2gray(It);
    
    Mt = (abs(double(Ig) - Bprev) > TAU);
    Dt = abs(double(It) - double(prevIt));
    
    % Implement the background update as a running average
    if(Dt>TAU1)
        Bcurr = Bprev;
    else
        Bcurr = ((1-ALPHA)*Bprev + ALPHA*double(rgb2gray(imread(filename))));
    end
    %keyboard
    subplot(1, 3, 1), imshow(It);
    subplot(1, 3, 2), imshow(uint8(Bcurr));
    subplot(1, 3, 3), imshow(uint8(Mt*255));
    pause(0.1)
    Bprev = Bcurr;
    prevIt = It;
end


%% Task 3: Repeat the above experiment with the sequence frames_evento1 observing what happens as you change 
% the parameters TAU and ALPHA

% Let's use the first N  frames to initialize the background

FIRST_IDX = 4728; %index of first image
LAST_IDX = 6698; % index of last image

N = 5;

filename = sprintf('frames_evento1/frame%4.4d.jpg', FIRST_IDX);
B = double(rgb2gray(imread(filename)));
for t = FIRST_IDX+1 : FIRST_IDX + N-1
    
    filename = sprintf('frames_evento1/frame%4.4d.jpg', t);
    B = B + double(rgb2gray(imread(filename)));
    
end

B = B / N;

% Play with these parameters
TAU = 45;
TAU1 = 2;
ALPHA = 0.2;

% Now start the change detection while updating the background with the
% running average. For that you have to set the values for TAU and ALPHA
Bprev = B;
prevIt = imread(filename);

for t = FIRST_IDX+N : LAST_IDX
    
    filename = sprintf('frames_evento1/frame%4.4d.jpg', t);
    
    It = imread(filename);
    Ig = rgb2gray(It);
    
    Mt = (abs(double(Ig) - Bprev) > TAU);
    Dt = abs(double(It) - double(prevIt));
    
    % Implement the background update as a running average
    if(Dt>TAU1)
        Bcurr = Bprev;
    else
        Bcurr = ((1-ALPHA)*Bprev + ALPHA*double(rgb2gray(imread(filename))));
    end
    
    %keyboard
    subplot(1, 3, 1), imshow(It);
    subplot(1, 3, 2), imshow(uint8(Bcurr));
    subplot(1, 3, 3), imshow(uint8(Mt*255));
    pause(0.0001)
    Bprev = Bcurr;
    prevIt = It;
    
end


%% Task 4: Design a simple tracking system according to the following guidelines
% a. Initialize the background model 

% WE ADDED clc; AND clear; AT THE BEGINNING OF THE FILE TO 

% Let's use the first N  frames to initialize the background

FIRST_IDX = 250; %index of first image
LAST_IDX = 320; % index of last image

N = 5;

filename = sprintf('optical_flow/Data/videosurveillance/frame%4.4d.jpg', FIRST_IDX);
B = double(rgb2gray(imread(filename)));
for t = FIRST_IDX+1 : FIRST_IDX + N-1
    
    filename = sprintf('optical_flow/Data/videosurveillance/frame%4.4d.jpg', t);
    B = B + double(rgb2gray(imread(filename)));
    
end

B = B / N;

% b. Initialize the tracking history to empty
clear trackingHistory;
init = struct('x',0,'y',0);
trackingHistory(1:66,1) = init;

% b. At each time instant
%       i. Apply the change detection to obtain the binary map
%      ii. Update the background model
%     iii. Identify the connected components in the binary map (see e.g.
%          the matlab function bwconncomp)
%      iv. Try to associate each connected component with a previously seen
%          target
% Hint 1 - It would be good to keep track of the entire trajectory and produce a visualization 
% that can be done either frame by frame (so you should see the trajectory built
% incrementally) or only at the end (in this case you will see the entire final trajectory)
% Hint 2 - How to decide that a trajectory is closed?


TAU = 25;
TAU1 = 2;
ALPHA = 0.9;

% Now start the change detection while updating the background with the
% running average. For that you have to set the values for TAU and ALPHA
Bprev = B;
prevIt = imread(filename);
counter = 1;
threshold = 50;
check = 0;


for t = FIRST_IDX+N : LAST_IDX
    
    filename = sprintf('optical_flow/Data/videosurveillance/frame%4.4d.jpg', t);
    
    It = imread(filename);
    Ig = rgb2gray(It);
    
    Mt = (abs(double(Ig) - Bprev) > TAU);
    Dt = abs(double(It) - double(prevIt));
    Bw = imfill(Mt*255, "holes");
    se = [0 1 0; 1 1 1; 0 1 0];
    Bw = imdilate(Bw, se);
   
    
    % Implement the background update as a running average
    if(Dt>TAU1)
        Bcurr = Bprev;
    else
        Bcurr = ((1-ALPHA)*Bprev + ALPHA*double(rgb2gray(imread(filename))));
    end

    %trackingHistory{t-254,1} = bwconncomp(Bw);

    temp = regionprops(bwlabel(Bw), 'BoundingBox', 'Centroid', 'Area');
    
    %keyboard
    subplot(1, 3, 1), imshow(It);
    hold on
    for i = 1 : length(temp)
        if temp(i).Area > 1000
            bb = temp(i).BoundingBox;
            bc = temp(i).Centroid;
            bc_struct = struct('x', bc(1), 'y', bc(2));
            
            
            if t == FIRST_IDX+N
                trackingHistory(t-254,counter) = bc_struct;
                counter = counter + 1;
             
            else
                oldTrackingHistory = trackingHistory;
                sizeTH = size(oldTrackingHistory);
                for j = 1 : sizeTH(2)
                     
                    if abs(trackingHistory(t-255, j).x - bc_struct.x) < threshold && abs(trackingHistory(t-255, j).y - bc_struct.y) < threshold
                        trackingHistory(t-254, j) = bc_struct;
                        check = 1;
                    end
                end 

                    if check == 0
                        tempMatrix(1:66,1) = init;
                        tempMatrix(t-254, 1) = bc_struct;
                        trackingHistory = cat(2, trackingHistory, tempMatrix);
                    end    

            end

            rectangle('Position',bb, 'EdgeColor','r','LineWidth',3);
        end
        check = 0;
    end
    sizeTH = size(trackingHistory);
    for j = 1 : sizeTH(2)
        if trackingHistory(t-254,j).x ~= 0 && trackingHistory(t-254,j).y ~= 0
            % Plot the point
            plot(trackingHistory(t-254,j).x, trackingHistory(t-254,j).y, 'o')

            % Add a label to the point
            msg = sprintf('Object %d', j);
            text(trackingHistory(t-254,j).x, trackingHistory(t-254,j).y, msg)
        end
    end
    subplot(1, 3, 2), imshow(uint8(Bcurr));
    subplot(1, 3, 3), imshow(uint8(Bw));
    pause(0.1)
    Bprev = Bcurr;
    prevIt = It;
end  

figure, imagesc(It), title('Final trajectory');
hold on


%we are plotting only the trajectory of the first man because it is the
%only object detected that doesn't touch any other so the trajectory is
%precise until the end.

for col = 1 : sizeTH(2)
    for row = 1 : sizeTH(1)
        if trackingHistory(row,col).x ~= 0 && trackingHistory(row,col).y ~= 0
            if col == 1
                plot(trackingHistory(row,col).x, trackingHistory(row,col).y, '*r' )
            end
        end
    end
end





