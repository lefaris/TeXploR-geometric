% Agile Robotics Laboratory at UA
% TExploR Project
% Date: 05/23/2024
% 
% Takes in image folder as input and produces video
% 
% Usage:
% Modify image folder if necessary
% Click run!

% load the images

myFolder = 'C:\Users\lefaris.STUDENT\Box\Agile Robotics Lab\TeXploreR\Experiments\static_angle_simulation_figures_V3';
filePattern = fullfile(myFolder, '*.png');
pngFiles = dir(filePattern);  
pngFiles = natsortfiles(pngFiles);
nfiles = length(pngFiles);    % Number of files found
for ii=1:nfiles
   baseFileName = pngFiles(ii).name;
   fullFileName = fullfile(myFolder, baseFileName);
   currentimage = imread(fullFileName);
   images{ii} = currentimage;
end

% create the video writer with 1 fps
 writerObj = VideoWriter('supplemental_static_equilibrium_path_simulations.avi');
 writerObj.FrameRate = 1;
 % set the seconds per image
 secsPerImage = ones(111);
 % open the video writer
 open(writerObj);
 % write the frames to the video
 for u=1:length(images)
     % convert the image to a frame
     frame = im2frame(images{u});
     %for v=1:secsPerImage(u) 
         writeVideo(writerObj, frame);
     %end
 end
 % close the writer object
 close(writerObj);