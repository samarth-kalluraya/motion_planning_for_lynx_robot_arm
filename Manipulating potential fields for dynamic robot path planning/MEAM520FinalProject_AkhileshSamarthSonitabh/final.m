% Creating camera object
c = webcam("GENERAL - UVC");                         

% Set to lowest resolution
res = c.AvailableResolutions;
res = res(size(res,2));
res = res{1};
c.Resolution = res;

% Opening camera window
preview(c);
pause;                      % waiting for keypress to capture image

% capture
img = snapshot(c);

% close camera
clear('c');

%%%%%testing%%%%%%
%img = imread('I2.png');

% display picture
figure(1);
subplot(1,2,1);
image(img);
rt = img(:,:,1)>90;
gt = img(:,:,2)>90;
bt = img(:,:,3)>90;

rt = 255.*rt;
gt = 255.*gt;
bt = 255.*bt;
t = cat(3,rt,gt,bt);
% subplot(1,2,2);
% image(t);
% impixelinfo;

%% Colour Segmentation
img = t;

img = im2double(img);               % change uint8 to double
[r,c,p] = size(img);                % extract size of true colour image
Rimg = squeeze(img(:,:,1));         % get the R level
Gimg = squeeze(img(:,:,2));         % get the G level
Bimg = squeeze(img(:,:,3));         % get the B level

%Rimg = img(:,:,1)>0.7;

Rimg_bin = im2bw(Rimg,graythresh(Rimg));            % change to binary for R 
Gimg_bin = im2bw(Gimg,graythresh(Gimg));            % change to binary for G
Bimg_bin = im2bw(Bimg,graythresh(Bimg));            % change to binary for B
img_bin = imcomplement(Rimg_bin&Gimg_bin&Bimg_bin); % get complementary binary image

se = strel('square',10);                             % define strel for cleaning the image
img_clean = imopen(img_bin,se);                     % cleaning small size dots from image (noise)
img_clean = imfill(img_clean,'holes');              % filling holes introduced due to noise
img_clean = imclearborder(img_clean);               % clean border of image

[labels,numlabels] = bwlabel(img_clean);            % get the pixel label and an array for label values
 
Rl = zeros(r,c);                                    % declaring R label    
Gl = zeros(r,c);                                    % declaring G label
Bl = zeros(r,c);                                    % declaring B label

% filling position for a match
for i = 1:numlabels
    Rl(labels==i) = median(Rimg(labels==i));
    Gl(labels==i) = median(Gimg(labels==i));
    Bl(labels==i) = median(Bimg(labels==i));
end

pic = cat(3,Rl,Gl,Bl);                        % forming final image 

%% Finding order

subplot(1,2,2);
image(pic);

Ronly = Rl - Gl - Bl;                       
red = (Ronly>0);                              % Red colour matrix
Gonly = Gl - Rl - Bl;
green = (Gonly>0);                            % Green colour matrix  
Bonly = Bl - Gl - Rl;
blue = (Bonly>0);                             % Blue colour matrix

% yellow = Rl & Gl;                             % Yellow colour matrix
% pink = Rl & Bl;                               % Pink colour matrix

[rr,rc] = find(red);                          % finding indices of non-zero elements for red matrices
R = [rr rc];

[gr,gc] = find(green);                        % finding indices of non-zero elements for green matrices
G = [gr gc];

[br,bc] = find(blue);                         % finding indices of non-zero elements for blue matrices
B = [br bc];

% [yr,yc] = find(yellow);                       % finding indices of non-zero elements for yellow matrices
% Y = [yr yc];
% 
% [pr,pc] = find(pink);                         % finding indices of non-zero elements for pink matrices  
% P = [pr pc];

M = struct(...
     'value', {median(R(:,2)), median(G(:,2)), median(B(:,2)), median(Y(:,2)), median(P(:,2))},...
     'colour', {'red', 'green', 'blue'});         % struct with median values of corresponding colours
 
Mtab = struct2table(M);                        % Convert to table
Ms = sortrows(Mtab,'value');                   % Sorting the table according to median values
disp(Ms.colour);                               % display order of colours