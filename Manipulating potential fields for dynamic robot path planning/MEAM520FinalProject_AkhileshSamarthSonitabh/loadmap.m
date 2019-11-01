function map = loadmap(filename)
% Load a map from disk.

fId = fopen(filename,'r');
text = textscan(fId,'%s %f %f %f %f %f %f','CommentStyle','#');

% load element names
elements = text{1};

% load element data
data_msg = ['Bad data in file: ', filename,...
    '. Make sure the fields for each element are filled out, ',...
    'are the correct datatype, and are properly formatted.'];
try
    data = horzcat(text{1,2:7});
catch err
    cause = MException('loadmap:bad_file_data', data_msg);
    err = addCause(err, cause);
    rethrow(err);
end

% missing elements become NaNs
if any(isnan(data(:)))
    error(data_msg);
end

% set obstacles
obstacles = [];
for i = 1:size(elements,1)
    if strcmp(elements{i},'boundary')
        boundary = data(i,:);
    elseif strcmp(elements{i},'block')
        obstacles = [obstacles; data(i,:)];
    else
        warning('Unknown element type "%s" in %s', elements{i}, filename);
    end
end

map.obstacles = obstacles;
map.boundary = boundary;
