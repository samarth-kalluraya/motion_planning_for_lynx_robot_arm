function f = plotmap(map, ax)


if nargin < 2
    ax = gca;
end

X = [0 0 0 0 0 1; 1 0 1 1 1 1; 1 0 1 1 1 1; 0 0 0 0 0 1];
Y = [0 0 0 0 1 0; 0 1 0 0 1 1; 0 1 1 1 1 1; 0 0 1 1 1 0];
Z = [0 0 1 0 0 0; 0 0 1 0 0 0; 1 1 1 0 1 1; 1 1 1 0 1 1];

hold on;
for i = 1:size(map.obstacles,1)
    box_corners = reshape(map.obstacles(i,:), [3 2])';
    centroid = mean(box_corners);
    side_length = abs(box_corners(1,:) - box_corners(2,:));
    Xi = side_length(1)*(X-0.5) + centroid(1);
    Yi = side_length(2)*(Y-0.5) + centroid(2);
    Zi = side_length(3)*(Z-0.5) + centroid(3);
    f(i,:) = fill3(Xi, Yi, Zi, 'b', 'FaceAlpha', 0.5);
end
hold off;

