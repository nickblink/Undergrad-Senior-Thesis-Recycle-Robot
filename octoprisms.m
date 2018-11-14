% testing to see if I can get some 3-D graphics working
close all

n = 8;
t = linspace(0,2*pi,n)';

octocenters = [-7 -2; 5 5; -1 4; 5 0; -3 -7];
ocs = octocenters;
[num_octos, ~] = size(octocenters);

% 16 for 2 octogons worth of vertices, 3 for 3-D space
all_vertices = zeros(16,3,num_octos);
for j = 1:num_octos
    verticesbottom = cat(2,cos(t)+ocs(j,1),sin(t)+ocs(j,2),zeros(n,1));
    verticestop = cat(2,cos(t)+ocs(j,1),sin(t)+ocs(j,2),ones(n,1));
    all_vertices(:,:,j) = cat(1,verticesbottom,verticestop);
end

top_bottom_faces = [1 2 3 4 5 6 7 8; 9 10 11 12 13 14 15 16];
% we have to repeat redundant coordinates to make faces a matrix
side_faces = ...
    [1 2 10 9 1 1 1 1;
     2 3 11 10 2 2 2 2;
     3 4 12 11 3 3 3 3;
     4 5 13 12 4 4 4 4;
     5 6 14 13 5 5 5 5;
     6 7 15 14 6 6 6 6;
     7 8 16 15 7 7 7 7;
     8 1 9 16 8 8 8 8];
faces = cat(1,top_bottom_faces,side_faces);
[num_faces, ~] = size(faces);

hold on
axis equal

% draw the ground
patch('Vertices',[10 10 0; 10 -10 0; -10 -10 0; -10 10 0], ...
    'Faces',[1 2 3 4],'FaceColor','b');

% draw the octos
for j = 1:num_octos
    patch('Vertices',all_vertices(:,:,j),'Faces',faces,...
        'FaceVertexCData',hsv(num_faces),'FaceColor','flat')
end

view([45 45])




  

