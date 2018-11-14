function robot_draw
% Draw the static Scara Robot

r = 0.2; % radius
h = 5; % height
n = 10; % number of vertices in circle
dtheta = 2*pi/n;
theta = 0; 

% create vertices
bottom_verts = zeros(n,3);
top_verts = bottom_verts;
for i = 1:n  
    bottom_verts(i,:) = [r*cos(theta), r*sin(theta), 0];
    top_verts(i,:) = [r*cos(theta), r*sin(theta), h];
  
    theta = theta + dtheta;
end
     
vertices = [bottom_verts; top_verts];

top_face = 1:n;
bottom_face = n+1:2*n;

side_faces = zeros(n);
for i = 1:n-1
    side_faces(i,1:4) = [i, i+1, i+n+1, i+n];
    side_faces(i,5:n) = ones(1,n-4)*(i+n);
end
side_faces(n,1:4) = [2*n, n+1, 1, n];
side_faces(n,5:n) = ones(1,n-4)*n;

faces = [top_face; bottom_face; side_faces];
figure(1); clf;

patch('Vertices',vertices,'Faces',faces,'facecolor','b')
view([45 45])
faces
% keyboard;


end


% 
% % patch(?Vertices?,vertices_link1,?Faces?,faces_link1,?faceColor?,[RGB values])
% vertices = [0 0 0;
%             0 1 0;
%             1 1 0;
%             1 0 0;
%             0 0 1;
%             0 1 1;
%             1 1 1;
%             1 0 1];
% faces = [1 2 3 4; 5 6 7 8;
%          1 2 6 5; 2 3 7 6; 3 4 8 7; 1 4 8 5];
% 
% faces2 = [2 3 4 1 5 6];
     