close all
clear all


plotSearch = true;


% read in files
search_tree_raw = csvread('search_tree.txt');
path_raw = csvread('output_path.txt');
setup_raw = csvread('setup.txt');
obst = csvread('obstacles_proj.txt');
robot_raw = csvread('proj_robot.txt');

start = setup_raw(1,:);
goal  = setup_raw(2,:);

% a bit of data processing for faster plotting
search_tree = nan(3*size(search_tree_raw, 1), 3);
 
search_tree(1:3:end-2, 1) = search_tree_raw(:, 2); %x1
search_tree(2:3:end-1, 1) = search_tree_raw(:, 6); %x2
search_tree(1:3:end-2, 2) = search_tree_raw(:, 3); %y1
search_tree(2:3:end-1, 2) = search_tree_raw(:, 7); %y2
search_tree(1:3:end-2, 3) = search_tree_raw(:, 4); %z1
search_tree(2:3:end-1, 3) = search_tree_raw(:, 8); %z2



fig = figure(1);
set(gcf, 'Position', [300, 100, 950, 800])
view([45-37.5,30]);
xlabel('X'); ylabel('Y'); zlabel('Z');
hold on

% vidObj = VideoWriter('RRT_vid.avi');
% vidObj.FrameRate = 60;
% open(vidObj);

% Plot obtacles
[x,y,z] = sphere;
for ii = 1:length(obst(:,1))
    surf(x*obst(ii,4)+obst(ii,1),y*obst(ii,4)+obst(ii,2),z*obst(ii,4)+obst(ii,3)) % centered at (0,1,-3)
end
g = surf(x*goal(4)+goal(1),y*goal(4)+goal(2),z*goal(4)+goal(3));
g.EdgeColor = 'none';
plot3(start(1),start(2),start(3),'rx','MarkerSize',24,'LineWidth',3)
daspect([1,1,1])
axis([0 100 0 100 0 100])


if plotSearch == true
    for n = 1:3:length(search_tree(:,2))
        plot3(search_tree(n:n+2, 1),search_tree(n:n+2, 2), search_tree(n:n+2, 3), 'b.-', 'LineWidth', 0.5,'MarkerSize',8);
        pause(0.001);
        if ~ishandle(fig) 
            break
        end
        currFrame = getframe;
        writeVideo(vidObj,currFrame);
    end
    
    % Plot Final Path
    plot3(path_raw(:,2), path_raw(:,3),path_raw(:,4), 'g:', 'LineWidth', 4);
%     currFrame = getframe;
%     writeVideo(vidObj,currFrame);
    
    % Plot Robot Orientation
    for ii = 1:length(path_raw(:,1))
        psi = path_raw(ii,5);
        theta = path_raw(ii,6);
        Rpsi = [cos(psi) -sin(psi) 0;
                sin(psi) cos(psi) 0;
                0 0 1];
        Rth = [cos(theta) 0 sin(theta)
               0 1 0;
               -sin(theta) 0 cos(theta)];
        rob_att = Rpsi'*Rth'*robot_raw'+ path_raw(ii,2:4)';
        plot3(rob_att(1,:),rob_att(2,:),rob_att(3,:),'m+','MarkerSize',2);
    end
    hold off
end

% close(vidObj);


