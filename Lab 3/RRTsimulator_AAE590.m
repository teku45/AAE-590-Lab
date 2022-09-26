% clear,clc; clear all; close all; fclose all; format long;
function[path] = RRTsimulator
close all
%   Function: Simulate an obstacle avoiding problem with a modified RRT
%             algorithm which returns the original path, the refined path,
%             and the computation time of both.

% Define the amount of simulation that is needed and initialize output
% parameters
simulation = 1;
output.path = [];
output.length = [0];

for i = 1:simulation
    tic
    % Define starting & ending points
    starting = [-1.2,0];
    ending = [1.8,0];

    % Evaluate & illustrate the world
    [obstacle,conic,polygon] = obstacle_eval();
    [xymax,xymin] = obstacle_plot(obstacle,conic,polygon,starting,ending);
    axis image
    axis([-3.9 15.9 -2.45 xymax(1,2)])  % Re-define axis for comparision consistency purpose

    % Solve the collision avoidance problem using RRT. Then, plot the
    % result tree and feasible path, and report computation time.
    [path,tree,connection] = RRT_eval(obstacle,starting,ending,xymax,xymin);
    plot1 = plot(path(:,1),path(:,2),':r','LineWidth',2.5);
    time1 = toc;
    fprintf('The computation time for RRT is %f sec.\n',time1)

    % Refine the exist path due to limitation of vehicle
    turning = 30;                       % limitation of turning angle (deg)
    [path,pathlength] = path_opt(path,obstacle,xymax,xymin,turning);
    plot(path(:,1),path(:,2),'LineWidth',2.5)
    time2 = toc;
    fprintf('The computation time of refined process is %f sec.\n',time2)
    fprintf('The total distance of traveling is %f\n',pathlength(1))

    output(i).time1 = time1;
    output(i).time2 = time2;
    output(i).path = path;
    output(i).length = pathlength(1);    
end
save(['Example1_7_',num2str(simulation),'.mat'],'output')
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
function [obstacle,conic,polygon] = obstacle_eval()
%   Function: Create obstacles (circle, ellipse, or ploygon shapes) in the
%             environment.

% Define obstacle with shape of circle or ellipse
% conic.name = shape and numbering
% conic.center = center of cirle or ellipse [x,y]
% conic.ab = radius of circle (a=b), or semi-major and semi-minor axis of
%            ellipse
conic = [];
conic(1).name = 'circle1';
conic(1).center = [0,0];
conic(1).ab = (0.3+0.08)./sqrt([1,1]);
% conic(2).name = 'circle2';
% conic(2).center = [1,8];
% conic(2).ab = 1.5./sqrt([1,1]);
% conic(3).name = 'ellipse1';
% conic(3).center = [6,8];
% conic(3).ab = 2.5./sqrt([1,3]);
% conic(4).name = 'ellipse2';
% conic(4).center = [3,1];
% conic(4).ab = 2.5./sqrt([1,3]);
% conic(5).name = 'ellipse3';
% conic(5).center = [2.5,5];
% conic(5).ab = 2./sqrt([2,1]);
% conic(6).name = 'ellipse4';
% conic(6).center = [8,2];
% conic(6).ab = 2./sqrt([2,1]);
% conic(7).name = 'ellipse5';
% conic(7).center = [8,10.5];
% conic(7).ab = 1.5./sqrt([2,1]);
% conic(8).name = 'ellipse6';
% conic(8).center = [7,4.5];
% conic(8).ab = 2./sqrt([1,2]);

% Define obstacle with shape of ploygon
% polygon.name = shape and numbering
% ploygon.vertex = vertex of ploygon [x,y]
polygon = [];


% Evaluate the edge(s) of obstacle with intermediate points and margin
obstacle.radius = 0.5E-1;         % margin between object and obstacle
density = 100;                   % density of intermediate points for cirle and ellipse
substep = obstacle.radius;      % substep of intermediate points for edges of polygon
obstacle.coordinate = [];
%
t=0:pi/density:2*pi;
if ~isempty(conic)
    for index = 1:size(conic,2)
        x = conic(index).center(1,1)+conic(index).ab(1,1)*cos(t);
        y = conic(index).center(1,2)+conic(index).ab(1,2)*sin(t);
        obstacle.coordinate = [obstacle.coordinate;x',y'];
    end
end
%
if ~isempty(polygon)
    for index = 1:size(polygon,2)
        pos = [polygon(index).vertex;polygon(index).vertex(1,:)];
        vec = pos(2:end,:)-pos(1:end-1,:);
        for jndex = 1:size(vec,1)
            dist = norm(vec(jndex,:));
            div = ceil(dist/substep);
            x = linspace(pos(jndex,1),pos(jndex+1,1),div)';
            y = linspace(pos(jndex,2),pos(jndex+1,2),div)';
            obstacle.coordinate = [obstacle.coordinate;x(1:end-1),y(1:end-1)];
        end
    end
end
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
function [xymax,xymin] = obstacle_plot(obstacle,conic,polygon,starting,...
                                       ending)
%   Function: Illustrate the environment with max & min coordinates,
%             obstacles, and boundary.

RRTfigure = figure;
set(RRTfigure,'Name','RRT Simulator')
set(RRTfigure,'NumberTitle','off')
set(RRTfigure,'Position',[500 50 900 900])
hold on

% Plot conic obstacles
t=0:pi/100:2*pi;
for index = 1:size(conic,2)
    x = conic(index).center(1,1)+conic(index).ab(1)*cos(t);
    y = conic(index).center(1,2)+conic(index).ab(2)*sin(t);
    fill(x,y,'w','EdgeColor','none')
    plot(x,y,'g')
end

% Plot ploygon obstacles
for jndex = 1:size(polygon,2)
    x = polygon(jndex).vertex(:,1);
    y = polygon(jndex).vertex(:,2);
    fill(x,y,'k','EdgeColor','none')
    plot(x,y,'g')
end

% Plot starting & ending point 
plot(starting(1,1),starting(1,2),'ob')
plot(ending(1,1),ending(1,2),'*b')
%
xymax = max([obstacle.coordinate;starting;ending],[],1)+0.1;
xymin = min([obstacle.coordinate;starting;ending],[],1)-0.1;
axis image
axis([xymin(1,1)-obstacle.radius xymax(1,1)+obstacle.radius...
      xymin(1,2)-obstacle.radius xymax(1,2)+obstacle.radius]);
grid off
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
function col = collision_eval(node1,node2,obstacle,xymax,xymin)
%   Function: Evaluate 1. if the new sample (node2) locates within the
%             workspace, 2. if the new sample (node2) locates within the
%             boundary of obstacles, and 3. if collision occurs while node1
%             connects to new sample (node2)

% Initialize collision check
col = 0;

% Check if the new sample (node2) locates within the workspace. Return true
% when new sample (node2) is outside the workspace
if node2(1,1)>xymax(1,1)||node2(1,2)>xymax(1,2)||...
   node2(1,1)<xymin(1,1)||node2(1,2)<xymin(1,2)
    col = 1;
    return
end

% Check of if the new sample (node2) locates within the boundary of
% obstacles. Return true when the width between new sample (node2) and
% obstacles is less than the boundary
if isempty(node1)
    point = node2;
    for index = 1:size(obstacle.coordinate,1)
        if norm(point-obstacle.coordinate(index,:))<obstacle.radius
            col = 1;
            return
        end
    end
end

% Check if collision occurs while node1 connects to new sample (node2).
% Return ture when obstacle exist on the segment between node1 and new
% sample (node2)
if ~isempty(node1)
    dist = norm(node2-node1);
    div = ceil(dist/obstacle.radius);
    for jndex = 0:1/(div-1):1
        point = (1-jndex)*node1+jndex*node2;
        for kndex = 1:size(obstacle.coordinate,1)
            if norm(point-obstacle.coordinate(kndex,:))<obstacle.radius
                col = 1;
                return
            end
        end
    end
end
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
function [costdist,costindex] = cost(tree,node2,ending)
%   Function: Determine and indentify the node from exist tree that
%             produces the minimum cost to the new sample.

% costdist = summation of distance from new sample to tree and ending that needs to be minimized
% costindex = the related index of node which returns the minimum cost from the tree
vec = (ones(size(tree,1),1)*node2-tree);
dist = sqrt(vec(:,1).^2+vec(:,2).^2);
[costdist,costindex] = min(dist+norm(ending-node2));
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
function [path,tree,connection] = RRT_eval(obstacle,starting,ending,...
                                           xymax,xymin)
%   Function: Determine a tree that explores the environment and evaluate a
%             feasible path from starting to ending by using RRT algorithm.

% Initialize parameters
step = 0.3;                         % step = distance from ending that potentially could connect to node directly
path = [];                          % path = feasible solution that extracts from tree
tree = [starting];                  % tree = RRT tree which is constructed by sample node with correspondong connection index
connection = [0];                   % connection = index that assigns to new added node which indicates the connection to previous node
costtemp = norm(xymax-xymin)*1E3;   % costyemp = performance index to be minimized while sampling in the environment for RRT*

% Begin RRT algorithm
while isempty(path)

    % Random sample a node from max & min coordinates
    x = randi([floor(xymin(1,1)),ceil(xymax(1,1))])+rand(1);
    y = randi([floor(xymin(1,2)),ceil(xymax(1,2))])+rand(1);

    % Evaluate whether the new sample locates inside the workspace, and
    % whether the new sample locates in the available region
    if collision_eval([],[x,y],obstacle,xymax,xymin)
        continue
    end

    % Determine the potential of connection from new sample to tree 
    [costdist,costindex] = cost(tree,[x,y],ending);

    % Evaluate whether collision occurs while trying to connect tree and
    % new sample
    if collision_eval(tree(costindex,:),[x,y],obstacle,xymax,xymin)
        continue
    end

    % Check for performance index (only use for RRT*)
        % if costdist>costtemp
        %     continue
        % else
        %     costtemp = costdist;
        % end

    % Add in the accepted new sample to the tree with corresponding
    % connection numbering
    tree = [tree;x,y];
    connection = [connection;costindex];

    % Illustrate the tree with added node and new branch 
    plot([tree(costindex,1),x],[tree(costindex,2),y],'.m','MarkerSize',1)
    plot([tree(costindex,1),x],[tree(costindex,2),y],'-b')

    % Check if new sample is close enough to reach ending directly
    if norm(ending-[x,y])<step

        if collision_eval([x,y],ending,obstacle,xymax,xymin)
            continue
        end

        % Add in the ending as the last node of the tree
        tree = [tree;ending];
        connection = [connection;size(tree,1)-1];
        path(1,:) = [ending,connection(end,1)];

        % Recongnize the feasible path from the exist tree
        while 1
            path = [path;tree(path(end,3),:),connection(path(end,3),:)];
            if path(end,3) == 0
                return
            end
        end

    end
    
end
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
function [path,pathlength] = path_opt(path,obstacle,xymax,xymin,turning)
%   Function: Smoothing the found path to satisfiy the vehicle kinematics
%   specified by the upper bound of the heading angle turn rate, 
%   defined by 'umax' 

% Define parameters
iteration = 200;               % iteration value for smoothing process
V = 0.4;                        % constant flight velocity
umax = 1;                       % heading angle change rate

% Initialize the stacking path length for each segment
pathlength = [0];       
for index = size(path,1):-1:2
    pathlength = [pathlength(1,:)+norm(path(index-1,1:2)-...
                  path(index,1:2));pathlength];
end

% Smoothing path based on random process of picking and connecting two
% nodes if intermediate region is obstacle free
fprintf('Refining in progress..(1/4)\n')
jndex = 1;
while jndex <= iteration

    % Randomly pick two points (segments) that would be polished, and
    % ensure the length of segment2 > segment1
    segment1 = rand(1)*pathlength(1,:);
    segment2 = rand(1)*pathlength(1,:);
    if segment1 > segment2
        temp = segment2;
        segment2 = segment1;
        segment1 = temp;
    end

    % Identify the first node of each two segments, and ensure the two
    % points (segments) are not on the same interval of path
    [kndex] = find(pathlength<=segment1,1,'first');
    [lndex] = find(pathlength<=segment2,1,'first');
    if kndex == lndex
        continue
    end

    % Interpolate two points onto corresponded intervals pf path, then
    % check for collision
    ratio1 = (segment1-pathlength(kndex))/(pathlength(kndex-1)-...
              pathlength(kndex));
    ratio2 = (segment2-pathlength(lndex))/(pathlength(lndex-1)-...
              pathlength(lndex));
    point1 = (1-ratio1)*path(kndex-1,1:2)+ratio1*path(kndex,1:2);
    point2 = (1-ratio2)*path(lndex-1,1:2)+ratio2*path(lndex,1:2);
    if collision_eval(point1,point2,obstacle,xymax,xymin)
        continue
    end

    % If collision does not occur in between, insert new points and update
    % the path length
    path = [path(1:lndex-1,:);point2,0;point1,0;path(kndex:end,:)];
    pathlength = [0];
    for index = size(path,1):-1:2
        pathlength = [pathlength(1,:)+norm(path(index-1,1:2)-...
                      path(index,1:2));pathlength];
    end
    jndex = jndex+1;
end

% Smoothing from the ending along the path
fprintf('Refining in progress..(2/4)\n')
while 1
    point1 = path(3,1:2);
    point2 = path(1,1:2);
    if collision_eval(point1,point2,obstacle,xymax,xymin)
        break
    end
    path = [path(1,:);path(3:end,:)]; 
    pathlength = [0];
    for kndex = size(path,1):-1:2
        pathlength = [pathlength(1,:)+norm(path(kndex-1,1:2)-...
                      path(kndex,1:2));pathlength];
    end
end

% Smoothing from the starting along the path
fprintf('Refining in progress..(3/4)\n')
while 1
    point1 = path(end,1:2);
    point2 = path(end-2,1:2);
    if collision_eval(point1,point2,obstacle,xymax,xymin)
        break
    end
    path = [path(1:end-2,:);path(end,:)]; 
    pathlength = [0];
    for kndex = size(path,1):-1:2
        pathlength = [pathlength(1,:)+norm(path(kndex-1,1:2)-...
                      path(kndex,1:2));pathlength];
    end
end

% Eliminate any intermediate turn that has greater turning rate than umax
fprintf('Refining in progress..(4/4)\n')
while 1
    
    % Evaluate the approximated turn rates that violate umax
    temp1_deltaphidot = [];
    for lndex = 1:size(path,1)-2
        [center,radius,deltaphi,arclength] = three_point_arc(path(lndex,1:2),path(lndex+1,1:2),path(lndex+2,1:2));
        travelingtime = arclength/V;
        temp1_deltaphidot = [temp1_deltaphidot;deltaphi/travelingtime];
        temp2_deltaphidot = find(temp1_deltaphidot>umax);
        temp3_deltaphidot = size(temp2_deltaphidot,1);
    end
    
    % Randomly choose one violation and sample points from two sides for
    % refinement
    if temp3_deltaphidot~=0
        temp4_deltaphidot = temp2_deltaphidot(randi([1,temp3_deltaphidot]));
        point1 = path(temp4_deltaphidot,1:2)+rand(1)*(path(temp4_deltaphidot,1:2)-path(temp4_deltaphidot+1,1:2));
        point2 = path(temp4_deltaphidot+1,1:2)+rand(1)*(path(temp4_deltaphidot+2,1:2)-path(temp4_deltaphidot+1,1:2));
        if collision_eval(point1,point2,obstacle,xymax,xymin)
            continue
        end
        path = [path(1:temp4_deltaphidot,:);point2,0;point1,0;path(temp4_deltaphidot+2:end,:)];
        pathlength = [0];
        for kndex = size(path,1):-1:2
            pathlength = [pathlength(1,:)+norm(path(kndex-1,1:2)-...
                          path(kndex,1:2));pathlength];
        end
    else
        return
    end
end
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
function [center,radius,deltaphi,arclength] = three_point_arc(p1,p2,p3)
%   Function: Fitting picked 3 points of trajectory with circular arc and
%             obtaining paramenters of arc

% center = center of circular arc
% radius = radius of circular arc
% deltaphi = heading angle change
% arclength = circular arc length
center = [];
center(1,1) = 0.5*(norm(p1)^2*(p3(1,2)-p2(1,2))+norm(p2)^2*(p1(1,2)-p3(1,2))+norm(p3)^2*(p2(1,2)-p1(1,2)))...
                 /(p1(1,1)*(p3(1,2)-p2(1,2))+p2(1,1)*(p1(1,2)-p3(1,2))+p3(1,1)*(p2(1,2)-p1(1,2)));
center(1,2) = 0.5*(norm(p1)^2*(p3(1,1)-p2(1,1))+norm(p2)^2*(p1(1,1)-p3(1,1))+norm(p3)^2*(p2(1,1)-p1(1,1)))...
                 /(p1(1,2)*(p3(1,1)-p2(1,1))+p2(1,2)*(p1(1,1)-p3(1,1))+p3(1,2)*(p2(1,1)-p1(1,1)));
radius = norm(p1-center);
vec1 = p1-center;
vec2 = p3-center;
deltaphi = acos(dot(vec1,vec2)/norm(vec1)/norm(vec2));
arclength = radius*deltaphi;