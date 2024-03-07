close all
clear
clc

lbr = loadrobot("kukaIiwa14",'DataFormat','row');
showdetails(lbr)

base = 'iiwa_link_0';
flange = 'iiwa_link_ee_kuka';

homeConfig = lbr.homeConfiguration;
% tform = getTransform(lbr,homeConfig,flange,base);
% show(lbr,homeConfig);

%% add bracket
body = rigidBody('bracket');
tform = trvec2tform([-43.84e-3, 0, 64.2315e-3])*eul2tform([-pi/2, 0, pi]); % User defined
setFixedTransform(body.Joint,tform);
addBody(lbr,body,flange);

bracket = 'bracket';

%% add scanner
body = rigidBody('scanner');
tform = [1 0 0 0; 0 0.983 -0.186 44.633e-3; 0 0.186 0.983 -27.776e-3; 0 0 0 1]; % User defined
setFixedTransform(body.Joint,tform);
addBody(lbr,body,bracket);

scanner = 'scanner';

%% input joint configuration

testConfig = deg2rad([65.90, 19.45, -51.21, -87.74, -20.94, 90.06, 21.13]);
show(lbr, testConfig)

tform = getTransform(lbr,testConfig,scanner,base);

%% load scan data
ptCloud_artec = pcread('C:\Users\joanneyoon\Documents\MATLAB\kuka_artec\case\self\artec\frame[0].ply');

ptCloud_mat = ptCloud_artec.Location * 1e-3;
ptCloud_tformed = zeros(size(ptCloud_mat));

for i=1:length(ptCloud_mat)
    ptCloud_tformed(i,:) = hom2cart(cart2hom(ptCloud_mat(i,:))*tform');
end

hold on
plot3(ptCloud_tformed(:,1), ptCloud_tformed(:,2), ptCloud_tformed(:,3),'.')
hold off

% for i=1
%     X = ptCloud_artec(i).Location(:,1);
%     Y = ptCloud_artec(i).Location(:,2);
%     Z = ptCloud_artec(i).Location(:,3);
%
%     plot3(X,Y,Z, '.', 'MarkerSize',8, ...
%         'MarkerFaceColor',[i*.4 0 0]);
% end

%% solve inverse kinematics
% ik = inverseKinematics('RigidBodyTree',lbr);
% weights = [0.5 0.5 0.5 1 1 1];
% initialguess = lbr.homeConfiguration;
%
% tform = trvec2tform([440.58, 144.31, 587.36]*1e-3)*eul2tform(deg2rad([176.70, 2.82, 142.72]),'XYZ');
% [configSoln,solnInfo] = ik(flange,tform,weights,initialguess);
