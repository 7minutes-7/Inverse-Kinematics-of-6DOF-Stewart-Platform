clear;
clc;

close all

%% Load data
for i=1:2
    ptCloud_artec(i) = pcread(sprintf('%s[%d].ply', 'C:\Users\joanneyoon\Documents\MATLAB\kuka_artec\case\self\artec\frame', i-1));
end

%% Pre processing
kuka = [
    524.094   117.677   478.731   1.4756   0.6497   1.3217
    520.203   77.2862   509.493   1.4757   0.6497   1.3219
    ];

for i = 1:2
    pc_array{i} = ptCloud_artec(i).Location;
    for j = 1:length(pc_array{i})
        rot{i} = eul2rotm(kuka(i,4:6));%*eul2rotm(deg2rad([-90 0 90]),'XYZ');
        pc_array_tf{i}(j,:)= pc_array{i}(j,:)*rot{i}' + kuka(i,1:3);
    end
end

figure()
hold on
plotTransforms([0,0,0],eul2quat([0,0,0]),'FrameSize',1000);
for i=1:2
    xx = pc_array_tf{i}(:,1);
    yy = pc_array_tf{i}(:,2);
    zz = pc_array_tf{i}(:,3);
    
    plotTransforms(kuka(i,1:3),rotm2quat(rot{i}),'FrameSize',1000);
    plot3(xx,yy,zz, '.', 'MarkerSize',8, ...
        'MarkerFaceColor',[i*.4 0 0]);
end
grid on
hold off
xlabel('X');
ylabel('Y');
zlabel('Z');
axis equal

xlim([-1000 1000])
ylim([-1000 1000])
zlim([-1000 1000])

% eul = [ kuka(1,4:6)
%         kuka(2,4:6) ];
% tformZYX = eul2tform(eul);

% tform = trvec2tform(kuka(1,1:3))*eul2tform(kuka(1,4:6));
% ptCloudTformed(1) = pctransform(ptCloud_artec(1),rigid3d(tform'));
%
% tform = trvec2tform(kuka(1,1:3)-delta(1:3))*eul2tform(kuka(1,4:6)+delta(4:6));
% ptCloudTformed(2) = pctransform(ptCloud_artec(2),rigid3d(tform'));

% for i=1:2
% %     tr = trvec2tform(kuka(i,1:3));
% %     ro = eul2tform(-kuka(i,4:6));
% %     pc = pctransform(ptCloud_artec(i),rigid3d(ro'));
% %     ptCloudTformed(i) = pctransform(pc,rigid3d(tr'));
%
%     tform = trvec2tform(-kuka(i,1:3))*eul2tform(kuka(i,4:6));
%     ptCloudTformed(i) = pctransform(ptCloud_artec(i),rigid3d(tform'));
% end

%% Display
figure()
hold on
plotTransforms([0,0,0],eul2quat(deg2rad([0,0,0])),'FrameSize',1000);
for i=1
    X = ptCloud_artec(i).Location(:,1);
    Y = ptCloud_artec(i).Location(:,2);
    Z = ptCloud_artec(i).Location(:,3);
    
    plot3(X,Y,Z, '.', 'MarkerSize',8, ...
        'MarkerFaceColor',[i*.4 0 0]);
end
grid on
hold off
xlabel('X');
ylabel('Y');
zlabel('Z');
axis equal
xlim([-1000 1000])
ylim([-1000 1000])
zlim([-1000 1000])
%
% subplot(1,2,2)
% for i=1:2
%     Xt = ptCloudTformed(i).Location(:,1);
%     Yt = ptCloudTformed(i).Location(:,2);
%     Zt = ptCloudTformed(i).Location(:,3);
%
%     grid on
%     plot3(Xt,Yt,Zt, '.', 'MarkerSize',8, ...
%         'MarkerFaceColor',[i*.4 0 0]);
%     hold on
% end
% plot3(0,0,0, '*', 'MarkerSize',20, ...
%     'MarkerFaceColor',[i*.4 0 1]);
% hold off;
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% axis equal