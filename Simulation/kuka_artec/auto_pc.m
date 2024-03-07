clear;
clc;

%% Load data
for i=1:6
    ptCloud_artec(i) = pcread(sprintf('%s[%d].ply', 'C:\Users\YeLin\Downloads\case\auto\artec\frame', i-1));
end

%% Pre processing
kuka = [ 524.044  117.555    478.742   1.4755   0.6497   1.3219
         524.051   96.9355   478.743   1.4757   0.6498   1.32201
         524.08    76.9597   478.802   1.4754   0.6496   1.32171
         524.033   56.9936   478.723   1.4755   0.6496   1.32201
         524.074   37.0746   478.739   1.4754   0.6497   1.32191
         524.049   17.0343   478.731   1.4756   0.6497   1.32191 ];
eul = [ kuka(1,4:6)
        kuka(2,4:6)
        kuka(3,4:6)
        kuka(4,4:6)
        kuka(5,4:6)
        kuka(6,4:6)];
tformZYX = eul2tform(eul);

for i=1:6
%     T(i) = affinetform3d((trvec2tform(kuka(i,1:3)) * tformZYX(:,:,i))); 
    tform = rigidtform3d(fliplr(kuka(i,4:6)),kuka(i,1:3));
    ptCloudTformed(i) = pctransform(ptCloud_artec(i),tform);
end

%% Display
figure()

subplot(1,2,1)
for i=1:6
    X = ptCloud_artec(i).Location(:,1,1);
    Y = ptCloud_artec(i).Location(:,2,1);
    Z = ptCloud_artec(i).Location(:,3,1);

    grid on
    plot3(X,Y,Z, '.', 'MarkerSize',8, ...
        'MarkerEdgeColor',[i*.15 i*.15 i*.15],...
        'MarkerFaceColor',[i*.15 i*.15 i*.15]);
    hold on
end
hold off;
xlabel('X');
ylabel('Z');
zlabel('Y');

subplot(1,2,2)
for i=1:6
    Xt = ptCloudTformed(i).Location(:,1,1);
    Yt = ptCloudTformed(i).Location(:,2,1);
    Zt = ptCloudTformed(i).Location(:,3,1);

    grid on
    plot3(Xt,Yt,Zt, '.', 'MarkerSize',8, ...
        'MarkerEdgeColor',[i*.15 i*.15 i*.15],...
        'MarkerFaceColor',[i*.15 i*.15 i*.15]);
    hold on
end
hold off;
xlabel('X');
ylabel('Z');
zlabel('Y');

% view([45 30]);
% xlim([-0.5 0.5])
% ylim([0.3 1])
% zlim([-0.5 0.5])