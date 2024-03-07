close all
clear
clc

tform{1} = trvec2tform([0,0,0])*eul2tform(deg2rad([0,0,0]));

tform{2} = tform{1}*trvec2tform([440.58, 144.31, 587.36])*eul2tform(deg2rad([176.70, 2.82, -142.72]));

tform{3} = tform{2}*trvec2tform([-50, 0, 60])*eul2tform(deg2rad([90, 180, 0]));

tform{4} = tform{3}*[1 0 0 0; 0 0.983 -0.186 44.633; 0 0.186 0.983 -27.776; 0 0 0 1];

figure()
hold on

for i=1:4
    plotTransforms(tform2trvec(tform{i}), tform2quat(tform{i}), 'FrameSize', 30);
end


for i=1:2
    ptCloud_artec{i} = pcread(sprintf('%s[%d].ply', 'C:\Users\joanneyoon\Documents\MATLAB\kuka_artec\case\self\artec\frame', i-1));
end

kuka = [
    524.094   117.677   478.731   1.4756   0.6497   1.3217
    520.203   77.2862   509.493   1.4757   0.6497   1.3219
    ];

for i=1:2
    tform_ex{i} = trvec2tform(kuka(i,1:3))*eul2tform(kuka(i,4:6));
    plotTransforms(tform2trvec(tform_ex{i}), tform2quat(tform_ex{i}), 'FrameSize', 50);
end

hold off

grid on
axis square
axis equal