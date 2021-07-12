clear all; close all; clc
load 'circ_cuts.mat'
load 'straight_cuts.mat'


figure;
hold on;grid on;
for cut=1:length(straight_cuts)
    data = straight_cuts{cut};
    plot3(data(:,3), data(:,6), data(:,2));
end
for cut=1:length(circ_cuts)
    data = circ_cuts{cut};
    plot3(data(:,3), data(:,6), data(:,2));
end
xlabel('twist X')
ylabel('force X')
zlabel('depth')


figure;
hold on;grid on;
for cut=1:length(straight_cuts)
    data = straight_cuts{cut};
    plot3(data(:,4), data(:,7), data(:,2));
end
for cut=1:length(circ_cuts)
    data = circ_cuts{cut};
    plot3(data(:,4), data(:,7), data(:,2));
end
xlabel('twist Y')
ylabel('force Y')
zlabel('depth')

