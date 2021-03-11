

figure()
hold on
for i=0:24
    load(['tw' num2str(i) '.mat'])
    for j=1:size(tmW,2)
        plot([150*i+1:(i+1)*150],tmW(:,j))
    end    
    
end