function [ output_args ] = learn_SVM( X,Y )
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明
    %% 2 a): normal svm model
        rng(1);
        t = templateSVM('Standardize',true,'KernelFunction','gaussian');
        SVMModel = fitcecoc(X,Y,'Learners',t,'FitPosterior',true,...
            'ClassNames',{'1','2','3','4'},'Verbose',2) %训练分类器
        
    %% 2 b) :optimal the svm model
        rng default
        options = statset('UseParallel',true);
        SVMModel = fitcecoc(X,Y,'OptimizeHyperparameters','auto',...
            'HyperparameterOptimizationOptions',struct('AcquisitionFunctionName',...
            'expected-improvement-plus'),'Options',options)
        
     %% 3: using train model get prediect
        [label,~,~,Posterior] = resubPredict(SVMModel,'Verbose',1);
        SVMModel.BinaryLoss
        idx = randsample(size(X,1),10,1);
        SVMModel.ClassNames
        table(Y(idx),label(idx),Posterior(idx,:),...
             'VariableNames',{'TrueLabel','PredLabel','Posterior'})
         
      %% 4: model error
        CodingMat = SVMModel.CodingMatrix
        error = resubLoss(SVMModel)
      % model CV error
        CVMdl = crossval(SVMModel,'Options',options);
        genError = kfoldLoss(CVMdl)
        
         
      %% 5: to show in low demon,relearning a  2D model
         X1=[train_set{ff}([9 15],:)]';
        Y1=label_of_data_all_phase';
         SVMModel_1 = fitcecoc(X1,Y1,'Learners',t,'FitPosterior',true,...
            'ClassNames',{'1','2','3','4'},'Verbose',2) %训练分类器
        xMax = max(X1);
        xMin = min(X1);

        x1Pts = linspace(xMin(1),xMax(1));
        x2Pts = linspace(xMin(2),xMax(2));
        [x1Grid,x2Grid] = meshgrid(x1Pts,x2Pts);

        [~,~,~,PosteriorRegion] = predict(SVMModel_1,[x1Grid(:),x2Grid(:)]);
        
        % plot result
        contourf(x1Grid,x2Grid,...
        reshape(max(PosteriorRegion,[],2),size(x1Grid,1),size(x1Grid,2)));
        h = colorbar;
        h.YLabel.String = 'Maximum posterior';
        h.YLabel.FontSize = 15;

        hold on
        gh = gscatter(X1(:,1),X1(:,2),Y,'krgy','*xdo',8);
        gh(2).LineWidth = 2;
        gh(3).LineWidth = 2;

        title('Iris Petal Measurements and Maximum Posterior')
        xlabel('force (N)')
        ylabel('force_d (N/s)')
        axis tight
        legend(gh,'Location','NorthWest')
        hold off


end

