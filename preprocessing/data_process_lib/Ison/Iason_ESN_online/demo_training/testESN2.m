function [score_test]=testESN2(trainesn,l_tresn,div,nUn,esn,nForgetPoints)

score_test=[];

%for i=1:div

    
    trainInputSequence=struct([]);
    trainOutputSequence=struct([]);
    
    for j=1:length(trainesn)
        trainInputSequence{j}=trainesn{j};
        trainOutputSequence{j}=l_tresn{j};
    end
    
    
    predictedTrainOutput = [];
    for j=1:length(trainInputSequence)
        predictedTrainOutput{j} = zeros(length(trainInputSequence{j})-nForgetPoints, size(trainOutputSequence{1},2));
        predictedTrainOutput{j} = test_esn(trainInputSequence{j}, esn, nForgetPoints);
    end   
    
    
    
    [all_output_train, av_predictedTrainOutput, success_rate_train, av_confidence_all_train, std_confidence_all_train, av_max_conf_train, std_max_conf_train, errortrain,errortrain2,Con_Matrix_train] = S_classify2(predictedTrainOutput, trainOutputSequence, 3, 1, 'train');

    score_test = [score_test; success_rate_train av_confidence_all_train std_confidence_all_train av_max_conf_train std_max_conf_train]


%end

end