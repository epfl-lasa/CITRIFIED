
function [sc,esn_nets]=ESNtraining(trainInputSequence,trainOutputSequence,nUn,i,nForgetPoints,save_target)

score_train=[];

disp('Generating ESN ............');
nInternalUnits = nUn;
nInputUnits =  size(trainInputSequence{1},2);   
nOutputUnits =  size(trainOutputSequence{1},2); 

check=1;

while check
    try   
        esn = generate_esn(nInputUnits, nInternalUnits, nOutputUnits, 'spectralRadius',1,'learningMode', 'offline_multipleTimeSeries', 'reservoirActivationFunction', 'tanh','outputActivationFunction', 'identity','inverseOutputActivationFunction','identity', 'type','plain_esn'); 
        esn.internalWeights = esn.spectralRadius * esn.internalWeights_UnitSR;
        check=0;
    catch
        disp('failure generating esn')
    end
end
    
% train ESN
    
disp('Training ESN ............');

%nForgetPoints = 10 ; % discard the first 100 points
[trainedEsn stateMatrix] = train_esn(trainInputSequence', trainOutputSequence', esn, nForgetPoints) ; 
    
predictedTrainOutput = [];
for j=1:length(trainInputSequence)
    predictedTrainOutput{j} = zeros(length(trainInputSequence{j})-nForgetPoints, size(trainOutputSequence{1},2));
    predictedTrainOutput{j} = test_esn(trainInputSequence{j}, trainedEsn, nForgetPoints);
end

[all_output_train, av_predictedTrainOutput, success_rate_train, av_confidence_all_train, std_confidence_all_train, av_max_conf_train, std_max_conf_train, errortrain{i},errortrain2{i},Con_Matrix_train{i}] = S_classify2(predictedTrainOutput, trainOutputSequence, 3, i, 'train');

score_train = [score_train; i success_rate_train av_confidence_all_train std_confidence_all_train av_max_conf_train std_max_conf_train]
    
filename=[save_target num2str(i) '.txt'];
    
writetofile(trainedEsn, filename, nForgetPoints);
    
esn_nets=trainedEsn;
sc=score_train;
    
end