function [trained_esn, predicted_train_output, predicted_test_output, test_time_mean, test_time_std] = ...
    esn_train_and_predict(esn, train_input, train_output, test_input, nb_forget_points)
    

disp('Training ESN ............');
esn.internalWeights = esn.spectralRadius * esn.internalWeights_UnitSR;
[trained_esn, state_matrix] = train_esn(train_input', train_output', esn, nb_forget_points) ; 

% plot the internal states of 4 units
% plot_states(state_matrix, [1 2 3 4], esn.nInternalUnits, 1, 'traces of first 4 reservoir units') ; 


disp('Predicting ESN ............');
predicted_train_output = [];
for j=1:length(train_input)
    predicted_train_output{j} = test_esn(train_input{j}, trained_esn, nb_forget_points);
end

calc_time=zeros(length(test_input),1);
predicted_test_output = [];
for j=1:length(test_input)
    tic
    predicted_test_output{j} = test_esn(test_input{j},  trained_esn, nb_forget_points) ; 
    calc_time(j)=toc;
end

test_time_mean = sum(calc_time)/length(test_input);
test_time_std = std(calc_time);

end