    %% % using GMM learning F-> vector imip for cross-validation
    function [Mu_F2VImagit,Sigma_F2VImagit,Priors_F2VImagit,expData_F2VImagit,...
        expSigma_F2VImagit,input_dim_F2VImagit,output_dim_F2VImagit,data_mean_F2VImagit,...
        data_std_F2VImagit,MSE] =insection_classfier_CV_normalize(different_tissue,pdemos_force,pdemos_imp,verfy_pdemos_force,verfy_pdemos_imp,k_GMM,plot_on,normalize)
    
%     normalize=1;


    
    demos_mix=[pdemos_force];
    train_label=pdemos_imp;
    test_label=verfy_pdemos_imp;
    verfine_data=[verfy_pdemos_force];
    input_dim_F2VImagit=3;
    output_dim_F2VImagit=6;

%     k_GMM=3;
    ellpsiod_magnitude_GMM=2;ellpsiod_magnitude_GMM_3D=1.8;
    ellpsiod_magnitude_GMR=1;ellpsiod_magnitude_GMR_3D=1.8;
    
    [Mu_F2VImagit,Sigma_F2VImagit,Priors_F2VImagit,expData_F2VImagit,expSigma_F2VImagit,...
        data_mean_F2VImagit,data_std_F2VImagit,MSE] ...
    = gmm_gmr_insection_classfier_CV(different_tissue,demos_mix,train_label,test_label,input_dim_F2VImagit,...
        output_dim_F2VImagit,verfine_data,k_GMM,normalize,ellpsiod_magnitude_GMM,...
        ellpsiod_magnitude_GMM_3D,ellpsiod_magnitude_GMR,ellpsiod_magnitude_GMR_3D,plot_on);
    end