    %% % using GMM learning F-> vector imip for cross-validation
    function [Mu_F2VImagit,Sigma_F2VImagit,Priors_F2VImagit,expData_F2VImagit,...
        expSigma_F2VImagit,input_dim_F2VImagit,output_dim_F2VImagit,data_mean_F2VImagit,...
        data_range_F2VImagit,MSE] =F2mV_CV(pdemos_force,pdemos_imp,verfy_pdemos_force,verfy_pdemos_imp,k_GMM,plot_on)
    normalize=0;
    
    magitude_vel=[sqrt(pdemos_imp(2,:).^2+pdemos_imp(3,:).^2+pdemos_imp(4,:).^2)];
    magitude_vel_verfy=[sqrt(verfy_pdemos_imp(2,:).^2+verfy_pdemos_imp(3,:).^2+verfy_pdemos_imp(4,:).^2)];

    
    demos_mix=[pdemos_force;magitude_vel];
    verfine_data=[verfy_pdemos_force;magitude_vel_verfy];
    input_dim_F2VImagit=3;
    output_dim_F2VImagit=1;

%     k_GMM=3;
        ellpsiod_magnitude_GMM=6;ellpsiod_magnitude_GMM_3D=4;
        ellpsiod_magnitude_GMR=1;ellpsiod_magnitude_GMR_3D=1.8;
        [Mu_F2VImagit,Sigma_F2VImagit,Priors_F2VImagit,expData_F2VImagit,expSigma_F2VImagit,...
        data_mean_F2VImagit,data_range_F2VImagit,MSE] ...
= gmm_gmr_F2mV_CV(demos_mix,input_dim_F2VImagit,...
        output_dim_F2VImagit,verfine_data,k_GMM,normalize,ellpsiod_magnitude_GMM,...
        ellpsiod_magnitude_GMM_3D,ellpsiod_magnitude_GMR,ellpsiod_magnitude_GMR_3D,plot_on);
    end