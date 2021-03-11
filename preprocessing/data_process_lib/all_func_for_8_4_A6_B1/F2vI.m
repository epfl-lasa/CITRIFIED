    %% % using GMM learning F-> vector imip
    function [Mu_F2VImagit,Sigma_F2VImagit,Priors_F2VImagit,expData_F2VImagit,...
        expSigma_F2VImagit,input_dim_F2VImagit,output_dim_F2VImagit,data_mean_F2VImagit,...
        data_range_F2VImagit] =F2vI(pdemos_force,pdemos_imp,verfy_pdemos_force)
    normalize=0;
    
    demos_mix=[pdemos_force;pdemos_imp(2:4,:)];
    verfine_data=[verfy_pdemos_force];
    input_dim_F2VImagit=3;
    output_dim_F2VImagit=3;

    k_GMM=3;
    ellpsiod_magnitude_GMM=2;ellpsiod_magnitude_GMM_3D=1.8;
    ellpsiod_magnitude_GMR=0.1;ellpsiod_magnitude_GMR_3D=1.8;
    [Mu_F2VImagit,Sigma_F2VImagit,Priors_F2VImagit,expData_F2VImagit,expSigma_F2VImagit,...
        data_mean_F2VImagit,data_range_F2VImagit] = gmm_gmr_F2vI(demos_mix,input_dim_F2VImagit,...
        output_dim_F2VImagit,verfine_data,k_GMM,normalize,ellpsiod_magnitude_GMM,...
        ellpsiod_magnitude_GMM_3D,ellpsiod_magnitude_GMR,ellpsiod_magnitude_GMR_3D);
    end