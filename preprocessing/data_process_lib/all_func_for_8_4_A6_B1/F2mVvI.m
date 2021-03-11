    %% % using GMM learning F-> magnititude of V,vector I
    function [Mu_F2VImagit,Sigma_F2VImagit,Priors_F2VImagit,expData_F2VImagit,...
        expSigma_F2VImagit,input_dim_F2VImagit,output_dim_F2VImagit,data_mean_F2VImagit,...
        data_range_F2VImagit] =F2mVvI(pdemos_force,pdemos_vel,pdemos_imp,verfy_pdemos_force,verfy_pdemos_imp)
    % %%% learning magnitude
    normalize=0;
    
    scale_vel=300e3;
    scale_vel=1;
    
    magitude_vel=[sqrt(pdemos_vel(2,:).^2+pdemos_vel(3,:).^2+pdemos_vel(4,:).^2)]*scale_vel;
    magitude_imp=[sqrt(pdemos_imp(2,:).^2+pdemos_imp(3,:).^2+pdemos_imp(4,:).^2)];
    demos_mix=[pdemos_force;magitude_vel;pdemos_imp(2:4,:)];
    verfine_data=[verfy_pdemos_force;verfy_pdemos_imp(2:4,:)];
    k_GMM=6;
    input_dim_F2VImagit=3;
    output_dim_F2VImagit=4;
        ellpsiod_magnitude_GMM=0.8;ellpsiod_magnitude_GMM_3D=1.8;
    ellpsiod_magnitude_GMR=0.1;ellpsiod_magnitude_GMR_3D=1.8;
    [Mu_F2VImagit,Sigma_F2VImagit,Priors_F2VImagit,expData_F2VImagit,expSigma_F2VImagit,...
        data_mean_F2VImagit,data_range_F2VImagit]...
        = gmm_gmr_mutil_dimension_normalize_learner(demos_mix,input_dim_F2VImagit,...
        output_dim_F2VImagit,verfine_data,k_GMM,normalize,ellpsiod_magnitude_GMM,...
        ellpsiod_magnitude_GMM_3D,ellpsiod_magnitude_GMR,ellpsiod_magnitude_GMR_3D);
    end