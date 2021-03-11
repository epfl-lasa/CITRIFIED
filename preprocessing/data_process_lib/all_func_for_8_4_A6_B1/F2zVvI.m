

    %% % using GMM learning F-> magnititude of z V and sca,vector I
    function [Mu_F2VImagit,Sigma_F2VImagit,Priors_F2VImagit,expData_F2VImagit,...
        expSigma_F2VImagit,input_dim_F2VImagit,output_dim_F2VImagit,data_mean_F2VImagit,...
        data_range_F2VImagit] =F2zVvI(pdemos_force,pdemos_vel,pdemos_imp,verfy_pdemos_force)
    % %%% learning only z direction velocity
    normalize=0;
    
    scale_vel=300e3;
 
    demos_mix=[pdemos_force;sqrt((pdemos_vel(4,:)*scale_vel).^2);pdemos_imp(2:4,:)];
    verfine_data=[verfy_pdemos_force];
    k_GMM=4;
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