  %% % using SEDS learning the phase1 and phase3
    function [Mu_P2VIdir,Sigma_P2VIdir,Priors_P2VIdir,expData_P2VIdir,expSigma_P2VIdir,...
        input_dim_P2VIdir,output_dim_P2VIdir]=SEDS_P2V(pdemos_pos,pdemos_vel,verfy_pdemos_pos,...
        verfine_time_1,phase,simulation_seds,pos_data_serial_norm_time)
    
    demos_mix=[pdemos_pos(1:4,:);pdemos_vel(2:4,:)];
    verfine_data=[verfy_pdemos_pos];
    input_dim_P2VIdir=3;
    output_dim_P2VIdir=3;
    
     k_GMM = 4; %Number of Gaussian funcitons
     normalize=0;
    
    
    ellpsiod_magnitude_GMM=1;ellpsiod_magnitude_GMM_3D=1;
    ellpsiod_magnitude_GMR=1;ellpsiod_magnitude_GMR_3D=1;
    
    [Mu_P2VIdir,Sigma_P2VIdir,Priors_P2VIdir,expData_P2VIdir,expSigma_P2VIdir,...
        data_mean_P2VIdir,data_range_P2VIdir]...
        = SEDS_mutil_dimension_normalize_learner(demos_mix,input_dim_P2VIdir,...
        output_dim_P2VIdir,verfine_data,verfine_time_1,k_GMM,normalize,ellpsiod_magnitude_GMM,...
        ellpsiod_magnitude_GMM_3D,ellpsiod_magnitude_GMR,ellpsiod_magnitude_GMR_3D,phase,...
        simulation_seds,pos_data_serial_norm_time);
    end