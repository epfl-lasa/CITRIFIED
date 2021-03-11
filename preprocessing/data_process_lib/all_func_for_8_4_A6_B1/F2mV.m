  %% % using GMM learning F-> magnititude of V
    function [Mu_F2mV,Sigma_F2mV,Priors_F2mV,expData_F2mV,expSigma_F2mV,input_dim_F2mV,...
        output_dim_F2mV,data_mean_F2mV,data_range_F2mV]...
        =F2mV(pdemos_force,pdemos_vel,verfy_pdemos_force,verfy_pdemos_vel)
    normalize=0;
    
    magitude_vel=[sqrt(pdemos_vel(2,:).^2+pdemos_vel(3,:).^2+pdemos_vel(4,:).^2)];
    magitude_vel_verfy=[sqrt(verfy_pdemos_vel(2,:).^2+verfy_pdemos_vel(3,:).^2+verfy_pdemos_vel(4,:).^2)];
%     magitude_imp=[sqrt(pdemos_imp(2,:).^2+pdemos_imp(3,:).^2+pdemos_imp(4,:).^2)];
    demos_mix=[pdemos_force;magitude_vel];%;pdemos_imp(2:4,:)
    verfine_data=[verfy_pdemos_force;magitude_vel_verfy];
    input_dim_F2mV=3;
    output_dim_F2mV=1;

    k_GMM=4;
    ellpsiod_magnitude_GMM=0.1;ellpsiod_magnitude_GMM_3D=1.8;
    ellpsiod_magnitude_GMR=0.1;ellpsiod_magnitude_GMR_3D=1.8;
    
    [Mu_F2mV,Sigma_F2mV,Priors_F2mV,expData_F2mV,expSigma_F2mV,data_mean_F2mV,data_range_F2mV]...
        = gmm_gmr_F2mV(demos_mix,input_dim_F2mV,output_dim_F2mV,verfine_data,k_GMM,normalize,...
        ellpsiod_magnitude_GMM,ellpsiod_magnitude_GMM_3D,ellpsiod_magnitude_GMR,ellpsiod_magnitude_GMR_3D);
    end