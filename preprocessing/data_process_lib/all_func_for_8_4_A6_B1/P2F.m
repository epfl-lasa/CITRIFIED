%% % using  GMM learning P->dir of V,I (ML course apply here)
function [Mu_P2VIdir,Sigma_P2VIdir,Priors_P2VIdir,expData_P2VIdir,expSigma_P2VIdir,...
    input_dim_P2VIdir,output_dim_P2VIdir]...
    =P2F(pdemos_pos,pdemos_force,verfy_pdemos_pos,verfy_pdemos_force)
%% get data
demos_mix=[pdemos_pos;pdemos_force(2:4,:)];
verfine_data=[verfy_pdemos_pos];
input_dim_P2VIdir=3;
output_dim_P2VIdir=3;
    
%% calculate GMM GMR
    for k=2:9
        k_GMM=k;
        ellpsiod_magnitude_GMM=0.8;ellpsiod_magnitude_GMM_3D=1.8;
        ellpsiod_magnitude_GMR=0.1;ellpsiod_magnitude_GMR_3D=1.8;
        [Mu_P2VIdir,Sigma_P2VIdir,Priors_P2VIdir,expData_P2VIdir,expSigma_P2VIdir]...
            = gmm_gmr_high_dimension_P2VIdir(demos_mix,input_dim_P2VIdir,output_dim_P2VIdir,...
            verfine_data,k_GMM,ellpsiod_magnitude_GMM,ellpsiod_magnitude_GMM_3D,ellpsiod_magnitude_GMR,...
            ellpsiod_magnitude_GMR_3D);

        %% using AIC BIC get the best K of GMM
        [min_like,max_like]=plot_GMR_likelihood...
            (verfine_data,demos_mix,Priors_P2VIdir, Mu_P2VIdir, Sigma_P2VIdir,ellpsiod_magnitude_GMR,ellpsiod_magnitude_GMR_3D);
        
        BIC_function
    end
end
