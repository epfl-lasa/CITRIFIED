function [gdata] = generateData(nb_samples, varargin)

dims = 1;

mu = 0;

sigma = 1;


if (length(varargin) >= 2)
    for i=1:2:length(varargin)
        if(varargin{i} == "dims")
            dims = varargin{i + 1};
        end
        if(varargin{i} == "mu")
            mu = varargin{i + 1};
        end
        if(varargin{i} == "sigma")
            sigma = varargin{i + 1};
        end
    end
end

R = chol(sigma);
gdata = repmat(mu,nb_samples,1) + randn(nb_samples,dims)*R;

end

