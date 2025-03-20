function [mu_combined, sigma_combined] = combineAverages(mu, sigma, n)
    % Combine means and standard deviations from multiple samples
    % 
    % Inputs:
    %   mu    - 1xN array of means
    %   sigma - 1xN array of standard deviations
    %   n     - 1xN array of sample sizes used to fine the above means/stds
    % 
    % Outputs:
    %   mu_combined    - Combined mean
    %   sigma_combined - Combined standard deviation

    % Compute the combined mean
    mu_combined = sum(n .* mu) / sum(n);
    
    % Compute the combined standard deviation
    num_samples = sum(n);
    pooled_variance = sum((n - 1) .* sigma.^2) + sum(n .* (mu - mu_combined).^2);
    sigma_combined = sqrt(pooled_variance / (num_samples - 1));
end