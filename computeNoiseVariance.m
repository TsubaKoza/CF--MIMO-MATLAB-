function noiseVariance = computeNoiseVariance(h_true,pilotEnergy,pilotSNRdB,cfg)
%COMPUTENOISEVARIANCE Map the configured pilot SNR to complex AWGN power.
switch lower(char(cfg.noiseMode))
    case 'networkaveragereceivedsnr'
        perAntennaPower = squeeze(sum(abs(h_true).^2,1)/size(h_true,1));
        referencePower = mean(perAntennaPower(:));
        referencePower = max(referencePower,cfg.nmseEpsilon);
        noiseVariance = mean(pilotEnergy(:))*referencePower/10^(pilotSNRdB/10);
    case 'fixed'
        noiseVariance = cfg.noiseVarianceFixed;
    otherwise
        error('Unknown noiseMode: %s',cfg.noiseMode);
end
end
