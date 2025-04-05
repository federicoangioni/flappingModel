load dataset_revision.mat

NEXP=[1 2 4 13 36 54 55 56 57 58 76 77 78 79 94 95 98 99 100 101 102 103 104];

for i=1:length(NEXP)
    Nexp=NEXP(i);
    
    eval(['data=experiment' num2str(Nexp) ';']);
    assign_repetition_variables
    
    fps=1/median(diff(TIME));
    
    if Nexp>=54 && Nexp<=58
        % Compute marker quality only for the segments shown in the figures, the tracking was bad outside of them (temporary OptiTrack setup)
        QUALavg(i)=mean2(QUAL(:,round(fps*1.4):round(fps*2.1)));
        QUALstd(i)=std2(QUAL(:,round(fps*1.4):round(fps*2.1)));
    else
        QUALavg(i)=mean2(QUAL);
        QUALstd(i)=std2(QUAL);
    end
    
    disp([num2str(Nexp) ': ' num2str(QUALavg(i),'%.1E') ' (' num2str(QUALstd(i),'%.1E') ')'])
    
    
end