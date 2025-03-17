clear all;clc;close all;

% Choose dataset here

load BAL15_set.mat


% The executing function that should be altered for alternate purposes than
% the sample functionality

sampleSimRunner(testsids,testpars);

%% Auxiliary functions

function sampleSimRunner(sidsel_cell,curpars)
    
    % Prepare files for consumption by Simulink
    filenames = {};
    for i = 1:numel(sidsel_cell)
        currentsidsel = sidsel_cell{i};
        t_dat = currentsidsel.t';
        d_dat = currentsidsel.onboard.thetacmd';
        class(t_dat)
        eval(['sidsel' num2str(i) '_validation = Simulink.SimulationData.Dataset();'])
        eval(['sidsel' num2str(i) '_validation = sidsel' num2str(i) '_validation.addElement([t_dat d_dat],''sp_pitch'');']);
        filenames{end+1,1} = ['temp_sidsel' num2str(i) '_validation.mat'];
        sidsel1_validation
        % name of file, variable to be saved in it
        save(filenames{i},['sidsel' num2str(i) '_validation'])
        pwd
    end
    % Set trim flapping frequency by establishing a force balance using
    % mean value of state w in the beginning of the maneuver and the 
    % thrust force approximation
    
    if numel(sidsel_cell{1}.opti.thetaF) > 0
        earliestsetpointstarttime = 9999999;
        earliestendtime = 9999999;
        for i = 1:numel(sidsel_cell)
            findpts = sidsel_cell{i}.onboard.thetacmd/pi*180;
            findpts(abs(findpts)/pi*180<10) = 0;
            earliestind = find(findpts,1);
            earliesttime = sidsel_cell{i}.t(earliestind);
            if earliesttime < earliestsetpointstarttime
                earliestsetpointstarttime = earliesttime;
            end
            if sidsel_cell{i}.t(end) < earliestendtime
                earliestendtime = sidsel_cell{i}.t(end);
            end
            sidsel_cell{i}.setpointstarttime = earliesttime;
        end

        meancounter = 0;
        for i = 1:numel(sidsel_cell)
            %if sum(sidsel_cell{i}.opti.theta) > 0
            %    continue
            %end
            
            
            deltatime = earliestsetpointstarttime - sidsel_cell{i}.setpointstarttime;
            meancounter = meancounter + 1;

            wvec = sidsel_cell{i}.opti.wF;

            if meancounter == 1
                seedtime = sidsel_cell{i}.t + deltatime;
                seedind = seedtime < 0.6*earliestendtime & seedtime > 0;
                seedtime = seedtime(seedind);
                sidsel_cell{i}.homogenized_w = wvec(seedind);
                added_w = sidsel_cell{i}.homogenized_w;
            else
                sidsel_cell{i}.homogenized_w = interp1(sidsel_cell{i}.t + deltatime,wvec,seedtime);
                added_w = added_w + sidsel_cell{i}.homogenized_w;
            end
        end
        mean_w = added_w/meancounter;
        curpars.f0 = (0.5*curpars.m*curpars.g - curpars.c2 - curpars.bz*mean_w(1))/curpars.c1;
        curpars.w0 = mean_w(1);
    end

    % fede:
    % sidsel_cell{1}: is a struct with fields t, opti, onboard, name,
    % setpointsstarttime, homogenized_w // basically the data

    sidsel_cell
    % Run simulations and collect results
    sims = struct();
    assignin('base','pars', curpars);
    for i = 1:numel(sidsel_cell)
        %filenames{i} is some sort of struct, 
        simOut = runSimulation(filenames{i},['sidsel' num2str(i) '_validation']);
            
        yout = get(simOut,'yout');
        
        try
            sims.(['sim' num2str(i)]).ud_s.Data = yout.get(1).Values.Data;
            sims.(['sim' num2str(i)]).ud_s.Time = yout.get(1).Values.Time;
            
            sims.(['sim' num2str(i)]).u_s.Data = yout.get(2).Values.Data;
            sims.(['sim' num2str(i)]).u_s.Time = yout.get(2).Values.Time;
            
            sims.(['sim' num2str(i)]).wd_s.Data = yout.get(3).Values.Data;
            sims.(['sim' num2str(i)]).wd_s.Time = yout.get(3).Values.Time;
            
            sims.(['sim' num2str(i)]).w_s.Data = yout.get(4).Values.Data;
            sims.(['sim' num2str(i)]).w_s.Time = yout.get(4).Values.Time;
            
            sims.(['sim' num2str(i)]).rp_s.Data = yout.get(5).Values.Data;
            sims.(['sim' num2str(i)]).rp_s.Time = yout.get(5).Values.Time;

            sims.(['sim' num2str(i)]).p_s.Data = yout.get(6).Values.Data;
            sims.(['sim' num2str(i)]).p_s.Time = yout.get(6).Values.Time;

            sims.(['sim' num2str(i)]).rpd_s.Data = yout.get(7).Values.Data;
            sims.(['sim' num2str(i)]).rpd_s.Time = yout.get(7).Values.Time;

            sims.(['sim' num2str(i)]).pd_s.Data = yout.get(8).Values.Data;
            sims.(['sim' num2str(i)]).pd_s.Time = yout.get(8).Values.Time;

            sims.(['sim' num2str(i)]).rpdd_s.Data = yout.get(9).Values.Data;
            sims.(['sim' num2str(i)]).rpdd_s.Time = yout.get(9).Values.Time;

            sims.(['sim' num2str(i)]).pdd_s.Data = yout.get(10).Values.Data;
            sims.(['sim' num2str(i)]).pdd_s.Time = yout.get(10).Values.Time;

            sims.(['sim' num2str(i)]).PPRZ_s.Data = yout.get(11).Values.Data;
            sims.(['sim' num2str(i)]).PPRZ_s.Time = yout.get(11).Values.Time;
            
            sims.(['sim' num2str(i)]).sp_s.Data = yout.get(14).Values.Data;
            sims.(['sim' num2str(i)]).sp_s.Time = yout.get(14).Values.Time;
            
            sims.(['sim' num2str(i)]).lp_s.Data = yout.get(15).Values.Data;
            sims.(['sim' num2str(i)]).lp_s.Time = yout.get(15).Values.Time;
            
            sims.(['sim' num2str(i)]).lpd_s.Data = yout.get(16).Values.Data;
            sims.(['sim' num2str(i)]).lpd_s.Time = yout.get(16).Values.Time;
        catch
            sims.(['sim' num2str(i)]).ud_s.Data = yout.signals(1).values;
            sims.(['sim' num2str(i)]).ud_s.Time = yout.time;
            
            sims.(['sim' num2str(i)]).u_s.Data = yout.signals(2).values;
            sims.(['sim' num2str(i)]).u_s.Time = yout.time;
            
            sims.(['sim' num2str(i)]).wd_s.Data = yout.signals(3).values;
            sims.(['sim' num2str(i)]).wd_s.Time = yout.time;
            
            sims.(['sim' num2str(i)]).w_s.Data = yout.signals(4).values;
            sims.(['sim' num2str(i)]).w_s.Time = yout.time;
            
            
            sims.(['sim' num2str(i)]).rp_s.Data = yout.signals(5).values;
            sims.(['sim' num2str(i)]).rp_s.Time = yout.time;

            sims.(['sim' num2str(i)]).p_s.Data = yout.signals(6).values;
            sims.(['sim' num2str(i)]).p_s.Time = yout.time;

            sims.(['sim' num2str(i)]).rpd_s.Data = yout.signals(7).values;
            sims.(['sim' num2str(i)]).rpd_s.Time = yout.time;

            sims.(['sim' num2str(i)]).pd_s.Data = yout.signals(8).values;
            sims.(['sim' num2str(i)]).pd_s.Time = yout.time;

            sims.(['sim' num2str(i)]).rpdd_s.Data = yout.signals(9).values;
            sims.(['sim' num2str(i)]).rpdd_s.Time = yout.time;

            sims.(['sim' num2str(i)]).pdd_s.Data = yout.signals(10).values;
            sims.(['sim' num2str(i)]).pdd_s.Time = yout.time;

            sims.(['sim' num2str(i)]).PPRZ_s.Data = yout.signals(11).values;
            sims.(['sim' num2str(i)]).PPRZ_s.Time = yout.time;
            
            sims.(['sim' num2str(i)]).sp_s.Data = yout.signals(14).values;
            sims.(['sim' num2str(i)]).sp_s.Time = yout.time;
            
            sims.(['sim' num2str(i)]).lp_s.Data = yout.signals(15).values;
            sims.(['sim' num2str(i)]).lp_s.Time = yout.time;
            
            sims.(['sim' num2str(i)]).lpd_s.Data = yout.signals(16).values;
            sims.(['sim' num2str(i)]).lpd_s.Time = yout.time;
        end
    end

    % Start plotting
    
    simpitchcolor = [44,123,182]/255;
    optipitchcolor = [215,25,28]/255;
    imupitchcolor = [253,174,97]/255;
    c2lw = 2;

    earliestsetpointstarttime = 9999999;
    earliestendtime = 9999999;
    for i = 1:numel(sidsel_cell)
        findpts = sims.(['sim' num2str(i)]).rp_s.Data;
        findpts(abs(findpts)/pi*180<10) = 0;
        earliestind = find(findpts,1);
        earliesttime = sims.(['sim' num2str(i)]).sp_s.Time(earliestind);
        endtime = sims.(['sim' num2str(i)]).sp_s.Time(end);
        if earliesttime < earliestsetpointstarttime
            earliestsetpointstarttime = earliesttime;
        end
        if endtime < earliestendtime
            earliestendtime = endtime;
        end
        sims.(['sim' num2str(i)]).setpointstarttime = earliesttime;
        sims.(['sim' num2str(i)]).endtime = endtime;
    end

    % Individual maneuvers
    
    for i = 1:numel(sidsel_cell)
       hc = figure;
       hold on
       doOpti = 1;
       if isempty(sidsel_cell{i}.opti.theta)
           doOpti = 0;
       end
       plot(sidsel_cell{i}.t,sidsel_cell{i}.onboard.thetaF/pi*180,'Color',imupitchcolor,'LineWidth',c2lw)
       if doOpti
           plot(sidsel_cell{i}.t,sidsel_cell{i}.opti.thetaF/pi*180,'Color',optipitchcolor,'LineWidth',c2lw)
       end
       plot(sims.(['sim' num2str(i)]).p_s.Time,sims.(['sim' num2str(i)]).p_s.Data/pi*180,'Color',simpitchcolor,'LineWidth',c2lw)
       plot(sims.(['sim' num2str(i)]).sp_s.Time,sims.(['sim' num2str(i)]).sp_s.Data/pi*180,'k-','LineWidth',c2lw)
       hold off
       psm = sum(sims.(['sim' num2str(i)]).sp_s.Data/pi*180);
       if psm > 0
           ptype = 'backward pulse';
           ptypeshort = 'bw';
       else
           ptype = 'forward pulse';
           ptypeshort = 'fw';
       end
       sims.(['sim' num2str(i)]).ptypeshort = ptypeshort;
       title({['Validation maneuver #' num2str(i) ': ' ptype],sidsel_cell{i}.name})
       if doOpti
           legend({'IMU','Opti','Sim','Setpoint'},'Location','best')
       else
           legend({'IMU','Sim','Setpoint'},'Location','best')
       end
       xlabel('Time [s]');ylabel('Pitch [deg]');
       axis tight

    end

    % Figure showing all setpoints and references
    hs = figure;
    for i = 1:numel(sidsel_cell)
       deltatime = earliestsetpointstarttime - sims.(['sim' num2str(i)]).setpointstarttime;
       subplot(3,1,1)
       hold on
       plot(sims.(['sim' num2str(i)]).sp_s.Time + deltatime,sims.(['sim' num2str(i)]).sp_s.Data/pi*180,'LineWidth',c2lw)
       hold off
       subplot(3,1,2)
       hold on
       plot(sims.(['sim' num2str(i)]).rp_s.Time + deltatime,sims.(['sim' num2str(i)]).rp_s.Data/pi*180,'LineWidth',c2lw)
       hold off
       subplot(3,1,3)
       hold on
       plot(sims.(['sim' num2str(i)]).rpd_s.Time + deltatime,sims.(['sim' num2str(i)]).rpd_s.Data/pi*180,'LineWidth',c2lw)
       hold off
    end

    subplot(3,1,1)
    axis([0 earliestsetpointstarttime+0.3 -Inf Inf])
    ylabel({'Pitch setpoint','[deg]'});
    subplot(3,1,2)
    axis([0 earliestsetpointstarttime+0.3 -Inf Inf])
    ylabel({'Pitch reference','[deg]'});
    subplot(3,1,3)
    axis([0 earliestsetpointstarttime+0.3 -Inf Inf])
    xlabel('Time [s]');ylabel({'Pitch rate reference','[deg/s]'});

    % Average flight maneuver

    ho_frontavg = figure;
    ho_frontavg_counter = 0;
    for i = 1:numel(sidsel_cell)
       if sims.(['sim' num2str(i)]).ptypeshort == 'bw'
           continue
       end

       ho_frontavg_counter = ho_frontavg_counter + 1;

       deltatime = earliestsetpointstarttime - sims.(['sim' num2str(i)]).setpointstarttime;

       doOpti = 1;
       if isempty(sidsel_cell{i}.opti.theta)
           doOpti = 0;
       end

       if doOpti
           pitchvec = sidsel_cell{i}.opti.thetaF/pi*180;
           pitchratevec = sidsel_cell{i}.opti.thetadFF/pi*180;
       else

           pitchvec = sidsel_cell{i}.onboard.thetaF/pi*180;
           pitchratevec = sidsel_cell{i}.onboard.thetadFF/pi*180;
       end


       simpitchvec = sims.(['sim' num2str(i)]).p_s.Data'/pi*180;
       simpitchratevec = sims.(['sim' num2str(i)]).pd_s.Data'/pi*180;

       velvec = sqrt(sidsel_cell{i}.opti.uF.^2 + sidsel_cell{i}.opti.wF.^2);
       uvec = sidsel_cell{i}.opti.uF;
       wvec = sidsel_cell{i}.opti.wF;
       simvelvec = sqrt(sims.(['sim' num2str(i)]).u_s.Data'.^2 + sims.(['sim' num2str(i)]).w_s.Data'.^2);
       simuvec = sims.(['sim' num2str(i)]).u_s.Data';
       simwvec = sims.(['sim' num2str(i)]).w_s.Data';
       if ho_frontavg_counter == 1
           seedtime = sidsel_cell{i}.t + deltatime;
           seedsimtime = sims.(['sim' num2str(i)]).p_s.Time' + deltatime;
           [seedsimtime,seedsimtimeuind] = unique(seedsimtime);
           seedind = seedtime < 0.6*earliestendtime & seedtime > 0;
           seedsimind = seedsimtime < 0.6*earliestendtime & seedsimtime > 0;
           seedtime = seedtime(seedind);
           seedsimtime = seedsimtime(seedsimind);
           sidsel_cell{i}.homogenized_pitch = pitchvec(seedind);

           sidsel_cell{i}.homogenized_pitchrate = pitchratevec(seedind);

           simpitchvec = simpitchvec(seedsimtimeuind);

           simpitchratevec = simpitchratevec(seedsimtimeuind);

           simvelvec = simvelvec(seedsimtimeuind);
           simuvec = simuvec(seedsimtimeuind);
           simwvec = simwvec(seedsimtimeuind);
           sidsel_cell{i}.homogenized_sim_pitch = simpitchvec(seedsimind);

           sidsel_cell{i}.homogenized_sim_pitchrate = simpitchratevec(seedsimind);

           sidsel_cell{i}.homogenized_sim_vel = simvelvec(seedsimind);
           sidsel_cell{i}.homogenized_sim_u = simuvec(seedsimind);
           sidsel_cell{i}.homogenized_sim_w = simwvec(seedsimind);
           if doOpti
               sidsel_cell{i}.homogenized_vel = velvec(seedind);
               sidsel_cell{i}.homogenized_u = uvec(seedind);
               sidsel_cell{i}.homogenized_w = wvec(seedind);
               added_vels = sidsel_cell{i}.homogenized_vel;
               added_u = sidsel_cell{i}.homogenized_u;
               added_w = sidsel_cell{i}.homogenized_w;
           end
           added_pitches = sidsel_cell{i}.homogenized_pitch;

           added_pitchrates = sidsel_cell{i}.homogenized_pitchrate;

           added_sim_pitches = sidsel_cell{i}.homogenized_sim_pitch;

           added_sim_pitchrates = sidsel_cell{i}.homogenized_sim_pitchrate;

           added_sim_vels = sidsel_cell{i}.homogenized_sim_vel;
           added_sim_u = sidsel_cell{i}.homogenized_sim_u;
           added_sim_w = sidsel_cell{i}.homogenized_sim_w;
       else
           sidsel_cell{i}.homogenized_pitch = interp1(sidsel_cell{i}.t + deltatime,pitchvec,seedtime);

           sidsel_cell{i}.homogenized_pitchrate = interp1(sidsel_cell{i}.t + deltatime,pitchratevec,seedtime);

           [uniquetime,uniquetimeind] = unique(sims.(['sim' num2str(i)]).p_s.Time' + deltatime);
           sidsel_cell{i}.homogenized_sim_pitch = interp1(uniquetime,simpitchvec(uniquetimeind),seedsimtime);

           sidsel_cell{i}.homogenized_sim_pitchrate = interp1(uniquetime,simpitchratevec(uniquetimeind),seedsimtime);

           sidsel_cell{i}.homogenized_sim_vel = interp1(uniquetime,simvelvec(uniquetimeind),seedsimtime);
           sidsel_cell{i}.homogenized_sim_u = interp1(uniquetime,simuvec(uniquetimeind),seedsimtime);
           sidsel_cell{i}.homogenized_sim_w = interp1(uniquetime,simwvec(uniquetimeind),seedsimtime);
           added_pitches = added_pitches + sidsel_cell{i}.homogenized_pitch;

           added_pitchrates = added_pitchrates + sidsel_cell{i}.homogenized_pitchrate;

           added_sim_pitches = added_sim_pitches + sidsel_cell{i}.homogenized_sim_pitch;

           added_sim_pitchrates = added_sim_pitchrates + sidsel_cell{i}.homogenized_sim_pitchrate;

           added_sim_vels = added_sim_vels + sidsel_cell{i}.homogenized_sim_vel;
           added_sim_u = added_sim_u + sidsel_cell{i}.homogenized_sim_u;
           added_sim_w = added_sim_w + sidsel_cell{i}.homogenized_sim_w;
           if doOpti
               sidsel_cell{i}.homogenized_vel = interp1(sidsel_cell{i}.t + deltatime,velvec,seedtime);
               sidsel_cell{i}.homogenized_u = interp1(sidsel_cell{i}.t + deltatime,uvec,seedtime);
               sidsel_cell{i}.homogenized_w = interp1(sidsel_cell{i}.t + deltatime,wvec,seedtime);
               added_vels = added_vels + sidsel_cell{i}.homogenized_vel;
               added_u = added_u + sidsel_cell{i}.homogenized_u;
               added_w = added_w + sidsel_cell{i}.homogenized_w;
           end
       end

    end
    mean_vels = added_vels/ho_frontavg_counter;
    mean_pitches = added_pitches/ho_frontavg_counter;

    mean_pitchrates = added_pitchrates/ho_frontavg_counter;

    mean_u = added_u/ho_frontavg_counter;
    mean_w = added_w/ho_frontavg_counter;
    mean_sim_vels = added_sim_vels/ho_frontavg_counter;
    mean_sim_pitches = added_sim_pitches/ho_frontavg_counter;

    mean_sim_pitchrates = added_sim_pitchrates/ho_frontavg_counter;

    mean_sim_u = added_sim_u/ho_frontavg_counter;
    mean_sim_w = added_sim_w/ho_frontavg_counter;

    % real
    added_square_errors = zeros(size(mean_pitches));
    for i = 1:numel(sidsel_cell)
       if sims.(['sim' num2str(i)]).ptypeshort == 'bw'
           continue
       end
       added_square_errors = added_square_errors + (sidsel_cell{i}.homogenized_pitch - mean_pitches).^2;
    end

    added_square_rateerrors = zeros(size(mean_pitchrates));
    for i = 1:numel(sidsel_cell)
       if sims.(['sim' num2str(i)]).ptypeshort == 'bw'
           continue
       end
       added_square_rateerrors = added_square_rateerrors + (sidsel_cell{i}.homogenized_pitchrate - mean_pitchrates).^2;
    end

    added_square_vel_errors = zeros(size(mean_vels));
    for i = 1:numel(sidsel_cell)
       if sims.(['sim' num2str(i)]).ptypeshort == 'bw'
           continue
       end
       added_square_vel_errors = added_square_vel_errors + (sidsel_cell{i}.homogenized_vel - mean_vels).^2;
    end

    added_square_u_errors = zeros(size(mean_u));
    for i = 1:numel(sidsel_cell)
       if sims.(['sim' num2str(i)]).ptypeshort == 'bw'
           continue
       end
       added_square_u_errors = added_square_u_errors + (sidsel_cell{i}.homogenized_u - mean_u).^2;
    end

    added_square_w_errors = zeros(size(mean_w));
    for i = 1:numel(sidsel_cell)
       if sims.(['sim' num2str(i)]).ptypeshort == 'bw'
           continue
       end
       added_square_w_errors = added_square_w_errors + (sidsel_cell{i}.homogenized_w - mean_w).^2;
    end

    mean_square_errors = added_square_errors/ho_frontavg_counter;

    mean_square_rateerrors = added_square_rateerrors/ho_frontavg_counter;

    mean_square_vel_errors = added_square_vel_errors/ho_frontavg_counter;
    mean_square_u_errors = added_square_u_errors/ho_frontavg_counter;
    mean_square_w_errors = added_square_w_errors/ho_frontavg_counter;
    stds = sqrt(mean_square_errors);

    stds_rate = sqrt(mean_square_rateerrors);

    stds_vel = sqrt(mean_square_vel_errors);
    stds_u = sqrt(mean_square_u_errors);
    stds_w = sqrt(mean_square_w_errors);

    % sim

    added_sim_square_errors = zeros(size(mean_sim_pitches));
    for i = 1:numel(sidsel_cell)
       if sims.(['sim' num2str(i)]).ptypeshort == 'bw'
           continue
       end
       added_sim_square_errors = added_sim_square_errors + (sidsel_cell{i}.homogenized_sim_pitch - mean_sim_pitches).^2;
    end

    added_sim_square_rateerrors = zeros(size(mean_sim_pitchrates));
    for i = 1:numel(sidsel_cell)
       if sims.(['sim' num2str(i)]).ptypeshort == 'bw'
           continue
       end
       added_sim_square_rateerrors = added_sim_square_rateerrors + (sidsel_cell{i}.homogenized_sim_pitchrate - mean_sim_pitchrates).^2;
    end

    added_square_sim_vel_errors = zeros(size(mean_sim_vels));
    for i = 1:numel(sidsel_cell)
       if sims.(['sim' num2str(i)]).ptypeshort == 'bw'
           continue
       end
       added_square_sim_vel_errors = added_square_sim_vel_errors + (sidsel_cell{i}.homogenized_sim_vel - mean_sim_vels).^2;
    end

    added_square_sim_u_errors = zeros(size(mean_sim_u));
    for i = 1:numel(sidsel_cell)
       if sims.(['sim' num2str(i)]).ptypeshort == 'bw'
           continue
       end
       added_square_sim_u_errors = added_square_sim_u_errors + (sidsel_cell{i}.homogenized_sim_u - mean_sim_u).^2;
    end

    added_square_sim_w_errors = zeros(size(mean_sim_w));
    for i = 1:numel(sidsel_cell)
       if sims.(['sim' num2str(i)]).ptypeshort == 'bw'
           continue
       end
       added_square_sim_w_errors = added_square_sim_w_errors + (sidsel_cell{i}.homogenized_sim_w - mean_sim_w).^2;
    end

    mean_square_sim_errors = added_sim_square_errors/ho_frontavg_counter;

    mean_square_sim_rateerrors = added_sim_square_rateerrors/ho_frontavg_counter;

    mean_square_sim_vel_errors = added_square_sim_vel_errors/ho_frontavg_counter;
    mean_square_sim_u_errors = added_square_sim_u_errors/ho_frontavg_counter;
    mean_square_sim_w_errors = added_square_sim_w_errors/ho_frontavg_counter;
    stds_sim = sqrt(mean_square_sim_errors);

    stds_ratesim = sqrt(mean_square_sim_rateerrors);

    stds_vel_sim = sqrt(mean_square_sim_vel_errors);
    stds_u_sim = sqrt(mean_square_sim_u_errors);
    stds_w_sim = sqrt(mean_square_sim_w_errors);

    
    hold on
    fill([seedtime fliplr(seedtime)],[mean_pitches+stds fliplr(mean_pitches-stds)],[.9 .9 .9],'linestyle','none');
    fill([seedsimtime fliplr(seedsimtime)],[mean_sim_pitches+stds_sim fliplr(mean_sim_pitches-stds_sim)],[.7 .7 .7],'linestyle','none');
    plot(seedtime,mean_pitches,'k-','LineWidth',c2lw)
    plot(seedsimtime,mean_sim_pitches,'k--','LineWidth',c2lw)
    hold off

    xlabel('Time [s]');ylabel('Pitch [deg]');
    axis tight
    title({'Average flight maneuver',sidsel_cell{i}.name})
    legend({'Standard deviation','Average maneuver'},'Location','best')



    [~,maxi_r] = max(abs(mean_pitches));
    [~,maxi_s] = max(abs(mean_sim_pitches));

    % Average flight maneuver VS simulation maneuver
    
    ho_frontavg_wvel = figure;
    subplot(4,1,1)
    hold on
    fill([seedtime fliplr(seedtime)],[mean_pitches+stds fliplr(mean_pitches-stds)],[.9 .9 .9],'linestyle','none');
    fill([seedsimtime fliplr(seedsimtime)],[mean_sim_pitches+stds_sim fliplr(mean_sim_pitches-stds_sim)],[.7 .7 .7],'linestyle','none');
    plot(seedtime,mean_pitches,'k-','LineWidth',c2lw)
    plot(seedsimtime,mean_sim_pitches,'k--','LineWidth',c2lw)
    hold off
    ylabel('\Theta [deg]');
    title({sidsel_cell{i}.name,['Sim peak = ' num2str(mean_sim_pitches(maxi_s)) ', real peak = ' num2str(mean_pitches(maxi_r))]})
    axis tight
    legend({'\sigma_{real}','\sigma_{sim}','\mu_{real}','\mu_{sim}'},'Location','best','Orientation','horizontal')
    legend('boxoff')

    subplot(4,1,2)
    hold on
    fill([seedtime fliplr(seedtime)],[mean_pitchrates+stds_rate fliplr(mean_pitchrates-stds_rate)],[.9 .9 .9],'linestyle','none');
    fill([seedsimtime fliplr(seedsimtime)],[mean_sim_pitchrates+stds_ratesim fliplr(mean_sim_pitchrates-stds_ratesim)],[.7 .7 .7],'linestyle','none');
    plot(seedtime,mean_pitchrates,'k-','LineWidth',c2lw)
    plot(seedsimtime,mean_sim_pitchrates,'k--','LineWidth',c2lw)
    hold off
    ylabel('q [deg/s]');
    axis tight

    subplot(4,1,3)
    hold on
    fill([seedtime fliplr(seedtime)],[mean_u+stds_u fliplr(mean_u-stds_u)],[.9 .9 .9],'linestyle','none');
    fill([seedsimtime fliplr(seedsimtime)],[mean_sim_u+stds_u_sim fliplr(mean_sim_u-stds_u_sim)],[.7 .7 .7],'linestyle','none');
    plot(seedtime,mean_u,'k-','LineWidth',c2lw)
    plot(seedsimtime,mean_sim_u,'k--','LineWidth',c2lw)
    hold off
    ylabel('u [m/s]');
    axis tight
    subplot(4,1,4)
    hold on
    fill([seedtime fliplr(seedtime)],[mean_w+stds_w fliplr(mean_w-stds_w)],[.9 .9 .9],'linestyle','none');
    fill([seedsimtime fliplr(seedsimtime)],[mean_sim_w+stds_w_sim fliplr(mean_sim_w-stds_w_sim)],[.7 .7 .7],'linestyle','none');
    plot(seedtime,mean_w,'k-','LineWidth',c2lw)
    plot(seedsimtime,mean_sim_w,'k--','LineWidth',c2lw)
    hold off
    ylabel('w [m/s]');
    axis tight

end

function simOut = runSimulation(filename,varname)

filename
% filename is testsids(1) or testsids(n)
% varname is a 
% Create a cell array storing the file names of the input data 
cellOfFiles = { ...
filename , ...
}; 
% varname, dataset variable name in the filename

% Create a cell array storing the variables names of the scenario data 
cellOfVarNames = { ...
varname , ...
}; 

% Create a cell array storing the input strings for each scenario 
cellOfInputStrings = { ...
[varname '.getElement(1)'] , ...
}; 

cellOfErrors = cell(1,length( cellOfInputStrings ));
simOut = Simulink.SimulationOutput.empty(0,length( cellOfInputStrings ));

disp(['Varname: ' varname])
disp(['Filename: ' filename])

% For each scenario 
for kScenario = 1: length( cellOfInputStrings ) 
    
	 try 
        % Load mat and get stoptime
        load(cellOfFiles{1},cellOfVarNames{1})
        dataarray = eval([varname '.get(1)']);
        stoptime = dataarray(end,1);
	 	% Load Variable from file 
        disp(['File: ' pwd '\' cellOfFiles{kScenario}])
        disp(['Variable: ' pwd '\' cellOfVarNames{kScenario}])
	 	loadScenarioToWorkspace( cellOfFiles{kScenario},cellOfVarNames{kScenario}); 
	 	% sim scenario 
     
	 	simOut(kScenario) = sim( 'CL_fullnonlin_prevval_ucorr', 'ExternalInput', cellOfInputStrings{kScenario}, 'LoadExternalInput', 'on','StopTime',num2str(stoptime),'timeout',30);
	 	varName = sprintf('%s',cellOfVarNames{kScenario}); 
	 	evalin('base',sprintf('clear %s',varName));
	 catch ME 
	 	cellOfErrors{ length(cellOfErrors) + 1 } = ME.message; 
	 end 
end 
% End for each scenario 

% Report Errors 
idxEmpty = cellfun(@isempty, cellOfErrors); 
cellOfErrors(idxEmpty)= []; 
if ~isempty(cellOfErrors) 

	disp('These errors occurred while running this script.');
	% For each error 
	for kErr = 1: length( cellOfErrors ) 
		disp(cellOfErrors{kErr}); 
	end 
	% End for each error 

end 
end 
