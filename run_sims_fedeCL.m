clc; clearvars;

% Try using closed loop control, i don't have any access to the  

load BAL15_set.mat;

actual = testsids{1, 1}.opti.udFF;
time = testsids{1, 1}.t;


% Organise input data
for i= 1:numel(testsids)
    current_struct = testsids{i};
    t_data = current_struct.t';
    pitch_data = current_struct.onboard.thetacmd';
    eval(['input' num2str(i) '_data = Simulink.SimulationData.Dataset();'])
    eval(['input' num2str(i) '_data = input' num2str(i) '_data.addElement([t_data pitch_data],''sp_pitch'');']);
    
    % Save data file in Simulink dataset object
    save(['temp/input' num2str(i) '_temp'], ['input' num2str(i) '_data'])

end



if numel(testsids{1}.opti.thetaF) > 0
    earliestsetpointstarttime = 9999999;
    earliestendtime = 9999999;
    for i = 1:numel(testsids)
            findpts = testsids{i}.onboard.thetacmd/pi*180;
            findpts(abs(findpts)/pi*180<10) = 0;
            earliestind = find(findpts,1);
            earliesttime = testsids{i}.t(earliestind);
            if earliesttime < earliestsetpointstarttime
                earliestsetpointstarttime = earliesttime;
            end
            if testsids{i}.t(end) < earliestendtime
                earliestendtime = testsids{i}.t(end);
            end
            testsids{i}.setpointstarttime = earliesttime;
    end
    meancounter = 0;
    for i = 1:numel(testsids)

            deltatime = earliestsetpointstarttime - testsids{i}.setpointstarttime;
            meancounter = meancounter + 1;

            wvec = testsids{i}.opti.wF;
        if meancounter == 1
            seedtime = testsids{i}.t + deltatime;
            seedind = seedtime < 0.6*earliestendtime & seedtime > 0;
            seedtime = seedtime(seedind);
            testsids{i}.homogenized_w = wvec(seedind);
            added_w = testsids{i}.homogenized_w;
        else
            testsids{i}.homogenized_w = interp1(testsids{i}.t + deltatime,wvec,seedtime);
            added_w = added_w + testsids{i}.homogenized_w;
        end
    end
    mean_w = added_w/meancounter;
    testpars.f0 = (0.5*testpars.m*testpars.g - testpars.c2 - testpars.bz*mean_w(1))/testpars.c1;
    testpars.w0 = mean_w(1);
end


cell_input = {'input1_data.getElement(1)'};

dataarray = input1_data.get(1);
stoptime = dataarray(end,1);
assignin('base','pars', testpars);

simOut = sim('CL_fullnonlin_prevval_ucorr', 'ExternalInput', cell_input{1}, 'LoadExternalInput', 'on','StopTime',num2str(stoptime),'timeout',30);

yout = get(simOut,'yout');

i = 0;

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
% save('output/outputdata', 'sims')

tiledlayout(3, 1)

nexttile
plot(sims.sim0.ud_s.Time, sims.sim0.ud_s.Data, 'DisplayName', 'model', 'LineStyle','--', 'Color', 'red')
hold on
plot(time, actual, 'DisplayName', 'flight data',  'Color', 'black')
hold off
ylabel('$\dot{u} [m/s^2]$', 'Interpreter','latex')

legend

nexttile
plot(sims.sim0.wd_s.Time, sims.sim0.wd_s.Data, 'DisplayName', 'model', 'LineStyle','--', 'Color', 'red')
hold on
plot(testsids{1, 1}.t, testsids{1, 1}.opti.wdFF, 'DisplayName', 'flight data', 'Color', 'black')
hold off
ylabel('$\dot{w} [m/s^2]$', 'Interpreter','latex')

nexttile
plot(sims.sim0.pdd_s.Time, sims.sim0.pdd_s.Data, 'DisplayName', 'model', 'LineStyle','--', 'Color', 'red')
hold on
plot(testsids{1, 1}.t, testsids{1, 1}.opti.thetaddFF, 'DisplayName', 'flight data', 'Color', 'black')
hold off
ylabel('$\ddot{\theta} [rad/s]$', 'Interpreter','latex')
