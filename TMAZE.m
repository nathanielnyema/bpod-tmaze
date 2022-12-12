function TMAZE
    global BpodSystem

    %% Setup (runs once before the first trial)

    %--- Define parameters and trial structure
    S = BpodSystem.ProtocolSettings; % Loads settings file chosen in launch manager into current workspace as a struct called 'S'
    if isempty(fieldnames(S))  % If chosen settings file was an empty struct, populate struct with default settings
        % Define default settings here as fields of S (i.e S.InitialDelay = 3.2)
        % Note: Any parameters in S.GUI will be shown in UI edit boxes. 
        % See ParameterGUI plugin documentation to show parameters as other UI types (listboxes, checkboxes, buttons, text)
        S.GUI.RewardAmount = 15; % 211010-211029: 4 uL; changed 211010; was 35? before
        S.GUI.PortOutRegDelay = 90; %%% TIME OUT:  How long the mouse must remain out before poking back in (secs)
    end

    %% Define trials
    BpodSystem.Data.TrialTypes = []; % The trial type of each trial completed will be added here.

    %% --- Initialize plots and start USB connections to any modules
%     BpodSystem.ProtocolFigures.OutcomePlotFig = figure('Position', [50 540 1000 200],'name','Outcome plot','numbertitle','off', 'MenuBar', 'none', 'Resize', 'off');
%     BpodSystem.GUIHandles.OutcomePlot = axes('Position', [.075 .3 .89 .6]);
%     TrialTypeOutcomePlot(BpodSystem.GUIHandles.OutcomePlot,'init',[]);
    BpodNotebook('init'); % Initialize Bpod notebook (for manual data annotation)
    BpodParameterGUI('init', S); % Initialize parameter GUI plugin

    %% Main loop (runs once per trial)

    runningExp = 1;
    currentTrial = 1;
    init = false;

    while runningExp
        for trial = [1 2]
            S = BpodParameterGUI('sync', S); % Sync parameters with BpodParameterGUI plugin
            ValveTime = GetValveTimes(S.GUI.RewardAmount, 1); % Update reward amounts
            sma = NewStateMachine();
            if trial == 1
                sma = AddState(sma, 'Name', 'WaitForPoke', ...
                    'Timer', 0,...
                    'StateChangeConditions', {'Port1In', 'Reward1',...
                                              'Port8In', 'Reward8'},...
                    'OutputActions', {});

                sma = AddState(sma, 'Name', 'Reward1', ...
                    'Timer', ValveTime,...
                    'StateChangeConditions', {'Tup',  'LightOn1'},...
                    'OutputActions', {'Valve1', 1}); 
                sma = AddState(sma, 'Name',  'LightOn1', ...
                    'Timer', 1,...
                    'StateChangeConditions', {'Tup', 'exit'},...
                    'OutputActions', {'PWM1', 255}); 
                
                sma = AddState(sma, 'Name', 'Reward8', ...
                    'Timer', ValveTime,...
                    'StateChangeConditions', {'Tup',  'LightOn8'},...
                    'OutputActions', {'Valve8', 1}); 
                sma = AddState(sma, 'Name',  'LightOn8', ...
                    'Timer', 1,...
                    'StateChangeConditions', {'Tup', 'WaitForPoke'},...
                    'OutputActions', {'PWM8', 255}); 
            
            elseif ~init
                sma = AddState(sma, 'Name', 'WaitForPoke', ...
                    'Timer', 0,...
                    'StateChangeConditions', {'Port2In', 'Reward2',...
                                              'Port3In', 'Reward3'},...
                    'OutputActions', {});

                sma = AddState(sma, 'Name','Reward2', ...
                    'Timer', ValveTime,...
                    'StateChangeConditions', {'Tup',  'LightOn2'},...
                    'OutputActions', {'Valve2', 1});
                sma = AddState(sma, 'Name',  'LightOn2', ...
                    'Timer', 1,...
                    'StateChangeConditions', {'Tup', 'exit'},...
                    'OutputActions', {'PWM2', 255});
                
                sma = AddState(sma, 'Name','Reward3', ...
                    'Timer', ValveTime,...
                    'StateChangeConditions', {'Tup',  'LightOn3'},...
                    'OutputActions', {'Valve3', 1});
                sma = AddState(sma, 'Name',  'LightOn3', ...
                    'Timer', 1,...
                    'StateChangeConditions', {'Tup', 'exit'},...
                    'OutputActions', {'PWM3', 255});
            else
                r_port = mod(PREV_REWARDED - 1, 2) + 2;
                REWARDED_PORT_IN = sprintf('Port%dIn', r_port); 
                REWARDED_VALVE = sprintf('Valve%d', r_port);
                LED_WIRE = sprintf('PWM%d', r_port);
                RewardN = sprintf('Reward%d',r_port);
                LightOn = sprintf('LightOn%d',r_port);
                
                sma = AddState(sma, 'Name', 'WaitForPoke', ...
                    'Timer', 0,...
                    'StateChangeConditions', {REWARDED_PORT_IN, RewardN},...
                    'OutputActions', {});

                sma = AddState(sma, 'Name',RewardN, ...
                    'Timer', ValveTime,...
                    'StateChangeConditions', {'Tup',  LightOn},...
                    'OutputActions', {REWARDED_VALVE, 1}); 
                sma = AddState(sma, 'Name',  LightOn, ...
                    'Timer', 1,...
                    'StateChangeConditions', {'Tup', 'exit'},...
                    'OutputActions', {LED_WIRE, 255});    
            end


            SendStateMatrix(sma); % Send state machine to the Bpod state machine device
            RawEvents = RunStateMatrix; % Run the trial and return events



            %--- Package and save the trial's data, update plots
            if ~isempty(fieldnames(RawEvents)) % If you didn't stop the session manually mid-trial
                BpodSystem.Data = AddTrialEvents(BpodSystem.Data,RawEvents); % Adds raw events to a human-readable data struct
                BpodSystem.Data = BpodNotebook('sync', BpodSystem.Data); % Sync with Bpod notebook plugin
                BpodSystem.Data.TrialSettings(currentTrial) = S; % Adds the settings used for the current trial to the Data struct (to be saved after the trial ends)
                BpodSystem.Data.TrialTypes(currentTrial) = trial; % Adds the trial type of the current trial to data
%                 UpdateOutcomePlot(TrialTypes, BpodSystem.Data); %%% update online plots using the newly updated BpodSystem.Data
                SaveBpodSessionData; % Saves the field BpodSystem.Data to the current data file
            end
            
            currentTrial = currentTrial + 1; 
            if trial == 2
                if ~init
                    t2States = BpodSystem.Data.RawEvents.Trial{2}.States;
                    if any(isnan(t2States.Reward2))
                        PREV_REWARDED = 3;
                    else
                        PREV_REWARDED = 2;
                    end
                    init = true;
                else
                    PREV_REWARDED = r_port;
                end
            end
            %--- This final block of code is necessary for the Bpod console's pause and stop buttons to work
            HandlePauseCondition; % Checks to see if the protocol is paused. If so, waits until user resumes.
            if BpodSystem.Status.BeingUsed == 0
                return
            end
        end
    end
end


% function UpdateOutcomePlot(TrialTypes, Data)
%     global BpodSystem
%     Outcomes = zeros(1,Data.nTrials);
%     for x = 1:Data.nTrials
%         if ~isnan(Data.RawEvents.Trial{x}.States.LightOn(1))
%             Outcomes(x) = 1;
%         else
%             Outcomes(x) = 3;
%         end
%     end
%     TrialTypeOutcomePlot(BpodSystem.GUIHandles.OutcomePlot,'update',Data.nTrials+1,TrialTypes,Outcomes);
% end