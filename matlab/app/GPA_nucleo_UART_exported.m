classdef GPA_nucleo_UART_exported < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        NucleoconnectionUIFigure    matlab.ui.Figure
        IOValuesPanel               matlab.ui.container.Panel
        In2EditField                matlab.ui.control.NumericEditField
        In2Label                    matlab.ui.control.Label
        In1EditField                matlab.ui.control.NumericEditField
        In1Label                    matlab.ui.control.Label
        OutEditField                matlab.ui.control.NumericEditField
        OutEditFieldLabel           matlab.ui.control.Label
        GPAPanel                    matlab.ui.container.Panel
        gpa_meas_startButton        matlab.ui.control.Button
        GPA_N                       matlab.ui.control.NumericEditField
        NLabel                      matlab.ui.control.Label
        GPA_A1                      matlab.ui.control.NumericEditField
        A1Label                     matlab.ui.control.Label
        GPA_A0                      matlab.ui.control.NumericEditField
        A0Label                     matlab.ui.control.Label
        GPA_f0                      matlab.ui.control.NumericEditField
        f0Label                     matlab.ui.control.Label
        GPA_f1                      matlab.ui.control.NumericEditField
        f1Label                     matlab.ui.control.Label
        VarNameEditField            matlab.ui.control.EditField
        VarNameEditFieldLabel       matlab.ui.control.Label
        GPAtext                     matlab.ui.control.TextArea
        GPAonLamp                   matlab.ui.control.Lamp
        OnLabel                     matlab.ui.control.Label
        TxtOutput                   matlab.ui.control.TextArea
        TimemeasuresPanel           matlab.ui.container.Panel
        HzLabel                     matlab.ui.control.Label
        DownsamplingEditField       matlab.ui.control.NumericEditField
        DownsamplingEditFieldLabel  matlab.ui.control.Label
        OffsetEditField             matlab.ui.control.NumericEditField
        OffsetEditFieldLabel        matlab.ui.control.Label
        FreqEditField               matlab.ui.control.NumericEditField
        FreqEditFieldLabel          matlab.ui.control.Label
        AmpEditField                matlab.ui.control.NumericEditField
        AmpEditFieldLabel           matlab.ui.control.Label
        TypeDropDown                matlab.ui.control.DropDown
        TypeDropDownLabel           matlab.ui.control.Label
        VarNameEditField_ti         matlab.ui.control.EditField
        TimeseriesNameLabel         matlab.ui.control.Label
        timeMeasstartButton         matlab.ui.control.Button
        connectionLabel             matlab.ui.control.Label
        baudEditField               matlab.ui.control.NumericEditField
        baudEditFieldLabel          matlab.ui.control.Label
        COMPortsListBox             matlab.ui.control.ListBox
        COMPortsLabel               matlab.ui.control.Label
        V30Label                    matlab.ui.control.Label
        ConnectButton               matlab.ui.control.Button
        SearchCOMPortsButton        matlab.ui.control.Button
    end


    properties (Access = public)
        selectedComPort     string % Selected COM Port
        % Flags
        flagPortSelect = false
        flagPortsFree = false
        flagPortConnect = false
        textposition;
        baudRate
        serialPort
        recData
        GPA_data;
        GPA_i;
        Jumbo;
        loc_data;
    end

    properties (Access = private)
        TimerObj % Description
        data
        N=1e4;
    end




    methods (Access = private)

        function my_callback_fcn(app)
            loc_count = 0;
            while app.serialPort.NumBytesAvailable > 0
                li = readline(app.serialPort); %string
                app.I_li=app.I_li+1;
                loc_count=loc_count+1;
                dum = str2num(li);
                if ~app.array_defined && ~isempty(dum)
                    app.data=zeros(app.N,size(dum,2));
                    app.data(1,:)=dum;
                    app.array_defined = true;
                else
                    app.data(app.I_li,:)=dum;
                end
                %app.reciveLabel.Text=txt;
                if loc_count>0
                    app.TxtOutput.Value{1,:}=sprintf('received %3d lines, %5d in total.',loc_count,app.I_li);
                end
            end
        end

        function sendBinaryData(app,senddat) % binary
            try
                write(app.serialPort,mod(senddat,48),"UINT8");
                %                 write(app.serialPort,'/n',"char");
            catch
            end
        end

        function sendData(app,senddat) % String
            try
                writeline(app.serialPort,senddat);
            catch

            end
        end

        function addmessage(app,txt)
            if app.textposition<6
                app.textposition = app.textposition+1;
                app.TxtOutput.Value(app.textposition) = txt;
            else
                for kk=1:5
                    app.TxtOutput.Value(kk)=app.TxtOutput.Value(kk+1);
                end
                app.TxtOutput.Value(6) = txt;
            end
        end



        function sendCommand(app, cmdId, data)
            cmdHeader = uint8([254 1 255 cmdId]);
            header = [cmdHeader typecast(uint16(length(data)),'uint8')];
            checksum = uint8(mod(sum(header)+sum(data),256));
            terminator = uint8([char(13) char(10)]); %CR/LF / "\r\n"
            uartPackage = [header data checksum terminator];
            %tic;
            write(app.serialPort ,uartPackage,'uint8') % does not send terminator
        end

    end

    methods (Access = public)


    end


    % Callbacks that handle component events
    methods (Access = private)

        % Code that executes after component creation
        function startupFcn(app)
            app.connectionLabel.Text="Not Connected";

            %Timer objekt erstellen
            %app.TimerObj=timer('Period', 0.05, 'TasksToExecute', Inf,'ExecutionMode', 'fixedRate');
            %app.TimerObj.TimerFcn =@(~,~)app.getValuesUART;
            %Im manuellen Modus Starten
            app.textposition = 2;
            app.TxtOutput.Value(3:6)={''};
            app.GPA_i = 0;
            app.loc_data = [];
            app.loc_data.rest_data = [];
            app.Jumbo = [];
        end

        % Button pushed function: SearchCOMPortsButton
        function SearchCOMPortsButtonPushed(app, event)
            ports = serialportlist("available"); % evtl. freeports = serialportlist("available") umschreiben zu try/catch

            if isempty(ports)
                app.COMPortsListBox.Items = "no Ports";
                %app.portsFree=false;
                app.ConnectButton.Enable = 'off';
            else
                app.COMPortsListBox.Items=ports;
                %app.portsFree=true;
                app.ConnectButton.Enable = 'on';
            end
        end

        % Button pushed function: ConnectButton
        function ConnectButtonPushed(app, event)
            app.baudRate=app.baudEditField.Value;
            serialCallback = @(~,~)app.getValuesUART;
            if ((app.baudRate > 0) & (~app.flagPortConnect))
                app.serialPort = serialport(app.selectedComPort,app.baudRate,"Timeout",3);
                % configureTerminator(app.serialPort,"CR/LF","CR/LF");
                % configureCallback(app.serialPort,"terminator",serialCallback);
                configureCallback(app.serialPort,"byte",1,serialCallback); % fire on any byte

                %configureTerminator(app.serialPort,"LF") % Terminator (default "LF")
                app.flagPortConnect = true;
                app.connectionLabel.Text = "Connected";
                app.ConnectButton.Text = "Disconnect";
                %assignin('base', 's', app.serialPort) % export to Workspace
                app.SearchCOMPortsButton.Enable = 'off';
                flush(app.serialPort,"output");flush(app.serialPort,"input");
                %                start(app.TimerObj);

            elseif (app.flagPortConnect)
                clear app.serialPort; % geht nicht
                app.serialPort=[]; % Disconnect serial
                SearchCOMPortsButtonPushed(app,event);
                app.flagPortConnect = false;
                app.connectionLabel.Text = "Not Connected";
                app.ConnectButton.Text = "Connect";
                app.SearchCOMPortsButton.Enable = 'on';

            end
        end

        % Value changed function: COMPortsListBox
        function COMPortsListBoxValueChanged(app, event)
            app.selectedComPort = app.COMPortsListBox.Value;
            app.flagPortSelect = true;
        end

        % Value changed function: baudEditField
        function baudEditFieldValueChanged(app, event)
            if app.baudEditField.Value < 0 % evtl druch Dropdownliste ersetzten
                app.baudRate = 0;
                app.baudEditField.Value = 0;
            elseif app.baudEditField.Value > 0
                app.baudRate = app.baudEditField.Value;
            end
        end

        % Close request function: NucleoconnectionUIFigure
        function NucleoconnectionUIFigureCloseRequest(app, event)

            try
                delete(app.serialPort);
            catch end
            try
                stop(app.TimerObj)
                o=timerfindall;
                delete(o);
            catch end
            try
                delete(serialportfind)
            catch end
            delete(app)
            % try
            %     desktop = com.mathworks.mde.desk.MLDesktop.getInstance;
            %     desktop.bringToFront; % bring MATLAB desktop forward
            % catch end

        end

        % Callback function
        function sendDataButtonPushed(app, event)
            sendData(app,app.sendTextEditField.Value)
        end

        % Callback function
        function ButtonPushed(app, event)

        end

        % Callback function
        function HomeallButtonPushed(app, event)

        end

        % Callback function
        function ChargeSwitchValueChanged(app, event)

        end

        % Callback function
        function UserResetButtonValueChanged(app, event)
            value = app.UserResetButton.Value;
            sendData(app,"360000");
        end

        % Callback function
        function CenterButtonPushed(app, event)
            sendData(app,"420000");
        end

        % Callback function
        function clcButtonPushed(app, event)
            app.TxtOutput.Value={''};
        end

        % Callback function
        function send2nucButtonPushed(app, event)
            sendData(app,app.TxtOutput.Value)
        end

        % Callback function
        function sendWSButtonPushed(app, event)

        end

        % Callback function
        function getValuesUART(app, event)
            try
                app.N = app.serialPort.NumBytesAvailable;
                % app.N
                if app.N>0
                    if isfield(app.loc_data,'rest_data')
                        da = [app.loc_data.rest_data,uint8(read(app.serialPort,app.N,'uint8'))];
                    else
                        da = uint8(read(app.serialPort,app.N,'uint8'));
                    end
                    app.loc_data = cut_data(da);
                    for k = 1:app.loc_data(1).N
                        if app.loc_data(k).complete_data
                            switch app.loc_data(k).id1
                                case 101
                                    switch app.loc_data(k).id2
                                        case 12
                                            val = double(typecast(uint8(app.loc_data(k).grab), 'single'))';
                                            app.Phi1_act.Value=val(1);
                                            dum=typecast(int64(round(val(1)*4000/2/pi)),'int16');
                                            app.Phi1_act_inc.Value=double(dum(1));
                                            app.Phi2_act.Value=val(2);
                                            dum=typecast(int64(round(val(2)*4000/2/pi)),'int16');
                                            app.Phi2_act_inc.Value=double(dum(1));
                                        case 34
                                            val = double(typecast(uint8(app.loc_data(k).grab), 'single'))';
                                            app.X_act.Value=val(1);
                                            app.Y_act.Value=val(2);
                                            if (val(1)<65 && val(1)>-65 && val(2)<65 && val(2)>-55)
                                                app.laserpoint.XData=val(1);
                                                app.laserpoint.YData=val(2);
                                                app.laserpoint.MarkerSize = 10;
                                            else
                                                app.laserpoint.MarkerSize = 1;
                                            end

                                    end
                                case 115
                                    switch app.loc_data(k).id2
                                        case 1
                                            val = double(typecast(uint8(app.loc_data(k).grab), 'single'))';
                                            app.OutEditField.Value = val(1);
                                            app.In1EditField.Value = val(2);
                                            app.In2EditField.Value = val(3);
                                    end
                                case 202
                                    val = double(typecast(uint8(app.loc_data(k).grab), 'single'))';
                                    switch app.loc_data(k).id2
                                        case 12
                                            app.Phi1_des.Value=val(1);
                                            app.Phi2_des.Value=val(2);
                                        case 34
                                            app.X_des.Value=val(1);
                                            app.Y_des.Value=val(2);
                                    end
                                case 125
                                    switch app.loc_data(k).id2
                                        case 1
                                            val = double(typecast(uint8(app.loc_data(k).grab), 'single'))';
                                    end
                                case 241
                                    switch app.loc_data(k).id2
                                        case 1
                                            app.addmessage({char(typecast(uint8(app.loc_data(k).grab), 'uint8'))});
                                    end
                                case 210
                                    % if app.loc_data(k).id2 < 99
                                    %     %disp(app.loc_data(k).id2)
                                    %     packet = double(app.loc_data(k).id2);
                                    %     app.Jumbo=[app.Jumbo,app.loc_data(k).grab];
                                    % end
                                    % if app.loc_data(k).id2 ==99
                                    %     val = double(typecast(uint8(app.Jumbo), 'single'));
                                    %     assignin('base','rawdata',app.Jumbo);
                                    %     app.Jumbo = [];
                                    %     NCOL = 4;
                                    %     val = reshape(val,NCOL,length(val)/NCOL)';
                                    %     timedata.time = val(:,1);
                                    %     timedata.values = val(:,2:NCOL);
                                    %     assignin('base',app.VarNameEditField_ti.Value,timedata);
                                    %     addmessage(app,{'sent data to WS'})
                                    % end
                                    if app.loc_data(k).id2 == 99
                                        val = double(typecast(uint8(app.Jumbo), 'single'));
                                        assignin('base','rawdata',app.Jumbo);
                                        app.Jumbo = [];
                                        NCOL = 7;
                                        val = reshape(val,NCOL,length(val)/NCOL)';
                                        assignin('base','rawtime',val(:,1));
                                        timedata.time = (0:length(val(:,1))-1).' * round(mean(diff(val(:,1))) * 1e5) * 1e-5; % round to 10 mus
                                        timedata.values = val(:,2:NCOL);
                                        assignin('base',app.VarNameEditField_ti.Value,timedata);
                                        addmessage(app,{'Time data received (Workspace)'})
                                    else
                                        app.Jumbo=[app.Jumbo,app.loc_data(k).grab];
                                    end
                                case 250
                                    switch app.loc_data(k).id2
                                        case 1  % seven floats: frequency, real/imag of U, Y and R
                                            try
                                                if app.loc_data(k).N_bytes ~= 28
                                                    error('GPA:Data', 'Expected 28 GPA bytes. Update firmware and GUI together.');
                                                end
                                                val = double(typecast(uint8(app.loc_data(k).grab), 'single'))';
                                                app.GPA_i = app.GPA_i+1;
                                                app.GPA_data(app.GPA_i,:)=val;
                                                %disp('received')
                                                app.GPAtext.Value = sprintf('Rec. val. @ %4.2f Hz',val(1));
                                                app.GPAonLamp.Color =[0.93,0.69,0.13];
                                            catch ME
                                                warning('UART:Receive', '%s', ME.message);
                                                %disp(head)
                                                %disp(dat(1:end-2))
                                            end
                                        case 255  % meas finished
                                            assignin('base','GPA_data',app.GPA_data)
                                            % Columns: frequency, real/imag of U, Y and R.
                                            if isempty(app.GPA_data) || size(app.GPA_data,2) ~= 7 || any(~isfinite(app.GPA_data), 'all')
                                                error('GPA:Data', 'GPA transfer functions need nonempty, finite seven-column data.');
                                            end
                                            U = app.GPA_data(:,2) + 1i*app.GPA_data(:,3);
                                            Y = app.GPA_data(:,4) + 1i*app.GPA_data(:,5);
                                            R = app.GPA_data(:,6) + 1i*app.GPA_data(:,7);
                                            if any(U == 0) || any(R == 0)
                                                error('GPA:Data', 'GPA transfer functions are undefined: zero U or R.');
                                            end
                                            Gur = frd(U ./ R, app.GPA_data(:,1), 'FrequencyUnit', 'Hz');
                                            Gyr = frd(Y ./ R, app.GPA_data(:,1), 'FrequencyUnit', 'Hz');
                                            G = Gyr / Gur; % Gyu; U is the signal passed to GPA in firmware.
                                            assignin('base','Gur',Gur)
                                            assignin('base','Gyr',Gyr)
                                            assignin('base',app.VarNameEditField.Value,G)
                                            app.GPAtext.Value = sprintf('Frd with %d pts in WS',size(app.GPA_data,1));
                                            app.GPAonLamp.Color =[0 1 0];
                                            addmessage(app,{'FRD data received (Workspace)'})
                                        case 2      % reset GPA
                                            app.GPA_i = 0;
                                            app.GPA_data = [];
                                        case 3
                                            %disp(app.loc_data(k).grab
                                            %app.received_packages
                                    end
                            end
                        end
                    end
                end
            catch ME
                warning('UART:Receive', '%s', ME.message);
                addmessage(app, {ME.message});
            end
        end

        % Button pushed function: timeMeasstartButton
        function timeMeasstartButtonPushed(app, event)
            va = find(strcmp(app.TypeDropDown.Items,app.TypeDropDown.Value));
            dat = [typecast(uint8(va),'uint8') typecast(single(app.AmpEditField.Value),'uint8') ...
                typecast(single(app.FreqEditField.Value*2*pi),'uint8') typecast(single(app.OffsetEditField.Value),'uint8') typecast(uint8(app.DownsamplingEditField.Value),'uint8')];
            sendCommand(app,[210 101],dat);
        end

        % Button pushed function: gpa_meas_startButton
        function gpa_meas_startButtonPushed(app, event)
            values = double(single([app.GPA_f0.Value app.GPA_f1.Value app.GPA_A0.Value app.GPA_A1.Value app.GPA_N.Value]));
            if any(~isfinite(values)) || any(values(1:4) <= 0) || values(1) >= values(2) ...
                    || values(5) < 2 || values(5) > 100 || values(5) ~= round(values(5))
                addmessage(app, {'GPA: use 2-100 points, positive amplitudes and 0 < f0 < f1.'});
                return
            end
            inverse_span = single(1)/single(values(2)) - single(1)/single(values(1));
            if inverse_span == 0 || ~isfinite(inverse_span)
                addmessage(app, {'GPA: frequency limits are too close for single precision.'});
                return
            end
            a = (single(values(4))-single(values(3))) / inverse_span;
            b = single(values(3)) - a/single(values(1));
            if ~isfinite(a) || ~isfinite(b)
                addmessage(app, {'GPA: amplitudes/frequency limits exceed single precision.'});
                return
            end
            % Firmware also checks its sampling-dependent Nyquist limit.
            dat = [typecast(single(app.GPA_f0.Value),'uint8') typecast(single(app.GPA_f1.Value),'uint8')...
                typecast(single(app.GPA_A0.Value),'uint8') typecast(single(app.GPA_A1.Value),'uint8') typecast(uint8(app.GPA_N.Value),'uint8')];
            sendCommand(app,[250 101],dat);
        end

        % Value changed function: VarNameEditField_ti
        function VarNameEditField_tiValueChanged(app, event)
            value = app.VarNameEditField_ti.Value;

        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create NucleoconnectionUIFigure and hide until all components are created
            app.NucleoconnectionUIFigure = uifigure('Visible', 'off');
            colormap(app.NucleoconnectionUIFigure, 'parula');
            app.NucleoconnectionUIFigure.Position = [300 300 694 334];
            app.NucleoconnectionUIFigure.Name = 'Nucleo connection';
            app.NucleoconnectionUIFigure.CloseRequestFcn = createCallbackFcn(app, @NucleoconnectionUIFigureCloseRequest, true);

            % Create SearchCOMPortsButton
            app.SearchCOMPortsButton = uibutton(app.NucleoconnectionUIFigure, 'push');
            app.SearchCOMPortsButton.ButtonPushedFcn = createCallbackFcn(app, @SearchCOMPortsButtonPushed, true);
            app.SearchCOMPortsButton.Position = [50 198 123 23];
            app.SearchCOMPortsButton.Text = 'Search COM Ports';

            % Create ConnectButton
            app.ConnectButton = uibutton(app.NucleoconnectionUIFigure, 'push');
            app.ConnectButton.ButtonPushedFcn = createCallbackFcn(app, @ConnectButtonPushed, true);
            app.ConnectButton.Enable = 'off';
            app.ConnectButton.Position = [184 198 86 23];
            app.ConnectButton.Text = 'Connect';

            % Create V30Label
            app.V30Label = uilabel(app.NucleoconnectionUIFigure);
            app.V30Label.FontColor = [0.651 0.651 0.651];
            app.V30Label.Position = [12 1 30 22];
            app.V30Label.Text = 'V3.0';

            % Create COMPortsLabel
            app.COMPortsLabel = uilabel(app.NucleoconnectionUIFigure);
            app.COMPortsLabel.HorizontalAlignment = 'right';
            app.COMPortsLabel.Position = [12 252 31 53];
            app.COMPortsLabel.Text = {'COM'; 'Ports'};

            % Create COMPortsListBox
            app.COMPortsListBox = uilistbox(app.NucleoconnectionUIFigure);
            app.COMPortsListBox.Items = {};
            app.COMPortsListBox.ValueChangedFcn = createCallbackFcn(app, @COMPortsListBoxValueChanged, true);
            app.COMPortsListBox.Position = [50 232 123 92];
            app.COMPortsListBox.Value = {};

            % Create baudEditFieldLabel
            app.baudEditFieldLabel = uilabel(app.NucleoconnectionUIFigure);
            app.baudEditFieldLabel.HorizontalAlignment = 'right';
            app.baudEditFieldLabel.Position = [231 301 32 22];
            app.baudEditFieldLabel.Text = 'baud';

            % Create baudEditField
            app.baudEditField = uieditfield(app.NucleoconnectionUIFigure, 'numeric');
            app.baudEditField.Limits = [0 2560000];
            app.baudEditField.ValueDisplayFormat = '%.0f';
            app.baudEditField.ValueChangedFcn = createCallbackFcn(app, @baudEditFieldValueChanged, true);
            app.baudEditField.Position = [180 301 54 22];
            app.baudEditField.Value = 115200;

            % Create connectionLabel
            app.connectionLabel = uilabel(app.NucleoconnectionUIFigure);
            app.connectionLabel.Position = [184 232 100 22];
            app.connectionLabel.Text = '---';

            % Create TimemeasuresPanel
            app.TimemeasuresPanel = uipanel(app.NucleoconnectionUIFigure);
            app.TimemeasuresPanel.Title = 'Time measures';
            app.TimemeasuresPanel.Position = [293 27 246 159];

            % Create timeMeasstartButton
            app.timeMeasstartButton = uibutton(app.TimemeasuresPanel, 'push');
            app.timeMeasstartButton.ButtonPushedFcn = createCallbackFcn(app, @timeMeasstartButtonPushed, true);
            app.timeMeasstartButton.Position = [191 83 40 23];
            app.timeMeasstartButton.Text = 'Start';

            % Create TimeseriesNameLabel
            app.TimeseriesNameLabel = uilabel(app.TimemeasuresPanel);
            app.TimeseriesNameLabel.HorizontalAlignment = 'right';
            app.TimeseriesNameLabel.Position = [59 8 99 22];
            app.TimeseriesNameLabel.Text = 'Timeseries Name';

            % Create VarNameEditField_ti
            app.VarNameEditField_ti = uieditfield(app.TimemeasuresPanel, 'text');
            app.VarNameEditField_ti.ValueChangedFcn = createCallbackFcn(app, @VarNameEditField_tiValueChanged, true);
            app.VarNameEditField_ti.Position = [172 8 59 22];
            app.VarNameEditField_ti.Value = 'data';

            % Create TypeDropDownLabel
            app.TypeDropDownLabel = uilabel(app.TimemeasuresPanel);
            app.TypeDropDownLabel.HorizontalAlignment = 'right';
            app.TypeDropDownLabel.Position = [60 111 32 22];
            app.TypeDropDownLabel.Text = 'Type';

            % Create TypeDropDown
            app.TypeDropDown = uidropdown(app.TimemeasuresPanel);
            app.TypeDropDown.Items = {'Step Sequence', 'Sine Wave', 'ZigZag'};
            app.TypeDropDown.Position = [101 112 130 22];
            app.TypeDropDown.Value = 'Step Sequence';

            % Create AmpEditFieldLabel
            app.AmpEditFieldLabel = uilabel(app.TimemeasuresPanel);
            app.AmpEditFieldLabel.HorizontalAlignment = 'right';
            app.AmpEditFieldLabel.Position = [19 87 34 22];
            app.AmpEditFieldLabel.Text = 'Amp.';

            % Create AmpEditField
            app.AmpEditField = uieditfield(app.TimemeasuresPanel, 'numeric');
            app.AmpEditField.Limits = [-5000 5000];
            app.AmpEditField.Position = [58 87 39 22];
            app.AmpEditField.Value = 0.5;

            % Create FreqEditFieldLabel
            app.FreqEditFieldLabel = uilabel(app.TimemeasuresPanel);
            app.FreqEditFieldLabel.HorizontalAlignment = 'right';
            app.FreqEditFieldLabel.Position = [20 62 34 17];
            app.FreqEditFieldLabel.Text = 'Freq.';

            % Create FreqEditField
            app.FreqEditField = uieditfield(app.TimemeasuresPanel, 'numeric');
            app.FreqEditField.Limits = [0.1 100];
            app.FreqEditField.Position = [59 60 38 22];
            app.FreqEditField.Value = 10;

            % Create OffsetEditFieldLabel
            app.OffsetEditFieldLabel = uilabel(app.TimemeasuresPanel);
            app.OffsetEditFieldLabel.HorizontalAlignment = 'right';
            app.OffsetEditFieldLabel.Position = [18 30 37 22];
            app.OffsetEditFieldLabel.Text = 'Offset';

            % Create OffsetEditField
            app.OffsetEditField = uieditfield(app.TimemeasuresPanel, 'numeric');
            app.OffsetEditField.Limits = [-1000 1000];
            app.OffsetEditField.Position = [60 33 38 22];

            % Create DownsamplingEditFieldLabel
            app.DownsamplingEditFieldLabel = uilabel(app.TimemeasuresPanel);
            app.DownsamplingEditFieldLabel.HorizontalAlignment = 'right';
            app.DownsamplingEditFieldLabel.Position = [121 33 84 22];
            app.DownsamplingEditFieldLabel.Text = 'Downsampling';

            % Create DownsamplingEditField
            app.DownsamplingEditField = uieditfield(app.TimemeasuresPanel, 'numeric');
            app.DownsamplingEditField.Limits = [1 50];
            app.DownsamplingEditField.Position = [208 33 23 22];
            app.DownsamplingEditField.Value = 1;

            % Create HzLabel
            app.HzLabel = uilabel(app.TimemeasuresPanel);
            app.HzLabel.Position = [101 59 25 22];
            app.HzLabel.Text = 'Hz';

            % Create TxtOutput
            app.TxtOutput = uitextarea(app.NucleoconnectionUIFigure);
            app.TxtOutput.FontName = 'Courier New';
            app.TxtOutput.FontSize = 11;
            app.TxtOutput.FontColor = [0 1 0];
            app.TxtOutput.BackgroundColor = [0 0 0];
            app.TxtOutput.Position = [293 198 387 126];
            app.TxtOutput.Value = {'Nucleo Communication'; '--------------------'};

            % Create GPAPanel
            app.GPAPanel = uipanel(app.NucleoconnectionUIFigure);
            app.GPAPanel.Title = 'GPA';
            app.GPAPanel.Position = [12 27 258 159];

            % Create OnLabel
            app.OnLabel = uilabel(app.GPAPanel);
            app.OnLabel.HorizontalAlignment = 'right';
            app.OnLabel.Position = [10 82 25 22];
            app.OnLabel.Text = 'On';

            % Create GPAonLamp
            app.GPAonLamp = uilamp(app.GPAPanel);
            app.GPAonLamp.Position = [50 82 20 20];
            app.GPAonLamp.Color = [0.8 0.8 0.8];

            % Create GPAtext
            app.GPAtext = uitextarea(app.GPAPanel);
            app.GPAtext.FontName = 'Courier New';
            app.GPAtext.FontSize = 11;
            app.GPAtext.Position = [6 111 162 19];
            app.GPAtext.Value = {'Wait'};

            % Create VarNameEditFieldLabel
            app.VarNameEditFieldLabel = uilabel(app.GPAPanel);
            app.VarNameEditFieldLabel.HorizontalAlignment = 'right';
            app.VarNameEditFieldLabel.Position = [22 51 70 22];
            app.VarNameEditFieldLabel.Text = 'Gyu Name';

            % Create VarNameEditField
            app.VarNameEditField = uieditfield(app.GPAPanel, 'text');
            app.VarNameEditField.Position = [102 51 63 22];
            app.VarNameEditField.Value = 'G_est';

            % Create f1Label
            app.f1Label = uilabel(app.GPAPanel);
            app.f1Label.HorizontalAlignment = 'right';
            app.f1Label.Position = [167 84 25 22];
            app.f1Label.Text = 'f1';

            % Create GPA_f1
            app.GPA_f1 = uieditfield(app.GPAPanel, 'numeric');
            app.GPA_f1.Limits = [1 10000];
            app.GPA_f1.Position = [202 84 42 22];
            app.GPA_f1.Value = 4990;

            % Create f0Label
            app.f0Label = uilabel(app.GPAPanel);
            app.f0Label.HorizontalAlignment = 'right';
            app.f0Label.Position = [167 108 25 22];
            app.f0Label.Text = 'f0';

            % Create GPA_f0
            app.GPA_f0 = uieditfield(app.GPAPanel, 'numeric');
            app.GPA_f0.Limits = [0.01 10000];
            app.GPA_f0.Position = [202 108 42 22];
            app.GPA_f0.Value = 1;

            % Create A0Label
            app.A0Label = uilabel(app.GPAPanel);
            app.A0Label.HorizontalAlignment = 'right';
            app.A0Label.Position = [168 60 25 22];
            app.A0Label.Text = 'A0';

            % Create GPA_A0
            app.GPA_A0 = uieditfield(app.GPAPanel, 'numeric');
            app.GPA_A0.Limits = [0 Inf];
            app.GPA_A0.Position = [202 60 42 22];
            app.GPA_A0.Value = 1;

            % Create A1Label
            app.A1Label = uilabel(app.GPAPanel);
            app.A1Label.HorizontalAlignment = 'right';
            app.A1Label.Position = [168 36 25 22];
            app.A1Label.Text = 'A1';

            % Create GPA_A1
            app.GPA_A1 = uieditfield(app.GPAPanel, 'numeric');
            app.GPA_A1.Limits = [0 Inf];
            app.GPA_A1.Position = [202 36 42 22];
            app.GPA_A1.Value = 1;

            % Create NLabel
            app.NLabel = uilabel(app.GPAPanel);
            app.NLabel.HorizontalAlignment = 'right';
            app.NLabel.Position = [168 10 25 22];
            app.NLabel.Text = 'N';

            % Create GPA_N
            app.GPA_N = uieditfield(app.GPAPanel, 'numeric');
            app.GPA_N.Limits = [2 100];
            app.GPA_N.RoundFractionalValues = 'on';
            app.GPA_N.Position = [202 10 42 22];
            app.GPA_N.Value = 30;

            % Create gpa_meas_startButton
            app.gpa_meas_startButton = uibutton(app.GPAPanel, 'push');
            app.gpa_meas_startButton.ButtonPushedFcn = createCallbackFcn(app, @gpa_meas_startButtonPushed, true);
            app.gpa_meas_startButton.Position = [128 82 40 23];
            app.gpa_meas_startButton.Text = 'Start';

            % Create IOValuesPanel
            app.IOValuesPanel = uipanel(app.NucleoconnectionUIFigure);
            app.IOValuesPanel.Title = 'I/O Values';
            app.IOValuesPanel.Position = [556 27 124 159];

            % Create OutEditFieldLabel
            app.OutEditFieldLabel = uilabel(app.IOValuesPanel);
            app.OutEditFieldLabel.HorizontalAlignment = 'right';
            app.OutEditFieldLabel.Position = [12 97 25 22];
            app.OutEditFieldLabel.Text = 'Out';

            % Create OutEditField
            app.OutEditField = uieditfield(app.IOValuesPanel, 'numeric');
            app.OutEditField.ValueDisplayFormat = '%.3f';
            app.OutEditField.Position = [45 97 69 22];

            % Create In1Label
            app.In1Label = uilabel(app.IOValuesPanel);
            app.In1Label.HorizontalAlignment = 'right';
            app.In1Label.Position = [13 67 25 22];
            app.In1Label.Text = 'In1';

            % Create In1EditField
            app.In1EditField = uieditfield(app.IOValuesPanel, 'numeric');
            app.In1EditField.ValueDisplayFormat = '%.3f';
            app.In1EditField.Position = [45 70 69 22];

            % Create In2Label
            app.In2Label = uilabel(app.IOValuesPanel);
            app.In2Label.HorizontalAlignment = 'right';
            app.In2Label.Position = [14 40 25 22];
            app.In2Label.Text = 'In2';

            % Create In2EditField
            app.In2EditField = uieditfield(app.IOValuesPanel, 'numeric');
            app.In2EditField.Limits = [-1000 1000];
            app.In2EditField.ValueDisplayFormat = '%.3f';
            app.In2EditField.Position = [45 43 70 22];

            % Show the figure after all components are created
            app.NucleoconnectionUIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = GPA_nucleo_UART_exported

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.NucleoconnectionUIFigure)

            % Execute the startup function
            runStartupFcn(app, @startupFcn)

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.NucleoconnectionUIFigure)
        end
    end
end