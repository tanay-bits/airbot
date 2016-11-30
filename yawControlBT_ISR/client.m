function client()
%   client(port)
%
%   Input Arguments:
%       port - the name of the com port.  This should be the same as what
%               you use in screen or putty in quotes ' '
%
%   Example:
%       client('/dev/ttyUSB0') (Linux/Mac)
%       client('/dev/rfcomm0') (Linux - Bluetooth)
%       client('COM3') (PC)
%
%   For convenience, you may want to change this so that the port is hardcoded.
   
% Opening COM connection
if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

port = '/dev/rfcomm0';
fprintf('Opening port %s....\n',port);

% settings for opening the serial port. baud rate 9600, hardware flow control
% wait up to 120 seconds for data before timing out
mySerial = serial(port, 'BaudRate', 9600, 'FlowControl', 'hardware','Timeout',120); 
% opens serial connection
fopen(mySerial);
% closes serial port when function exits
clean = onCleanup(@()fclose(mySerial));                                 

has_quit = false;
% menu loop
while ~has_quit
    fprintf('\n\nAirBot INTERFACE\n\n');
    % display the menu options; this list will grow
    fprintf('l: Read Lighthouse angles\n');
    fprintf('m: Load step trajectory\n');
    fprintf('n: Load cubic trajectory\n');
    fprintf('o: Execute trajectory\n');
    fprintf('q: Quit\n');
    
    % read the user's choice
    selection = input('\nENTER COMMAND: ', 's');
    
    % send the command to Teensy
    fprintf(mySerial,'%c',selection);
    
    % take the appropriate action
    switch selection
        
        % LOAD STEP TRAJECTORY:
        case 'm'                         
            des_traj = input('\nEnter step trajectory, in sec and degrees [time1, ang1; time2, ang2; ...]: ');
            step_traj = genRef(des_traj, 'step');
            num_samples = size(step_traj, 2);
            if num_samples > 2000
                fprintf('\nError: Maximum trajectory time is 10 seconds.\n'); 
            end

            fprintf(mySerial, '%d\n', num_samples);  
            for i=1:num_samples
                fprintf(mySerial, '%d\n',step_traj(i));
            end
            fprintf('Plotting the desired trajectory and sending to PIC32 ... completed.\n')

        % LOAD CUBIC TRAJECTORY:
        case 'n'                         
            des_traj = input('\nEnter cubic trajectory, in sec and degrees [time1, ang1; time2, ang2; ...]: ');
            step_traj = genRef(des_traj, 'cubic');
            num_samples = size(step_traj, 2);
            if num_samples > 2000
                fprintf('\nError: Maximum trajectory time is 10 seconds.\n'); 
            end

            fprintf(mySerial, '%d\n', num_samples);  
            for i=1:num_samples
                fprintf(mySerial, '%d\n',step_traj(i));
            end
            fprintf('Plotting the desired trajectory and sending to PIC32 ... completed.\n')

        % EXECUTE TRAJECTORY AND PLOT:
        case 'o'                         
            read_plot_matrix_pos(mySerial);
        
        % READ ANGLES: 
        case 'l'                         
            h0 = fscanf(mySerial);
            fprintf('h0 = %s\n', h0);
            
            v0 = fscanf(mySerial);
            fprintf('v0 = %s\n', v0);
            
            h1 = fscanf(mySerial);
            fprintf('h1 = %s\n', h1);
            
            v1 = fscanf(mySerial);
            fprintf('v1 = %s\n', v1);
            
            h2 = fscanf(mySerial);
            fprintf('h2 = %s\n', h2);
                            
            v2 = fscanf(mySerial);
            fprintf('v2 = %s\n', v2);
            
        % EXIT CLIENT:
        case 'q'
            has_quit = true;
            
        otherwise
            fprintf('Invalid Selection %c\n', selection);
    end
end
end

