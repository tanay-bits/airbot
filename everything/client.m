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
    fprintf('\nAirBot INTERFACE\n\n');
    % display the menu options; this list will grow
    fprintf('p: Unpower motors\n');
    fprintf('e: Equalize motors\n');
    fprintf('r: Read and display yaw\n');
    fprintf('?: Get controller parameters\n');
    fprintf('s: Set PID gains\n');
    fprintf('l: Read Lighthouse angles\n');
    fprintf('m: Manually change motor speeds\n');
    fprintf('h: Go to target yaw and hold\n');
    fprintf('t: Tune gains from disturbance response\n');
%     fprintf('x: Load step trajectory\n');
%     fprintf('y: Load cubic trajectory\n');
%     fprintf('o: Execute trajectory\n');
    fprintf('q: Quit\n');
    
    % read the user's choice
    selection = input('\nENTER COMMAND: ', 's');
    
    % send the command to Teensy
    fprintf(mySerial,'%c',selection);
    
    % take the appropriate action
    switch selection
        
        % EXIT CLIENT:
        case 'q'
            has_quit = true;
        
        % UNPOWER THE MOTORS
        case 'p'
            in = fscanf(mySerial);
            fprintf('\n%s\n', in);
            
        % EQUALIZE MOTORS TO MAX_SPEED/2
        case 'e'
            in = fscanf(mySerial);
            fprintf('\n%s\n', in);
            
        % READ AND DISPLAY YAW
        case 'r'
            fprintf('\nCurrent yaw:\n');
            yaw = fscanf(mySerial);
            fprintf('%s\n', yaw); 
            
        % GET CONTROLLER PARAMETERS
        case '?'
            fprintf('\nPID gains are:\n');
            kp = fscanf(mySerial);
            fprintf('Kp = %s\n', kp);            
            ki = fscanf(mySerial);
            fprintf('Ki = %s\n', ki);            
            kd = fscanf(mySerial);
            fprintf('Kd = %s\n', kd);
            
            fprintf('\nyawTol is:\n');
            yawTol = fscanf(mySerial);
            fprintf('%s\n', yawTol);
            
            fprintf('\nEINT_CAP is:\n');
            EINT_CAP = fscanf(mySerial);
            fprintf('%s\n', EINT_CAP);
            
            fprintf('\nThe mode is\n');
            mode = fscanf(mySerial);
            fprintf('%s\n', mode);
            
        % SET PID GAINS
        case 's'
            p = input('\nEnter P gain: ', 's');
            fprintf(mySerial, '%s', p);  
            
            i = input('\nEnter I gain: ', 's');
            fprintf(mySerial, '%s', i);
            
            d = input('\nEnter D gain: ', 's');
            fprintf(mySerial, '%s', d);
            
            fprintf('\nThe new gains are:\n');
            kp = fscanf(mySerial);
            fprintf('Kp = %s\n', kp);
            ki = fscanf(mySerial);
            fprintf('Ki = %s\n', ki);
            kd = fscanf(mySerial);
            fprintf('Kd = %s\n', kd);
            
        % GO TO TARGET YAW AND HOLD
        case 'h'
            yawChange = input('\nEnter desired yaw change (deg): ', 's');
            fprintf(mySerial, '%s', yawChange);
            in = input('\nPress enter to go back to IDLE mode', 's');
            fprintf(mySerial, '%s', in);
            
        % MANUALLY CHANGE MOTOR SPEEDS
        case 'm'
            ms = input('\Enter motor speeds separated by whitespace:: ', 's');
            fprintf(mySerial, '%s', ms);  

        % TUNE GAINS FROM DISTURBANCE RESPONSE
        case 't'
            fprintf('\nYou are in HOLD (tune) mode. Disturb and observe.\n');
            in = input('Press enter to go back to IDLE mode', 's');
            fprintf(mySerial, '%s', in);
            
        % LOAD STEP TRAJECTORY:
        case 'x'                         
            des_traj = input('\nEnter step trajectory, in sec and degrees [time1, ang1; time2, ang2; ...]: ');
            step_traj = genRef(des_traj, 'step');
            num_samples = size(step_traj, 2);
            if num_samples > 450
                fprintf('\nError: Maximum trajectory time is 10 seconds.\n'); 
            end

            fprintf(mySerial, '%d\n', num_samples);  
            for i=1:num_samples
                fprintf(mySerial, '%d\n',step_traj(i));
            end
            fprintf('Plotting the desired trajectory and sending to Teensy ... completed.\n')

        % LOAD CUBIC TRAJECTORY:
        case 'y'                         
            des_traj = input('\nEnter cubic trajectory, in sec and degrees [time1, ang1; time2, ang2; ...]: ');
            step_traj = genRef(des_traj, 'cubic');
            num_samples = size(step_traj, 2);
            if num_samples > 450
                fprintf('\nError: Maximum trajectory time is 10 seconds.\n'); 
            end

            fprintf(mySerial, '%d\n', num_samples);  
            for i=1:num_samples
                fprintf(mySerial, '%d\n',step_traj(i));
            end
            fprintf('Plotting the desired trajectory and sending to Teensy ... completed.\n')

        % EXECUTE TRAJECTORY AND PLOT:
        case 'o'                         
%             read_plot_matrix_pos(mySerial);
            msg = fscanf(mySerial);
            fprintf('%s\n', msg);
            in = input('\nPress enter to go back to IDLE mode', 's');
            fprintf(mySerial, '%s', in);
        
        % READ ANGLES AND TRIANGULATE: 
        case 'l'                         
            h1 = fscanf(mySerial);
            fprintf('h1 = %s\n', h1);
            
            v1 = fscanf(mySerial);
            fprintf('v1 = %s\n', v1);
            
            h2 = fscanf(mySerial);
            fprintf('h2 = %s\n', h2);
            
            v2 = fscanf(mySerial);
            fprintf('v2 = %s\n', v2);
            
            h3 = fscanf(mySerial);
            fprintf('h3 = %s\n', h3);
                            
            v3 = fscanf(mySerial);
            fprintf('v3 = %s\n', v3);
            
            xy = triangulate(h1,h2,h3,v1,v2,v3,AB,BC,AC);
            Ax = xy[1,1];
            Ay = xy[2,1];
            Bx = xy[1,2];
            By = xy[2,2];
            Cx = xy[1,3];
            Cy = xy[2,3];
            
            fprintf('A: %d , %d\n', Ax,Ay);
            fprintf('B: %d , %d\n', Bx,By);
            fprintf('C: %d , %d\n', Cx,Cy);
            
        otherwise
            fprintf('Invalid Selection %c\n', selection);
    end
end
end

