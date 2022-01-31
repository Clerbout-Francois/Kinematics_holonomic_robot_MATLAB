clear all
close all


%% Description of our 4-wheel holonomous robot
% Understand the movements that can be made : 
% Please refer to the description of the movements according to the direction of rotation of the wheels
% 4 wheels independent of each other

%% Description of the possible movements
%Forward/reverse motion of the robot
%Lateral movements left/right
%Diagonal
%Rotation on itself

%% On initialise la figure de notre robot holonome

% A comparison is created for the infinite loop
arretcomparaison = 0;


% Initialization of the condition for stopping the robot
condition = 0;

% Initialization of the different precisions
precision = 0.1;
precision_rotation=0.01;


% Definition of the axes 
axis ([-40,40,-40,40]);


% Definition of the robot
largeur=5;
demie_largeur = 2.5;
longueur=10;
demie_longueur=5;

%Wheel radius
r = 0.5;


%You have to define the "corners" of the robot in order to make it move then thanks to the different sets

%SCHEMA
% Wheel2-----Wheel1
%       |   |   
%       |   |
%       |   |
%       |   |
% Wheel3-----Wheel4

%ROLLER ORIENTATION DIAGRAM
%     //-----\\
%       |   |   
%       |   |
%       |   |
%       |   |
%     \\-----//

%W1: Engine corner 1
Xw1 = 2.5;
Yw1 = 5;
%W2: Engine corner 2
Xw2 = -2.5;
Yw2 = 5;
%W3: Engine corner 3
Xw3 = -2.5;
Yw3 = -5;
%W4: Engine corner 4
Xw4 = 2.5;
Yw4 = -5;
%G : centre of gravity of the robot (useful for rotation)
Xg = 0;
Yg = 0;

%THETA is the angle between the horizontal (x-axis, x-axis) and the 
%line parallel to the width of the robot (the motor axis) passing through the centre of gravity G of the robot
theta = 0;


%Initialization of the corners and theta angle for the graphical representation
%W1: Engine corner 1
Xw1initial = Xw1;
Yw1initial = Yw1;
%W2: Engine corner 2
Xw2initial = Xw2;
Yw2initial = Yw2;
%W3: Engine corner 3
Xw3initial = Xw3;
Yw3initial = Yw3;
%W4: Engine corner 4
Xw4initial = Xw4;
Yw4initial = Yw4;
%G : centre of gravity of the robot (useful for rotation)
Xginitial = Xg;
Yginitial = Yg;
%theta :
theta_initial = theta;



%See Modeling and Adaptive Control of an Omni Mecanum Wheeled Robot for
%explanation of a, b, l and alpha
b = 4.5;
a = 3;
l = sqrt(a*a + b*b);%Distance from the centre of gravity of the robot to points W1, W2, W3 and W4
alpha = atan(b/a);
%Useful for rotations
demi_diag = sqrt(demie_longueur * demie_longueur + demie_largeur * demie_largeur);
angle_coin = atan(demie_longueur/demie_largeur);


%thetaW1 is the angle between the horizontal and the straight line (GW1). 
%The same applies to thetaW2, thetaW3, thetaW4 according to the clockwise direction (not trigonometric)
%because the positive direction of the rotations is clockwise
thetaW1 = angle_coin;
thetaW2 = pi - angle_coin;
thetaW3 = -pi + angle_coin;
thetaW4 = -angle_coin;

%thetaW
thetaW1initial = thetaW1;
thetaW2initial = thetaW2;
thetaW3initial = thetaW3;
thetaW4initial = thetaW4;

% Initialisation of the robot
AXE1_2 = line([Xw1, Xw2],[Yw1, Yw2], 'Color','g');%front of the robot
AXE2_3 = line([Xw2, Xw3],[Yw2, Yw3], 'Color','b');
AXE3_4 = line([Xw3, Xw4],[Yw3, Yw4], 'Color','r');%rear of the robot
AXE4_1 = line([Xw4, Xw1],[Yw4, Yw1], 'Color','b');


%% We enter the information, the kinematic model of the robot
%theta the angle of the robot with respect to the x-axis
mat1 = [(sqrt(2)/2) (sqrt(2)/2) l*sin(pi/4 - alpha); (sqrt(2)/2) (-sqrt(2)/2) l*sin(pi/4 - alpha); (-sqrt(2)/2) (-sqrt(2)/2) l*sin(pi/4 - alpha); (-sqrt(2)/2) (sqrt(2)/2) l*sin(pi/4 - alpha)];
mat2 = [cos(theta) sin(theta) 0; -sin(theta) cos(theta) 0; 0 0 1];
J = mat1*mat2;
T_J = transpose(J);
J1 = T_J * J;
J_1 = inv(J1);
J_plus = J1\T_J;

%The robot has three degrees of freedom, which is what we want to show in
%this program, simulating its movement as a function of speed input for each wheel

%% Setting up the simulation
% Whether or not to stop the system
prompt0 = 'Arrêt du robot ? Indiquez 0 pour OUI et tout autre chiffre pour NON ';
arretacomparer = input(prompt0);

while arretacomparer ~= arretcomparaison
    % We need the speed of each wheel, it is the user who will choose them
    prompt1 = 'A vous de jouer ! Indiquez les vitesses de chaque moteur en rad/s';
    prompt2 = 'Valeur positive pour un moteur qui tourne dans le sens horaire et valeur négative pour un moteur dans le sens anti-horaire (sens trigo)';
    disp(prompt1);
    disp(prompt2);
    prompt3 = ' ';
    disp(prompt3);
    prompt4 = 'Le déplacement représenté a lieu sur 10 secondes (sauf la rotation sur lui-même) afin d être plus représentatif';
    disp(prompt4);
    prompt5 = ' ';
    disp(prompt5);
    prompt6 = 'Vitesse roue 1 ?';
    prompt7 = 'Vitesse roue 2 ?';
    prompt8 = 'Vitesse roue 3 ?';
    prompt9 = 'Vitesse roue 4 ?';
    
    % The values entered are then assigned to each speed
    V1 = input(prompt6);
    V2 = input(prompt7);
    V3 = input(prompt8);
    V4 = input(prompt9);
    mat_vitesses = [V1 ; V2 ; V3 ; V4];
    
    
    resultat = -(sqrt(2)/2) * r * J_plus * mat_vitesses;
    
    
    vitesse_x = resultat(1); %speed of translation along the x-axis
    vitesse_y = resultat(2); %speed of translation along the y-axis
    vitesse_rotation = -resultat(3); %speed of rotation around the z-axis, the "-" is used to obtain the correct direction of rotation according to the direction of rotation of each of the 4 motors
    pas_x = (vitesse_x * 10)/50; %arbitrarily determined
    pas_y = (vitesse_y * 10)/50; %arbitrarily determined
    pas_rotation = (vitesse_rotation)/50; %arbitrarily determined

    if vitesse_rotation == 0
               
        % Updating the coordinates of the points
        if theta_initial ==0
            %W1: Engine corner 1
            Xw1 = Xw1 + vitesse_x * 10;%pour 10 secondes
            Yw1 = Yw1 + vitesse_y * 10;
            %W2: Engine corner 2
            Xw2 = Xw2 + vitesse_x * 10;
            Yw2 = Yw2 + vitesse_y * 10;
            %W3: Engine corner 3
            Xw3 = Xw3 + vitesse_x * 10;
            Yw3 = Yw3 + vitesse_y * 10;
            %W4: Engine corner 4
            Xw4 = Xw4 + vitesse_x * 10;
            Yw4 = Yw4 + vitesse_y * 10;
            %G : centre of gravity of the robot
            Xg = Xg + vitesse_x * 10;
            Yg = Yg + vitesse_y * 10;
            %Calculation of deltas
            deltaXw1 = abs(Xw1 - Xw1initial);
            deltaYw1 = abs(Yw1 - Yw1initial);
            
            % Updating the coordinates of the points
            while (deltaXw1 >= precision) || (deltaYw1 >= precision)
                set (AXE1_2, 'XData', [Xw1initial + pas_x, Xw2initial + pas_x], 'YData', [Yw1initial + pas_y, Yw2initial + pas_y]);
                set (AXE2_3, 'XData', [Xw2initial + pas_x, Xw3initial + pas_x], 'YData', [Yw2initial + pas_y, Yw3initial + pas_y]);
                set (AXE3_4, 'XData', [Xw3initial + pas_x, Xw4initial + pas_x], 'YData', [Yw3initial + pas_y, Yw4initial + pas_y]);
                set (AXE4_1, 'XData', [Xw4initial + pas_x, Xw1initial + pas_x], 'YData', [Yw4initial + pas_y, Yw1initial + pas_y]);
                Xw1initial = Xw1initial + pas_x;
                Xw2initial = Xw2initial + pas_x;
                Xw3initial = Xw3initial + pas_x;
                Xw4initial = Xw4initial + pas_x;
                Yw1initial = Yw1initial + pas_y;
                Yw2initial = Yw2initial + pas_y;
                Yw3initial = Yw3initial + pas_y;
                Yw4initial = Yw4initial + pas_y;
                deltaXw1 = abs(Xw1 - Xw1initial);
                deltaYw1 = abs(Yw1 - Yw1initial);
                % The graphical representation is updated
                drawnow;
            end
        else
            theta_initial;
            modulo_theta = mod(abs(theta_initial), 2 * pi);
            if theta_initial < 0
                modulo_theta = -modulo_theta;
            end
            
            
            %CASE 1 : Quarter circle between 0 and pi/2, angles then plotted according to our situation
            if (0 >= modulo_theta) && (-(pi/2) <= modulo_theta)
                
                %Same formula for both normally, each time working in rectangular triangles
                %Formulas obtained for forward/reverse movement
                formule_xy = sign(vitesse_y) * sqrt(power(pas_y, 2)/(1 + power(tan(pi/2 + theta_initial), 2)));
                formule_yy = sign(vitesse_y) * tan(pi/2 + theta_initial) * sqrt(power(pas_y, 2)/(1 + power(tan(pi/2 + theta_initial), 2)));
                
                %Formulas obtained from a left/right lateral shift
                formule_xx = sign(vitesse_x) * sqrt(power(pas_x, 2)/(1 + power(tan(theta_initial), 2)));
                formule_yx = sign(vitesse_x) * tan(theta_initial) * sqrt(power(pas_x, 2)/(1 + power(tan(theta_initial), 2)));
                
                formule_x = formule_xy + formule_xx; %formula obtained with the sum of the terms according to x
                formule_y = formule_yy + formule_yx; %formula obtained with the sum of the terms according to y
                %W1: Engine corner 1
                Xw1 = Xw1 + formule_x * 5 * 10;%for 10 seconds et *5 because pas = 0.2*déplacement total
                Yw1 = Yw1 + formule_y * 5 * 10;
                %W2: Engine corner 2
                Xw2 = Xw2 + formule_x * 5 * 10;
                Yw2 = Yw2 + formule_y * 5 * 10;
                %W3: Engine corner 3
                Xw3 = Xw3 + formule_x * 5 * 10;
                Yw3 = Yw3 + formule_y * 5 * 10;
                %W4: Engine corner 4
                Xw4 = Xw4 + formule_x * 5 * 10;
                Yw4 = Yw4 + formule_y * 5 * 10;
                %G : centre of gravity of the robot
                Xg = Xg + formule_x * 5 * 10;
                Yg = Yg + formule_y * 5 * 10;
                %Calculation of deltas
                deltaXw1 = abs(Xw1 - Xw1initial);
                deltaYw1 = abs(Yw1 - Yw1initial);
                
                % Updating the coordinates of the points
                while (deltaXw1 >= precision) || (deltaYw1 >= precision)
                    set (AXE1_2, 'XData', [Xw1initial + formule_x, Xw2initial + formule_x], 'YData', [Yw1initial + formule_y, Yw2initial + formule_y]);
                    set (AXE2_3, 'XData', [Xw2initial + formule_x, Xw3initial + formule_x], 'YData', [Yw2initial + formule_y, Yw3initial + formule_y]);
                    set (AXE3_4, 'XData', [Xw3initial + formule_x, Xw4initial + formule_x], 'YData', [Yw3initial + formule_y, Yw4initial + formule_y]);
                    set (AXE4_1, 'XData', [Xw4initial + formule_x, Xw1initial + formule_x], 'YData', [Yw4initial + formule_y, Yw1initial + formule_y]);
                    Xw1initial = Xw1initial + formule_x;
                    Xw2initial = Xw2initial + formule_x;
                    Xw3initial = Xw3initial + formule_x;
                    Xw4initial = Xw4initial + formule_x;
                    Yw1initial = Yw1initial + formule_y;
                    Yw2initial = Yw2initial + formule_y;
                    Yw3initial = Yw3initial + formule_y;
                    Yw4initial = Yw4initial + formule_y;
                    deltaXw1 = abs(Xw1 - Xw1initial);
                    deltaYw1 = abs(Yw1 - Yw1initial);
                    % The graphical representation is updated
                    drawnow;
                end
               
                
            %CASE 2 : Quarter circle between 0 and pi/2, angles then plotted according to our situation
            elseif (-(pi/2) >= modulo_theta) && (-pi <= modulo_theta)
                
                %Same formula for both normally, each time working in rectangular triangles
                %Formulas obtained for forward/reverse movement
                formule_xy = sign(vitesse_y) * sqrt(power(pas_y, 2)/(1 + power(tan(pi/2 + theta_initial), 2)));
                formule_yy = sign(vitesse_y) * tan(pi/2 + theta_initial) * sqrt(power(pas_y, 2)/(1 + power(tan(pi/2 + theta_initial), 2)));
                
                %Formulas obtained from a left/right lateral shift
                formule_xx = -sign(vitesse_x) * sqrt(power(pas_x, 2)/(1 + power(tan(theta_initial), 2)));
                formule_yx = -sign(vitesse_x) * tan(theta_initial) * sqrt(power(pas_x, 2)/(1 + power(tan(theta_initial), 2)));
                
                formule_x = formule_xy + formule_xx; %formula obtained with the sum of the terms according to x
                formule_y = formule_yy + formule_yx; %formula obtained with the sum of the terms according to y
                %W1: Engine corner 1
                Xw1 = Xw1 + formule_x * 5 * 10;%for 10 seconds et *5 because pas = 0.2*déplacement total
                Yw1 = Yw1 + formule_y * 5 * 10;
                %W2: Engine corner 2
                Xw2 = Xw2 + formule_x * 5 * 10;
                Yw2 = Yw2 + formule_y * 5 * 10;
                %W3: Engine corner 3
                Xw3 = Xw3 + formule_x * 5 * 10;
                Yw3 = Yw3 + formule_y * 5 * 10;
                %W4: Engine corner 4
                Xw4 = Xw4 + formule_x * 5 * 10;
                Yw4 = Yw4 + formule_y * 5 * 10;
                %G : centre of gravity of the robot
                Xg = Xg + formule_x * 5 * 10;
                Yg = Yg + formule_y * 5 * 10;
                %Calculation of deltas
                deltaXw1 = abs(Xw1 - Xw1initial);
                deltaYw1 = abs(Yw1 - Yw1initial);
                
                % Updating the coordinates of the points
                while (deltaXw1 >= precision) || (deltaYw1 >= precision)
                    set (AXE1_2, 'XData', [Xw1initial + formule_x, Xw2initial + formule_x], 'YData', [Yw1initial + formule_y, Yw2initial + formule_y]);
                    set (AXE2_3, 'XData', [Xw2initial + formule_x, Xw3initial + formule_x], 'YData', [Yw2initial + formule_y, Yw3initial + formule_y]);
                    set (AXE3_4, 'XData', [Xw3initial + formule_x, Xw4initial + formule_x], 'YData', [Yw3initial + formule_y, Yw4initial + formule_y]);
                    set (AXE4_1, 'XData', [Xw4initial + formule_x, Xw1initial + formule_x], 'YData', [Yw4initial + formule_y, Yw1initial + formule_y]);
                    Xw1initial = Xw1initial + formule_x;
                    Xw2initial = Xw2initial + formule_x;
                    Xw3initial = Xw3initial + formule_x;
                    Xw4initial = Xw4initial + formule_x;
                    Yw1initial = Yw1initial + formule_y;
                    Yw2initial = Yw2initial + formule_y;
                    Yw3initial = Yw3initial + formule_y;
                    Yw4initial = Yw4initial + formule_y;
                    deltaXw1 = abs(Xw1 - Xw1initial);
                    deltaYw1 = abs(Yw1 - Yw1initial);
                    % The graphical representation is updated
                    drawnow;
                end
            
                
            %CASE 3 : Quarter circle between pi/2 and pi, angles then plotted according to our situation
            elseif ((pi/2) >= modulo_theta) && (0 <= modulo_theta)
                
                %Same formula for both normally, each time working in rectangular triangles
                %Formulas obtained for forward/reverse movement
                formule_xy = -sign(vitesse_y) * sqrt(power(pas_y, 2)/(1 + power(tan(pi/2 + theta_initial), 2)));
                formule_yy = -sign(vitesse_y) * tan(pi/2 + theta_initial) * sqrt(power(pas_y, 2)/(1 + power(tan(pi/2 + theta_initial), 2)));
                
                %Formulas obtained from a left/right lateral shift
                formule_xx = sign(vitesse_x) * sqrt(power(pas_x, 2)/(1 + power(tan(theta_initial), 2)));
                formule_yx = sign(vitesse_x) * tan(theta_initial) * sqrt(power(pas_x, 2)/(1 + power(tan(theta_initial), 2)));
                
                formule_x = formule_xy + formule_xx; %formula obtained with the sum of the terms according to x
                formule_y = formule_yy + formule_yx; %formula obtained with the sum of the terms according to y
                
                %W1: Engine corner 1
                Xw1 = Xw1 + formule_x * 5 * 10;%pour 10 secondes
                Yw1 = Yw1 + formule_y * 5 * 10;
                %W2: Engine corner 2
                Xw2 = Xw2 + formule_x * 5 * 10;
                Yw2 = Yw2 + formule_y * 5 * 10;
                %W3: Engine corner 3
                Xw3 = Xw3 + formule_x * 5 * 10;
                Yw3 = Yw3 + formule_y * 5 * 10;
                %W4: Engine corner 4
                Xw4 = Xw4 + formule_x * 5 * 10;
                Yw4 = Yw4 + formule_y * 5 * 10;
                %G : centre of gravity of the robot
                Xg = Xg + formule_x * 5 * 10;
                Yg = Yg + formule_y * 5 * 10;
                %Calculation of deltas
                deltaXw1 = abs(Xw1 - Xw1initial);
                deltaYw1 = abs(Yw1 - Yw1initial);
                
                % Updating the coordinates of the points
                while (deltaXw1 >= precision) || (deltaYw1 >= precision)
                    set (AXE1_2, 'XData', [Xw1initial + formule_x, Xw2initial + formule_x], 'YData', [Yw1initial + formule_y, Yw2initial + formule_y]);
                    set (AXE2_3, 'XData', [Xw2initial + formule_x, Xw3initial + formule_x], 'YData', [Yw2initial + formule_y, Yw3initial + formule_y]);
                    set (AXE3_4, 'XData', [Xw3initial + formule_x, Xw4initial + formule_x], 'YData', [Yw3initial + formule_y, Yw4initial + formule_y]);
                    set (AXE4_1, 'XData', [Xw4initial + formule_x, Xw1initial + formule_x], 'YData', [Yw4initial + formule_y, Yw1initial + formule_y]);
                    Xw1initial = Xw1initial + formule_x;
                    Xw2initial = Xw2initial + formule_x;
                    Xw3initial = Xw3initial + formule_x;
                    Xw4initial = Xw4initial + formule_x;
                    Yw1initial = Yw1initial + formule_y;
                    Yw2initial = Yw2initial + formule_y;
                    Yw3initial = Yw3initial + formule_y;
                    Yw4initial = Yw4initial + formule_y;
                    deltaXw1 = abs(Xw1 - Xw1initial);
                    deltaYw1 = abs(Yw1 - Yw1initial);
                    % The graphical representation is updated
                    drawnow;
                end
                
                
            %CASE 4 : Quarter circle between -pi/2 and -pi, angles then transferred according to our situation
            else
                
                %Same formula for both normally, each time working in rectangular triangles
                %Formulas obtained for forward/reverse movement
                formule_xy = -sign(vitesse_y) * sqrt(power(pas_y, 2)/(1 + power(tan(pi/2 + theta_initial), 2)));
                formule_yy = -sign(vitesse_y) * tan(pi/2 + theta_initial) * sqrt(power(pas_y, 2)/(1 + power(tan(pi/2 + theta_initial), 2)));
                
                %Formulas obtained from a left/right lateral shift
                formule_xx = -sign(vitesse_x) * sqrt(power(pas_x, 2)/(1 + power(tan(theta_initial), 2)));
                formule_yx = -sign(vitesse_x) * tan(theta_initial) * sqrt(power(pas_x, 2)/(1 + power(tan(theta_initial), 2)));
                
                formule_x = formule_xy + formule_xx; %formula obtained with the sum of the terms according to x
                formule_y = formule_yy + formule_yx; %formula obtained with the sum of the terms according to y
                
                %W1: Engine corner 1
                Xw1 = Xw1 + formule_x * 5 * 10;%for 10 seconds
                Yw1 = Yw1 + formule_y * 5 * 10;
                %W2: Engine corner 2
                Xw2 = Xw2 + formule_x * 5 * 10;
                Yw2 = Yw2 + formule_y * 5 * 10;
                %W3: Engine corner 3
                Xw3 = Xw3 + formule_x * 5 * 10;
                Yw3 = Yw3 + formule_y * 5 * 10;
                %W4: Engine corner 4
                Xw4 = Xw4 + formule_x * 5 * 10;
                Yw4 = Yw4 + formule_y * 5 * 10;
                %G : centre of gravity of the robot
                Xg = Xg + formule_x * 5 * 10;
                Yg = Yg + formule_y * 5 * 10;
                %Calculation of deltas
                deltaXw1 = abs(Xw1 - Xw1initial);
                deltaYw1 = abs(Yw1 - Yw1initial);
                
                % Updating the coordinates of the points
                while (deltaXw1 >= precision) || (deltaYw1 >= precision)
                    set (AXE1_2, 'XData', [Xw1initial + formule_x, Xw2initial + formule_x], 'YData', [Yw1initial + formule_y, Yw2initial + formule_y]);
                    set (AXE2_3, 'XData', [Xw2initial + formule_x, Xw3initial + formule_x], 'YData', [Yw2initial + formule_y, Yw3initial + formule_y]);
                    set (AXE3_4, 'XData', [Xw3initial + formule_x, Xw4initial + formule_x], 'YData', [Yw3initial + formule_y, Yw4initial + formule_y]);
                    set (AXE4_1, 'XData', [Xw4initial + formule_x, Xw1initial + formule_x], 'YData', [Yw4initial + formule_y, Yw1initial + formule_y]);
                    Xw1initial = Xw1initial + formule_x;
                    Xw2initial = Xw2initial + formule_x;
                    Xw3initial = Xw3initial + formule_x;
                    Xw4initial = Xw4initial + formule_x;
                    Yw1initial = Yw1initial + formule_y;
                    Yw2initial = Yw2initial + formule_y;
                    Yw3initial = Yw3initial + formule_y;
                    Yw4initial = Yw4initial + formule_y;
                    deltaXw1 = abs(Xw1 - Xw1initial);
                    deltaYw1 = abs(Yw1 - Yw1initial);
                    % The graphical representation is updated
                    drawnow;
                end
            
            
            end
            
        end
            
        
        
    % In the case where speed_rotation ~= 0
    else
        theta_initial = theta_initial + vitesse_rotation;
        %W1: Engine corner 1
        Xw1 = Xg + cos(thetaW1initial + vitesse_rotation) * demi_diag;
        Yw1 = Yg + sin(thetaW1initial + vitesse_rotation) * demi_diag;
        %W2: Engine corner 2
        Xw2 = Xg + cos(thetaW2initial + vitesse_rotation) * demi_diag;
        Yw2 = Yg + sin(thetaW2initial + vitesse_rotation) * demi_diag;
        %W3: Engine corner 3
        Xw3 = Xg + cos(thetaW3initial + vitesse_rotation) * demi_diag;
        Yw3 = Yg + sin(thetaW3initial + vitesse_rotation) * demi_diag;
        %W4: Engine corner 4
        Xw4 = Xg + cos(thetaW4initial + vitesse_rotation) * demi_diag;
        Yw4 = Yg + sin(thetaW4initial + vitesse_rotation) * demi_diag;
        thetaW1 = thetaW1 + vitesse_rotation;
        delta_theta = abs(thetaW1 - thetaW1initial);
        
        
        
        % Updating the coordinates of the points
        while delta_theta >= precision_rotation
            set (AXE1_2, 'XData', [Xg + cos(thetaW1initial + pas_rotation) * demi_diag, Xg + cos(thetaW2initial + pas_rotation) * demi_diag], 'YData', [Yg + sin(thetaW1initial + pas_rotation) * demi_diag, Yg + sin(thetaW2initial + pas_rotation) * demi_diag]);
            set (AXE2_3, 'XData', [Xg + cos(thetaW2initial + pas_rotation) * demi_diag, Xg + cos(thetaW3initial + pas_rotation) * demi_diag], 'YData', [Yg + sin(thetaW2initial + pas_rotation) * demi_diag, Yg + sin(thetaW3initial + pas_rotation) * demi_diag]);
            set (AXE3_4, 'XData', [Xg + cos(thetaW3initial + pas_rotation) * demi_diag, Xg + cos(thetaW4initial + pas_rotation) * demi_diag], 'YData', [Yg + sin(thetaW3initial + pas_rotation) * demi_diag, Yg + sin(thetaW4initial + pas_rotation) * demi_diag]);
            set (AXE4_1, 'XData', [Xg + cos(thetaW4initial + pas_rotation) * demi_diag, Xg + cos(thetaW1initial + pas_rotation) * demi_diag], 'YData', [Yg + sin(thetaW4initial + pas_rotation) * demi_diag, Yg + sin(thetaW1initial + pas_rotation) * demi_diag]);
            Xw1initial = Xg + cos(thetaW1initial + pas_rotation) * demi_diag;
            Xw2initial = Xg + cos(thetaW2initial + pas_rotation) * demi_diag;
            Xw3initial = Xg + cos(thetaW3initial + pas_rotation) * demi_diag;
            Xw4initial = Xg + cos(thetaW4initial + pas_rotation) * demi_diag;
            Yw1initial = Yg + sin(thetaW1initial + pas_rotation) * demi_diag;
            Yw2initial = Yg + sin(thetaW2initial + pas_rotation) * demi_diag;
            Yw3initial = Yg + sin(thetaW3initial + pas_rotation) * demi_diag;
            Yw4initial = Yg + sin(thetaW4initial + pas_rotation) * demi_diag;
            thetaW1initial = thetaW1initial + pas_rotation;
            thetaW2initial = thetaW2initial + pas_rotation;
            thetaW3initial = thetaW3initial + pas_rotation;
            thetaW4initial = thetaW4initial + pas_rotation;
            delta_theta = abs(thetaW1 - thetaW1initial);
            % The graphical representation is updated
            drawnow;
         end
        
        
        
        
        
        
    end 
    
    
    
    
    
    
    
    
    
    
    
    
    
arretacomparer = input(prompt0);
end

%% Stopping the robot
promptstop = 'Arrêt du robot en cours';
disp(promptstop);
clear all;
close all;
clc;