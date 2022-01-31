clear all
close all


%% Description de notre robot holonome à 4 roues
% Comprendre les mouvements réalisables : 
% Il faut se référer à la description des déplacements selon le sens de rotation des roues
% 4 roues indépendantes les unes des autres

%% Description des mouvements réalisables
% Marche avant/Marche arrière du robot
%Déplacements latéraux gauche/droite
%Diagonales
%Rotation sur lui-même

%% On initialise la figure de notre robot holonome

% On crée une comparaison pour la boucle infinie, afin de permettre
arretcomparaison = 0;


% Initialisation de la condition
condition = 0;

% Initialisation des différentes precisions
precision = 0.1;
precision_rotation=0.01;


% Définition des axes 
axis ([-40,40,-40,40]);


%Définition du robot
largeur=5;
demie_largeur = 2.5;
longueur=10;
demie_longueur=5;

%Rayon de la roue
r = 0.5;


%Il faut définir les "coins" du robot afin de pouvoir le faire se déplacer
%ensuite grâce aux différents set

%SCHEMA
% Wheel2-----Wheel1
%       |   |   
%       |   |
%       |   |
%       |   |
% Wheel3-----Wheel4

%SCHEMA ORIENTATION DES GALETS
%     //-----\\
%       |   |   
%       |   |
%       |   |
%       |   |
%     \\-----//

%W1 : coin du moteur 1
Xw1 = 2.5;
Yw1 = 5;
%W2 : coin du moteur 2
Xw2 = -2.5;
Yw2 = 5;
%W3 : coin du moteur 3
Xw3 = -2.5;
Yw3 = -5;
%W4 : coin du moteur 4
Xw4 = 2.5;
Yw4 = -5;
%G : centre de gravité du robot (utile pour la rotation)
Xg = 0;
Yg = 0;


%Partie nécessaire à la représentation des vecteurs vitesses
Xv = 0;
Yv = 0;
%Coordonnées de chaque axe de roue
%Position axe roue moteur 1
Xc1 = 3;
Yc1 = 4.5;
%Position axe roue moteur 2
Xc2 = -3;
Yc2 = 4.5;
%Position axe roue moteur 3
Xc3 = -3;
Yc3 = -4.5;
%Position axe roue moteur 4
Xc4 = 3;
Yc4 = -4.5;

%Coordonnées de chaque vecteur vitesse
%Pointe vecteur vitesse moteur 1
Xv1 = 3;
Yv1 = 4.5;
%Pointe vecteur vitesse moteur 2
Xv2 = -3;
Yv2 = 4.5;
%Pointe vecteur vitesse moteur 3
Xv3 = -3;
Yv3 = -4.5;
%Pointe vecteur vitesse moteur 4
Xv4 = 3;
Yv4 = -4.5;


%THETA est l'angle entre l'horizontale(l'axe des x, l'axe des abscisses)
%et la droite parallèle à la largeur du robot (l'axe des moteurs) passant par le centre de gravité G du robot
theta = 0;


%Initialisation des coins et de l'angle theta pour la représentation graphique
%W1 : coin du moteur 1
Xw1initial = Xw1;
Yw1initial = Yw1;
%W2 : coin du moteur 2
Xw2initial = Xw2;
Yw2initial = Yw2;
%W3 : coin du moteur 3
Xw3initial = Xw3;
Yw3initial = Yw3;
%W4 : coin du moteur 4
Xw4initial = Xw4;
Yw4initial = Yw4;
%G : centre de gravité du robot (utile pour la rotation)
Xginitial = Xg;
Yginitial = Yg;
%theta :
theta_initial = theta;



%Coordonnées initiales de chaque axe de roue
%Position axe roue moteur 1
Xc1initial = Xc1;
Yc1initial = Yc1;
%Position axe roue moteur 2
Xc2initial = Xc2;
Yc2initial = Yc2;
%Position axe roue moteur 3
Xc3initial = Xc3;
Yc3initial = Yc3;
%Position axe roue moteur 4
Xc4initial = Xc4;
Yc4initial = Yc4;


%Coordonnées initiales de chaque vecteur vitesse
%Partie nécessaire à la représentation des vecteurs vitesses
Xvinitial = Xv;
Yvinitial = Yv;
%Pointe vecteur vitesse moteur 1
Xv1initial = Xv1;
Yv1initial = Yv1;
%Pointe vecteur vitesse moteur 2
Xv2initial = Xv2;
Yv2initial = Yv2;
%Pointe vecteur vitesse moteur 3
Xv3initial = Xv3;
Yv3initial = Yv3;
%Pointe vecteur vitesse moteur 4
Xv4initial = Xv4;
Yv4initial = Yv4;



%Voir Modeling and Adaptive Control of an Omni Mecanum Wheeled Robot pour
%explication de a, b, l et alpha
b = 4.5;
a = 3;
l = sqrt(a*a + b*b);%Distance entre le centre de gravité du robot et les points W1, W2, W3 et W4
alpha = atan(b/a);
%Utiles pour les rotations
demi_diag = sqrt(demie_longueur * demie_longueur + demie_largeur * demie_largeur);
angle_coin = atan(demie_longueur/demie_largeur);

%thetaW1 est l'angle entre l'horizontale et la droite(GW1)
%De même pour thetaW2, thetaW3, thetaW4
%selon le sens horaire (et non trigo) car le sens positif des rotations est le sens horaire
thetaW1 = angle_coin;
thetaW2 = pi - angle_coin;
thetaW3 = -pi + angle_coin;
thetaW4 = -angle_coin;

%angles alpha (angle avec axe roue)
alpha1 = alpha;
alpha2 = pi - alpha;
alpha3 = - pi + alpha;
alpha4 = - alpha;

%thetaW
thetaW1initial = thetaW1;
thetaW2initial = thetaW2;
thetaW3initial = thetaW3;
thetaW4initial = thetaW4;

%angles alpha (angle avec axe roue)
alpha1initial = alpha1;
alpha2initial = alpha2;
alpha3initial = alpha3;
alpha4initial = alpha4;

% Initialisation du robot
AXE1_2 = line([Xw1, Xw2],[Yw1, Yw2], 'Color','g');
AXE2_3 = line([Xw2, Xw3],[Yw2, Yw3], 'Color','b');
AXE3_4 = line([Xw3, Xw4],[Yw3, Yw4], 'Color','r');
AXE4_1 = line([Xw4, Xw1],[Yw4, Yw1], 'Color','b');

% Initialisation des vecteurs vitesses de chaque roue
VITESSE1 = line([Xc1, Xv1],[Yc1, Yv1], 'Color','m', 'Marker', 'x');
VITESSE2 = line([Xc2, Xv2],[Yc2, Yv2], 'Color','m', 'Marker', 'x');
VITESSE3 = line([Xc3, Xv3],[Yc3, Yv3], 'Color','m', 'Marker', 'x');
VITESSE4 = line([Xc4, Xv4],[Yc4, Yv4], 'Color','m', 'Marker', 'x');
VITESSE_ROBOT = line([Xg, Xv],[Yg, Yv], 'Color','k', 'Marker', '.');


%% On rentre les infos, le modèle cinématique du robot
%theta l'angle du robot par rapport à l'axe des abscisses
mat1 = [(sqrt(2)/2) (sqrt(2)/2) l*sin(pi/4 - alpha); (sqrt(2)/2) (-sqrt(2)/2) l*sin(pi/4 - alpha); (-sqrt(2)/2) (-sqrt(2)/2) l*sin(pi/4 - alpha); (-sqrt(2)/2) (sqrt(2)/2) l*sin(pi/4 - alpha)];
mat2 = [cos(theta) sin(theta) 0; -sin(theta) cos(theta) 0; 0 0 1];
J = mat1*mat2;
T_J = transpose(J);
J1 = T_J * J;
J_1 = inv(J1);
J_plus = J1\T_J;

%Le robot possède trois degrès de libertés, c'est ce que l'on veut montrer
%dans ce programme, simuler son déplacement en fonction de la vitesse
%entrée pour chaque roue

%% Mise en place de la simulation
% Arrêt ou non du système
prompt0 = 'Arrêt du robot ? Indiquez 0 pour OUI et tout autre chiffre pour NON ';
arretacomparer = input(prompt0);

while arretacomparer ~= arretcomparaison
    % Il nous faut la vitesse de chaque roue, c'est l'utilisateur qui va les choisir
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
    
    % On assigne ensuite les valeurs entrées à chaque vitesse
    V1 = input(prompt6);
    V2 = input(prompt7);
    V3 = input(prompt8);
    V4 = input(prompt9);
    mat_vitesses = [V1 ; V2 ; V3 ; V4];
    
    
    resultat = -(sqrt(2)/2) * r * J_plus * mat_vitesses;
    
    
    vitesse_x = resultat(1); %vitesse de translation selon l'axe des x
    vitesse_y = resultat(2); %vitesse de translation selon l'axe des y
    vitesse_rotation = -resultat(3); %vitesse de rotation autour de l axe des z, le "-" permet d'obtenir le bon sens de rotation en fonction du sens de rotation de chacun des 4 moteurs
    pas_x = (vitesse_x * 10)/50; %déterminé arbitrairement
    pas_y = (vitesse_y * 10)/50; %déterminé arbitrairement
    pas_rotation = (vitesse_rotation)/50; %déterminé arbitrairement
    
    %Représentation des vecteurs vitesses
    %Discerner les cas selon le sens de rotation de chaque roue
    %Plus facile à faire avec les valeurs absolues
    
    
    if V1 > 0 %sens des aiguilles d'une montre
        set (VITESSE1, 'XData', [Xc1initial, Xc1initial + cos(theta_initial - (3 * pi)/4) * V1], 'YData', [Yc1initial, Yc1initial + sin(theta_initial - (3 * pi)/4) * V1]);
        Xv1initial = Xc1initial + cos(theta_initial - (3 * pi)/4) * V1;
        Yv1initial = Yc1initial + sin(theta_initial - (3 * pi)/4) * V1;
        theta_motor1 = theta_initial - (3 * pi)/4;
    elseif V1 < 0 %sens trigo
        set (VITESSE1, 'XData', [Xc1initial, Xc1initial + cos(theta_initial + pi/4) * abs(V1)], 'YData', [Yc1initial, Yc1initial + sin(theta_initial + pi/4) * abs(V1)]);
        Xv1initial = Xc1initial + cos(theta_initial + pi/4) * abs(V1);
        Yv1initial = Yc1initial + sin(theta_initial + pi/4) * abs(V1);
        theta_motor1 = theta_initial + pi/4;
    end
    pause(1)
    %Roue2
    if V2 > 0
        set (VITESSE2, 'XData', [Xc2initial, Xc2initial + cos(theta_initial + (3 * pi)/4) * V2], 'YData', [Yc2initial, Yc2initial + sin(theta_initial + (3 * pi)/4) * V2]);
        Xv2initial = Xc2initial + cos(theta_initial + (3 * pi)/4) * V2;
        Yv2initial = Yc2initial + sin(theta_initial + (3 * pi)/4) * V2;
        theta_motor2 = theta_initial + (3 * pi)/4;
    elseif V2 < 0
        set (VITESSE2, 'XData', [Xc2initial, Xc2initial + cos(theta_initial - pi/4) * abs(V2)], 'YData', [Yc2initial, Yc2initial + sin(theta_initial - pi/4) * abs(V2)]);
        Xv2initial = Xc2initial + cos(theta_initial - pi/4) * abs(V2);
        Yv2initial = Yc2initial + sin(theta_initial - pi/4) * abs(V2);
        theta_motor2 = theta_initial - pi/4;
    end
    pause(1)
    %Roue3
    if V3 > 0
        set (VITESSE3, 'XData', [Xc3initial, Xc3initial + cos(theta_initial + pi/4) * V3], 'YData', [Yc3initial, Yc3initial  + sin(theta_initial + pi/4) * V3]);
        Xv3initial = Xc3initial + cos(theta_initial + pi/4) * V3;
        Yv3initial = Yc3initial  + sin(theta_initial + pi/4) * V3;
        theta_motor3 = theta_initial + pi/4;
    elseif V3 < 0
        set (VITESSE3, 'XData', [Xc3initial, Xc3initial + cos(theta_initial - (3 * pi)/4) * abs(V3)], 'YData', [Yc3initial, Yc3initial + sin(theta_initial - (3 * pi)/4) * abs(V3)]);
        Xv3initial = Xc3initial + cos(theta_initial - (3 * pi)/4) * abs(V3);
        Yv3initial = Yc3initial + sin(theta_initial - (3 * pi)/4) * abs(V3);
        theta_motor3 = theta_initial - (3 * pi)/4;
    end
    pause(1)
    %Roue4
    if V4 > 0
        set (VITESSE4, 'XData', [Xc4initial, Xc4initial + cos(theta_initial - pi/4) * V4], 'YData', [Yc4initial, Yc4initial + sin(theta_initial - pi/4) * V4]);
        Xv4initial = Xc4initial + cos(theta_initial - pi/4) * V4;
        Yv4initial = Yc4initial + sin(theta_initial - pi/4) * V4;
        theta_motor4 = theta_initial - (pi)/4;
    elseif V4 < 0
        set (VITESSE4, 'XData', [Xc4initial, Xc4initial + cos(theta_initial + (3 * pi)/4) * abs(V4)], 'YData', [Yc4initial, Yc4initial + sin(theta_initial + (3 * pi)/4) * abs(V4)]);
        Xv4initial = Xc4initial + cos(theta_initial + (3 * pi)/4) * abs(V4);
        Yv4initial = Yc4initial + sin(theta_initial + (3 * pi)/4) * abs(V4);
        theta_motor4 = theta_initial + (3 * pi)/4;
    end
    pause(2)
    
    %Robot complet
    set (VITESSE_ROBOT, 'XData', [Xginitial, Xvinitial + vitesse_x * (abs(V1) + abs(V2) + abs(V3) + abs(V4)) * 0.5], 'YData', [Yginitial, Yvinitial + vitesse_y * (abs(V1) + abs(V2) + abs(V3) + abs(V4)) * 0.5]);
    Xvinitial = Xvinitial + vitesse_x * (abs(V1) + abs(V2) + abs(V3) + abs(V4)) * 0.5;
    Yvinitial = Yvinitial + vitesse_y * (abs(V1) + abs(V2) + abs(V3) + abs(V4)) * 0.5;
    pause(2)


    if vitesse_rotation == 0
               
        % Mise à jour des coordonnées des points
        if theta_initial ==0
            %W1 : coin du moteur 1
            Xw1 = Xw1 + vitesse_x * 10;%pour 10 secondes
            Yw1 = Yw1 + vitesse_y * 10;
            %W2 : coin du moteur 2
            Xw2 = Xw2 + vitesse_x * 10;
            Yw2 = Yw2 + vitesse_y * 10;
            %W3 : coin du moteur 3
            Xw3 = Xw3 + vitesse_x * 10;
            Yw3 = Yw3 + vitesse_y * 10;
            %W4 : coin du moteur 4
            Xw4 = Xw4 + vitesse_x * 10;
            Yw4 = Yw4 + vitesse_y * 10;
            %G : centre de gravité du robot
            Xg = Xg + vitesse_x * 10;
            Yg = Yg + vitesse_y * 10;
            %Calcul des deltas
            deltaXw1 = abs(Xw1 - Xw1initial);
            deltaYw1 = abs(Yw1 - Yw1initial);
            
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
                %Vecteurs
                set (VITESSE1, 'XData', [Xc1initial + pas_x, Xv1initial + pas_x], 'YData', [Yc1initial + pas_y, Yv1initial + pas_y]);
                set (VITESSE2, 'XData', [Xc2initial + pas_x, Xv2initial + pas_x], 'YData', [Yc2initial + pas_y, Yv2initial + pas_y]);
                set (VITESSE3, 'XData', [Xc3initial + pas_x, Xv3initial + pas_x], 'YData', [Yc3initial + pas_y, Yv3initial + pas_y]);
                set (VITESSE4, 'XData', [Xc4initial + pas_x, Xv4initial + pas_x], 'YData', [Yc4initial + pas_y, Yv4initial + pas_y]);
                set (VITESSE_ROBOT, 'XData', [Xginitial + pas_x, Xvinitial + pas_x], 'YData', [Yginitial + pas_y, Yvinitial + pas_y]);
                Xc1initial = Xc1initial + pas_x;
                Xc2initial = Xc2initial + pas_x;
                Xc3initial = Xc3initial + pas_x;
                Xc4initial = Xc4initial + pas_x;
                Yc1initial = Yc1initial + pas_y;
                Yc2initial = Yc2initial + pas_y;
                Yc3initial = Yc3initial + pas_y;
                Yc4initial = Yc4initial + pas_y;
                Xv1initial = Xv1initial + pas_x;
                Xv2initial = Xv2initial + pas_x;
                Xv3initial = Xv3initial + pas_x;
                Xv4initial = Xv4initial + pas_x;
                Yv1initial = Yv1initial + pas_y;
                Yv2initial = Yv2initial + pas_y;
                Yv3initial = Yv3initial + pas_y;
                Yv4initial = Yv4initial + pas_y;
                Xginitial = Xginitial + pas_x;
                Yginitial = Yginitial + pas_y;
                Xvinitial = Xvinitial + pas_x;
                Yvinitial = Yvinitial + pas_y;
                % On rafraichit la figure
                drawnow;
            end
            set (VITESSE1, 'XData', [Xc1initial, Xc1initial], 'YData', [Yc1initial, Yc1initial]);
            set (VITESSE2, 'XData', [Xc2initial, Xc2initial], 'YData', [Yc2initial, Yc2initial]);
            set (VITESSE3, 'XData', [Xc3initial, Xc3initial], 'YData', [Yc3initial, Yc3initial]);
            set (VITESSE4, 'XData', [Xc4initial, Xc4initial], 'YData', [Yc4initial, Yc4initial]);
            set (VITESSE_ROBOT, 'XData', [Xginitial, Xginitial], 'YData', [Yginitial, Yginitial]);     
            %Remise à 0 des vecteurs pour la suite
            Xv1initial = Xc1initial;
            Xv2initial = Xc2initial;
            Xv3initial = Xc3initial;
            Xv4initial = Xc4initial;
            Yv1initial = Yc1initial;
            Yv2initial = Yc2initial;
            Yv3initial = Yc3initial;
            Yv4initial = Yc4initial;
            Xvinitial = Xginitial;
            Yvinitial = Yginitial;
            
        else
            theta_initial;
            modulo_theta = mod(abs(theta_initial), 2 * pi);
            if theta_initial < 0
                modulo_theta = -modulo_theta;
            end
            
            
            %CAS 1 : Quart de cercle entre 0 et pi/2, angles ensuite reportés selon notre situation
            if (0 >= modulo_theta) && (-(pi/2) <= modulo_theta)
                
                %Même formule pour les deux normalement, à chaque fois on travaille dans des triangles rectangles
                %Formules obtenues pour une marche avant/marche arrière
                formule_xy = sign(vitesse_y) * sqrt(power(pas_y, 2)/(1 + power(tan(pi/2 + theta_initial), 2)));
                formule_yy = sign(vitesse_y) * tan(pi/2 + theta_initial) * sqrt(power(pas_y, 2)/(1 + power(tan(pi/2 + theta_initial), 2)));
                
                %Formules obtenues à partir d'un déplacement latéral gauche/droite
                formule_xx = sign(vitesse_x) * sqrt(power(pas_x, 2)/(1 + power(tan(theta_initial), 2)));
                formule_yx = sign(vitesse_x) * tan(theta_initial) * sqrt(power(pas_x, 2)/(1 + power(tan(theta_initial), 2)));
                
                formule_x = formule_xy + formule_xx; %formule obtenue avec la somme des termes selon x
                formule_y = formule_yy + formule_yx; %formule obtenue avec la somme des termes selon y
                %W1 : coin du moteur 1
                Xw1 = Xw1 + formule_x * 5 * 10;%pour 10 secondes et *5 car pas = 0.2*déplacement total
                Yw1 = Yw1 + formule_y * 5 * 10;
                %W2 : coin du moteur 2
                Xw2 = Xw2 + formule_x * 5 * 10;
                Yw2 = Yw2 + formule_y * 5 * 10;
                %W3 : coin du moteur 3
                Xw3 = Xw3 + formule_x * 5 * 10;
                Yw3 = Yw3 + formule_y * 5 * 10;
                %W4 : coin du moteur 4
                Xw4 = Xw4 + formule_x * 5 * 10;
                Yw4 = Yw4 + formule_y * 5 * 10;
                %G : centre de gravité du robot
                Xg = Xg + formule_x * 5 * 10;
                Yg = Yg + formule_y * 5 * 10;
                %Calcul des deltas
                deltaXw1 = abs(Xw1 - Xw1initial);
                deltaYw1 = abs(Yw1 - Yw1initial);

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
                    %Vecteurs
                    set (VITESSE1, 'XData', [Xc1initial + formule_x, Xv1initial + formule_x], 'YData', [Yc1initial + formule_y, Yv1initial + formule_y]);
                    set (VITESSE2, 'XData', [Xc2initial + formule_x, Xv2initial + formule_x], 'YData', [Yc2initial + formule_y, Yv2initial + formule_y]);
                    set (VITESSE3, 'XData', [Xc3initial + formule_x, Xv3initial + formule_x], 'YData', [Yc3initial + formule_y, Yv3initial + formule_y]);
                    set (VITESSE4, 'XData', [Xc4initial + formule_x, Xv4initial + formule_x], 'YData', [Yc4initial + formule_y, Yv4initial + formule_y]);
                    set (VITESSE_ROBOT, 'XData', [Xginitial + formule_x, Xvinitial + formule_x], 'YData', [Yginitial + formule_y, Yvinitial + formule_y]);
                    Xc1initial = Xc1initial + formule_x;
                    Xc2initial = Xc2initial + formule_x;
                    Xc3initial = Xc3initial + formule_x;
                    Xc4initial = Xc4initial + formule_x;
                    Yc1initial = Yc1initial + formule_y;
                    Yc2initial = Yc2initial + formule_y;
                    Yc3initial = Yc3initial + formule_y;
                    Yc4initial = Yc4initial + formule_y;
                    Xv1initial = Xv1initial + formule_x;
                    Xv2initial = Xv2initial + formule_x;
                    Xv3initial = Xv3initial + formule_x;
                    Xv4initial = Xv4initial + formule_x;
                    Yv1initial = Yv1initial + formule_y;
                    Yv2initial = Yv2initial + formule_y;
                    Yv3initial = Yv3initial + formule_y;
                    Yv4initial = Yv4initial + formule_y;
                    Xginitial = Xginitial + formule_x;
                    Yginitial = Yginitial + formule_y;
                    Xvinitial = Xvinitial + formule_x;
                    Yvinitial = Yvinitial + formule_y;
                    % On rafraichit la figure
                    drawnow;
                end
                set (VITESSE1, 'XData', [Xc1initial, Xc1initial], 'YData', [Yc1initial, Yc1initial]);
                set (VITESSE2, 'XData', [Xc2initial, Xc2initial], 'YData', [Yc2initial, Yc2initial]);
                set (VITESSE3, 'XData', [Xc3initial, Xc3initial], 'YData', [Yc3initial, Yc3initial]);
                set (VITESSE4, 'XData', [Xc4initial, Xc4initial], 'YData', [Yc4initial, Yc4initial]);
                set (VITESSE_ROBOT, 'XData', [Xginitial, Xginitial], 'YData', [Yginitial, Yginitial]);
                %Remise à 0 des vecteurs pour la suite
                Xv1initial = Xc1initial;
                Xv2initial = Xc2initial;
                Xv3initial = Xc3initial;
                Xv4initial = Xc4initial;
                Yv1initial = Yc1initial;
                Yv2initial = Yc2initial;
                Yv3initial = Yc3initial;
                Yv4initial = Yc4initial;
                Xvinitial = Xginitial;
                Yvinitial = Yginitial;
                
            %CAS 2 : Quart de cercle entre 0 et pi/2, angles ensuite reportés selon notre situation
            elseif (-(pi/2) >= modulo_theta) && (-pi <= modulo_theta)
                
                %Même formule pour les deux normalement, à chaque fois on travaille dans des triangles rectangles
                %Formules obtenues pour une marche avant/marche arrière
                formule_xy = sign(vitesse_y) * sqrt(power(pas_y, 2)/(1 + power(tan(pi/2 + theta_initial), 2)));
                formule_yy = sign(vitesse_y) * tan(pi/2 + theta_initial) * sqrt(power(pas_y, 2)/(1 + power(tan(pi/2 + theta_initial), 2)));
                
                %Formules obtenues à partir d'un déplacement latéral gauche/droite
                formule_xx = -sign(vitesse_x) * sqrt(power(pas_x, 2)/(1 + power(tan(theta_initial), 2)));
                formule_yx = -sign(vitesse_x) * tan(theta_initial) * sqrt(power(pas_x, 2)/(1 + power(tan(theta_initial), 2)));
                
                formule_x = formule_xy + formule_xx; %formule obtenue avec la somme des termes selon x
                formule_y = formule_yy + formule_yx; %formule obtenue avec la somme des termes selon y
                %W1 : coin du moteur 1
                Xw1 = Xw1 + formule_x * 5 * 10;%pour 10 secondes et *5 car pas = 0.2*déplacement total
                Yw1 = Yw1 + formule_y * 5 * 10;
                %W2 : coin du moteur 2
                Xw2 = Xw2 + formule_x * 5 * 10;
                Yw2 = Yw2 + formule_y * 5 * 10;
                %W3 : coin du moteur 3
                Xw3 = Xw3 + formule_x * 5 * 10;
                Yw3 = Yw3 + formule_y * 5 * 10;
                %W4 : coin du moteur 4
                Xw4 = Xw4 + formule_x * 5 * 10;
                Yw4 = Yw4 + formule_y * 5 * 10;
                %G : centre de gravité du robot
                Xg = Xg + formule_x * 5 * 10;
                Yg = Yg + formule_y * 5 * 10;
                %Calcul des deltas
                deltaXw1 = abs(Xw1 - Xw1initial);
                deltaYw1 = abs(Yw1 - Yw1initial);

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
                    %Vecteurs
                    set (VITESSE1, 'XData', [Xc1initial + formule_x, Xv1initial + formule_x], 'YData', [Yc1initial + formule_y, Yv1initial + formule_y]);
                    set (VITESSE2, 'XData', [Xc2initial + formule_x, Xv2initial + formule_x], 'YData', [Yc2initial + formule_y, Yv2initial + formule_y]);
                    set (VITESSE3, 'XData', [Xc3initial + formule_x, Xv3initial + formule_x], 'YData', [Yc3initial + formule_y, Yv3initial + formule_y]);
                    set (VITESSE4, 'XData', [Xc4initial + formule_x, Xv4initial + formule_x], 'YData', [Yc4initial + formule_y, Yv4initial + formule_y]);
                    set (VITESSE_ROBOT, 'XData', [Xginitial + formule_x, Xvinitial + formule_x], 'YData', [Yginitial + formule_y, Yvinitial + formule_y]);
                    Xc1initial = Xc1initial + formule_x;
                    Xc2initial = Xc2initial + formule_x;
                    Xc3initial = Xc3initial + formule_x;
                    Xc4initial = Xc4initial + formule_x;
                    Yc1initial = Yc1initial + formule_y;
                    Yc2initial = Yc2initial + formule_y;
                    Yc3initial = Yc3initial + formule_y;
                    Yc4initial = Yc4initial + formule_y;
                    Xv1initial = Xv1initial + formule_x;
                    Xv2initial = Xv2initial + formule_x;
                    Xv3initial = Xv3initial + formule_x;
                    Xv4initial = Xv4initial + formule_x;
                    Yv1initial = Yv1initial + formule_y;
                    Yv2initial = Yv2initial + formule_y;
                    Yv3initial = Yv3initial + formule_y;
                    Yv4initial = Yv4initial + formule_y;
                    Xginitial = Xginitial + formule_x;
                    Yginitial = Yginitial + formule_y;
                    Xvinitial = Xvinitial + formule_x;
                    Yvinitial = Yvinitial + formule_y;
                    % On rafraichit la figure
                    drawnow;
                end
                set (VITESSE1, 'XData', [Xc1initial, Xc1initial], 'YData', [Yc1initial, Yc1initial]);
                set (VITESSE2, 'XData', [Xc2initial, Xc2initial], 'YData', [Yc2initial, Yc2initial]);
                set (VITESSE3, 'XData', [Xc3initial, Xc3initial], 'YData', [Yc3initial, Yc3initial]);
                set (VITESSE4, 'XData', [Xc4initial, Xc4initial], 'YData', [Yc4initial, Yc4initial]);
                set (VITESSE_ROBOT, 'XData', [Xginitial, Xginitial], 'YData', [Yginitial, Yginitial]);
                %Remise à 0 des vecteurs pour la suite
                Xv1initial = Xc1initial;
                Xv2initial = Xc2initial;
                Xv3initial = Xc3initial;
                Xv4initial = Xc4initial;
                Yv1initial = Yc1initial;
                Yv2initial = Yc2initial;
                Yv3initial = Yc3initial;
                Yv4initial = Yc4initial;
                Xvinitial = Xginitial;
                Yvinitial = Yginitial;
                
            %CAS 3 : Quart de cercle entre pi/2 et pi, angles ensuite reportés selon notre situation
            elseif ((pi/2) >= modulo_theta) && (0 <= modulo_theta)

                %Même formule pour les deux normalement, à chaque fois on travaille dans des triangles rectangles
                %Formules obtenues pour une marche avant/marche arrière
                formule_xy = -sign(vitesse_y) * sqrt(power(pas_y, 2)/(1 + power(tan(pi/2 + theta_initial), 2)));
                formule_yy = -sign(vitesse_y) * tan(pi/2 + theta_initial) * sqrt(power(pas_y, 2)/(1 + power(tan(pi/2 + theta_initial), 2)));
                
                %Formules obtenues à partir d'un déplacement latéral gauche/droite
                formule_xx = sign(vitesse_x) * sqrt(power(pas_x, 2)/(1 + power(tan(theta_initial), 2)));
                formule_yx = sign(vitesse_x) * tan(theta_initial) * sqrt(power(pas_x, 2)/(1 + power(tan(theta_initial), 2)));
                
                formule_x = formule_xy + formule_xx; %formule obtenue avec la somme des termes selon x
                formule_y = formule_yy + formule_yx; %formule obtenue avec la somme des termes selon y
                
                %W1 : coin du moteur 1
                Xw1 = Xw1 + formule_x * 5 * 10;%pour 10 secondes
                Yw1 = Yw1 + formule_y * 5 * 10;
                %W2 : coin du moteur 2
                Xw2 = Xw2 + formule_x * 5 * 10;
                Yw2 = Yw2 + formule_y * 5 * 10;
                %W3 : coin du moteur 3
                Xw3 = Xw3 + formule_x * 5 * 10;
                Yw3 = Yw3 + formule_y * 5 * 10;
                %W4 : coin du moteur 4
                Xw4 = Xw4 + formule_x * 5 * 10;
                Yw4 = Yw4 + formule_y * 5 * 10;
                %G : centre de gravité du robot
                Xg = Xg + formule_x * 5 * 10;
                Yg = Yg + formule_y * 5 * 10;
                %Calcul des deltas
                deltaXw1 = abs(Xw1 - Xw1initial);
                deltaYw1 = abs(Yw1 - Yw1initial);

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
                    %Vecteurs
                    set (VITESSE1, 'XData', [Xc1initial + formule_x, Xv1initial + formule_x], 'YData', [Yc1initial + formule_y, Yv1initial + formule_y]);
                    set (VITESSE2, 'XData', [Xc2initial + formule_x, Xv2initial + formule_x], 'YData', [Yc2initial + formule_y, Yv2initial + formule_y]);
                    set (VITESSE3, 'XData', [Xc3initial + formule_x, Xv3initial + formule_x], 'YData', [Yc3initial + formule_y, Yv3initial + formule_y]);
                    set (VITESSE4, 'XData', [Xc4initial + formule_x, Xv4initial + formule_x], 'YData', [Yc4initial + formule_y, Yv4initial + formule_y]);
                    set (VITESSE_ROBOT, 'XData', [Xginitial + formule_x, Xvinitial + formule_x], 'YData', [Yginitial + formule_y, Yvinitial + formule_y]);
                    Xc1initial = Xc1initial + formule_x;
                    Xc2initial = Xc2initial + formule_x;
                    Xc3initial = Xc3initial + formule_x;
                    Xc4initial = Xc4initial + formule_x;
                    Yc1initial = Yc1initial + formule_y;
                    Yc2initial = Yc2initial + formule_y;
                    Yc3initial = Yc3initial + formule_y;
                    Yc4initial = Yc4initial + formule_y;
                    Xv1initial = Xv1initial + formule_x;
                    Xv2initial = Xv2initial + formule_x;
                    Xv3initial = Xv3initial + formule_x;
                    Xv4initial = Xv4initial + formule_x;
                    Yv1initial = Yv1initial + formule_y;
                    Yv2initial = Yv2initial + formule_y;
                    Yv3initial = Yv3initial + formule_y;
                    Yv4initial = Yv4initial + formule_y;
                    Xginitial = Xginitial + formule_x;
                    Yginitial = Yginitial + formule_y;
                    Xvinitial = Xvinitial + formule_x;
                    Yvinitial = Yvinitial + formule_y;
                    % On rafraichit la figure
                    drawnow;
                end
                set (VITESSE1, 'XData', [Xc1initial, Xc1initial], 'YData', [Yc1initial, Yc1initial]);
                set (VITESSE2, 'XData', [Xc2initial, Xc2initial], 'YData', [Yc2initial, Yc2initial]);
                set (VITESSE3, 'XData', [Xc3initial, Xc3initial], 'YData', [Yc3initial, Yc3initial]);
                set (VITESSE4, 'XData', [Xc4initial, Xc4initial], 'YData', [Yc4initial, Yc4initial]);
                set (VITESSE_ROBOT, 'XData', [Xginitial, Xginitial], 'YData', [Yginitial, Yginitial]);
                %Remise à 0 des vecteurs pour la suite
                Xv1initial = Xc1initial;
                Xv2initial = Xc2initial;
                Xv3initial = Xc3initial;
                Xv4initial = Xc4initial;
                Yv1initial = Yc1initial;
                Yv2initial = Yc2initial;
                Yv3initial = Yc3initial;
                Yv4initial = Yc4initial;
                Xvinitial = Xginitial;
                Yvinitial = Yginitial;
                
            %CAS 4 : Quart de cercle entre -pi/2 et -pi, angles ensuite reportés selon notre situation
            else

                %Même formule pour les deux normalement, à chaque fois on travaille dans des triangles rectangles
                %Formules obtenues pour une marche avant/marche arrière
                formule_xy = -sign(vitesse_y) * sqrt(power(pas_y, 2)/(1 + power(tan(pi/2 + theta_initial), 2)));
                formule_yy = -sign(vitesse_y) * tan(pi/2 + theta_initial) * sqrt(power(pas_y, 2)/(1 + power(tan(pi/2 + theta_initial), 2)));
                
                %Formules obtenues à partir d'un déplacement latéral gauche/droite
                formule_xx = -sign(vitesse_x) * sqrt(power(pas_x, 2)/(1 + power(tan(theta_initial), 2)));
                formule_yx = -sign(vitesse_x) * tan(theta_initial) * sqrt(power(pas_x, 2)/(1 + power(tan(theta_initial), 2)));
                
                formule_x = formule_xy + formule_xx; %formule obtenue avec la somme des termes selon x
                formule_y = formule_yy + formule_yx; %formule obtenue avec la somme des termes selon y
                
                %W1 : coin du moteur 1
                Xw1 = Xw1 + formule_x * 5 * 10;%pour 10 secondes
                Yw1 = Yw1 + formule_y * 5 * 10;
                %W2 : coin du moteur 2
                Xw2 = Xw2 + formule_x * 5 * 10;
                Yw2 = Yw2 + formule_y * 5 * 10;
                %W3 : coin du moteur 3
                Xw3 = Xw3 + formule_x * 5 * 10;
                Yw3 = Yw3 + formule_y * 5 * 10;
                %W4 : coin du moteur 4
                Xw4 = Xw4 + formule_x * 5 * 10;
                Yw4 = Yw4 + formule_y * 5 * 10;
                %G : centre de gravité du robot
                Xg = Xg + formule_x * 5 * 10;
                Yg = Yg + formule_y * 5 * 10;
                %Calcul des deltas
                deltaXw1 = abs(Xw1 - Xw1initial);
                deltaYw1 = abs(Yw1 - Yw1initial);

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
                    %Vecteurs
                    set (VITESSE1, 'XData', [Xc1initial + formule_x, Xv1initial + formule_x], 'YData', [Yc1initial + formule_y, Yv1initial + formule_y]);
                    set (VITESSE2, 'XData', [Xc2initial + formule_x, Xv2initial + formule_x], 'YData', [Yc2initial + formule_y, Yv2initial + formule_y]);
                    set (VITESSE3, 'XData', [Xc3initial + formule_x, Xv3initial + formule_x], 'YData', [Yc3initial + formule_y, Yv3initial + formule_y]);
                    set (VITESSE4, 'XData', [Xc4initial + formule_x, Xv4initial + formule_x], 'YData', [Yc4initial + formule_y, Yv4initial + formule_y]);
                    set (VITESSE_ROBOT, 'XData', [Xginitial + formule_x, Xvinitial + formule_x], 'YData', [Yginitial + formule_y, Yvinitial + formule_y]);
                    Xc1initial = Xc1initial + formule_x;
                    Xc2initial = Xc2initial + formule_x;
                    Xc3initial = Xc3initial + formule_x;
                    Xc4initial = Xc4initial + formule_x;
                    Yc1initial = Yc1initial + formule_y;
                    Yc2initial = Yc2initial + formule_y;
                    Yc3initial = Yc3initial + formule_y;
                    Yc4initial = Yc4initial + formule_y;
                    Xv1initial = Xv1initial + formule_x;
                    Xv2initial = Xv2initial + formule_x;
                    Xv3initial = Xv3initial + formule_x;
                    Xv4initial = Xv4initial + formule_x;
                    Yv1initial = Yv1initial + formule_y;
                    Yv2initial = Yv2initial + formule_y;
                    Yv3initial = Yv3initial + formule_y;
                    Yv4initial = Yv4initial + formule_y;
                    Xginitial = Xginitial + formule_x;
                    Yginitial = Yginitial + formule_y;
                    Xvinitial = Xvinitial + formule_x;
                    Yvinitial = Yvinitial + formule_y;
                    % On rafraichit la figure
                    drawnow;
                end
                set (VITESSE1, 'XData', [Xc1initial, Xc1initial], 'YData', [Yc1initial, Yc1initial]);
                set (VITESSE2, 'XData', [Xc2initial, Xc2initial], 'YData', [Yc2initial, Yc2initial]);
                set (VITESSE3, 'XData', [Xc3initial, Xc3initial], 'YData', [Yc3initial, Yc3initial]);
                set (VITESSE4, 'XData', [Xc4initial, Xc4initial], 'YData', [Yc4initial, Yc4initial]);
                set (VITESSE_ROBOT, 'XData', [Xginitial, Xginitial], 'YData', [Yginitial, Yginitial]);
                %Remise à 0 des vecteurs pour la suite
                Xv1initial = Xc1initial;
                Xv2initial = Xc2initial;
                Xv3initial = Xc3initial;
                Xv4initial = Xc4initial;
                Yv1initial = Yc1initial;
                Yv2initial = Yc2initial;
                Yv3initial = Yc3initial;
                Yv4initial = Yc4initial;
                Xvinitial = Xginitial;
                Yvinitial = Yginitial;
                
            end
            
        end
            
        
        
    % Dans le cas où vitesse_rotation ~= 0
    else
        theta_initial = theta_initial + vitesse_rotation;
        %W1 : coin du moteur 1
        Xw1 = Xg + cos(thetaW1initial + vitesse_rotation) * demi_diag;
        Yw1 = Yg + sin(thetaW1initial + vitesse_rotation) * demi_diag;
        %W2 : coin du moteur 2
        Xw2 = Xg + cos(thetaW2initial + vitesse_rotation) * demi_diag;
        Yw2 = Yg + sin(thetaW2initial + vitesse_rotation) * demi_diag;
        %W3 : coin du moteur 3;
        Xw3 = Xg + cos(thetaW3initial + vitesse_rotation) * demi_diag;
        Yw3 = Yg + sin(thetaW3initial + vitesse_rotation) * demi_diag;
        %W4 : coin du moteur 4
        Xw4 = Xg + cos(thetaW4initial + vitesse_rotation) * demi_diag;
        Yw4 = Yg + sin(thetaW4initial + vitesse_rotation) * demi_diag;
        thetaW1 = thetaW1 + vitesse_rotation;
        delta_theta = abs(thetaW1 - thetaW1initial);
        
        
        
        % Mise à jour des coordonnées des points
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

            Xc1initial = Xg + cos(alpha1initial + pas_rotation) * l;
            Xc2initial = Xg + cos(alpha2initial + pas_rotation) * l;
            Xc3initial = Xg + cos(alpha3initial + pas_rotation) * l;
            Xc4initial = Xg + cos(alpha4initial + pas_rotation) * l;
            Yc1initial = Yg + sin(alpha1initial + pas_rotation) * l;
            Yc2initial = Yg + sin(alpha2initial + pas_rotation) * l;
            Yc3initial = Yg + sin(alpha3initial + pas_rotation) * l;
            Yc4initial = Yg + sin(alpha4initial + pas_rotation) * l;

            
            
            if V1 > 0 %sens des aiguilles d'une montre
                set (VITESSE1, 'XData', [Xg + cos(alpha1initial + pas_rotation) * l, Xg + cos(alpha1initial + pas_rotation) * l + cos(theta_motor1 + pas_rotation) * V1], 'YData', [Yg + sin(alpha1initial + pas_rotation) * l, Yg + sin(alpha1initial + pas_rotation) * l + sin(theta_motor1 + pas_rotation) * V1]);
                Xc1initial = Xg + cos(alpha1initial + pas_rotation) * l;
                Yc1initial = Yg + sin(alpha1initial + pas_rotation) * l;
                theta_motor1 = theta_motor1 + pas_rotation;
            elseif V1 < 0 %sens trigo
                set (VITESSE1, 'XData', [Xg + cos(alpha1initial + pas_rotation) * l, Xg + cos(alpha1initial + pas_rotation) * l + cos(theta_motor1 + pas_rotation) * abs(V1)], 'YData', [Yg + sin(alpha1initial + pas_rotation) * l, Yg + sin(alpha1initial + pas_rotation) * l + sin(theta_motor1 + pas_rotation) * abs(V1)]);
                Xc1initial = Xg + cos(alpha1initial + pas_rotation) * l;
                Yc1initial = Yg + sin(alpha1initial + pas_rotation) * l;
                theta_motor1 = theta_motor1 + pas_rotation;
            end
            %Roue2
            if V2 > 0
                set (VITESSE2, 'XData', [Xg + cos(alpha2initial + pas_rotation) * l, Xg + cos(alpha2initial + pas_rotation) * l + cos(theta_motor2 + pas_rotation) * V2], 'YData', [Yg + sin(alpha2initial + pas_rotation) * l, Yg + sin(alpha2initial + pas_rotation) * l + sin(theta_motor2 + pas_rotation) * V2]);
                Xc2initial = Xg + cos(alpha2initial + pas_rotation) * l;
                Yc2initial = Yg + sin(alpha2initial + pas_rotation) * l;
                theta_motor2 = theta_motor2 + pas_rotation;
            elseif V2 < 0
                set (VITESSE2, 'XData', [Xg + cos(alpha2initial + pas_rotation) * l, Xg + cos(alpha2initial + pas_rotation) * l + cos(theta_motor2 + pas_rotation) * abs(V2)], 'YData', [Yg + sin(alpha2initial + pas_rotation) * l, Yg + sin(alpha2initial + pas_rotation) * l + sin(theta_motor2 + pas_rotation) * abs(V2)]);
                Xc2initial = Xg + cos(alpha2initial + pas_rotation) * l;
                Yc2initial = Yg + sin(alpha2initial + pas_rotation) * l;
                theta_motor2 = theta_motor2 + pas_rotation;
            end
            %Roue3
            if V3 > 0
                set (VITESSE3, 'XData', [Xg + cos(alpha3initial + pas_rotation) * l, Xg + cos(alpha3initial + pas_rotation) * l + cos(theta_motor3 + pas_rotation) * V3], 'YData', [Yg + sin(alpha3initial + pas_rotation) * l, Yg + sin(alpha3initial + pas_rotation) * l  + sin(theta_motor3 + pas_rotation) * V3]);
                Xc3initial = Xg + cos(alpha3initial + pas_rotation) * l;
                Yc3initial = Yg + sin(alpha3initial + pas_rotation) * l;
                theta_motor3 = theta_motor3 + pas_rotation;
            elseif V3 < 0
                set (VITESSE3, 'XData', [Xg + cos(alpha3initial + pas_rotation) * l, Xg + cos(alpha3initial + pas_rotation) * l + cos(theta_motor3 + pas_rotation) * abs(V3)], 'YData', [Yg + sin(alpha3initial + pas_rotation) * l, Yg + sin(alpha3initial + pas_rotation) * l + sin(theta_motor3 + pas_rotation) * abs(V3)]);
                Xc3initial = Xg + cos(alpha3initial + pas_rotation) * l;
                Yc3initial = Yg + sin(alpha3initial + pas_rotation) * l;
                theta_motor3 = theta_motor3 + pas_rotation;
            end
            %Roue4
            if V4 > 0
                set (VITESSE4, 'XData', [Xg + cos(alpha4initial + pas_rotation) * l, Xg + cos(alpha4initial + pas_rotation) * l + cos(theta_motor4 + pas_rotation) * V4], 'YData', [Yg + sin(alpha4initial + pas_rotation) * l, Yg + sin(alpha4initial + pas_rotation) * l + sin(theta_motor4 + pas_rotation) * V4]);
                Xc4initial = Xg + cos(alpha4initial + pas_rotation) * l;
                Yc4initial = Yg + sin(alpha4initial + pas_rotation) * l;
                theta_motor4 = theta_motor4 + pas_rotation;
            elseif V4 < 0
                set (VITESSE4, 'XData', [Xg + cos(alpha4initial + pas_rotation) * l, Xg + cos(alpha4initial + pas_rotation) * l + cos(theta_motor4 + pas_rotation) * abs(V4)], 'YData', [Yg + sin(alpha4initial + pas_rotation) * l, Yg + sin(alpha4initial + pas_rotation) * l + sin(theta_motor4 + pas_rotation) * abs(V4)]);
                Xc4initial = Xg + cos(alpha4initial + pas_rotation) * l;
                Yc4initial = Yg + sin(alpha4initial + pas_rotation) * l;
                theta_motor4 = theta_motor4 + pas_rotation;
            end
            theta_initial = theta_initial + pas_rotation;
            alpha1initial = alpha1initial + pas_rotation;
            alpha2initial = alpha2initial + pas_rotation;
            alpha3initial = alpha3initial + pas_rotation;
            alpha4initial = alpha4initial + pas_rotation;
            % On rafraichit la figure
            drawnow;
        end
        set (VITESSE1, 'XData', [Xc1initial, Xc1initial], 'YData', [Yc1initial, Yc1initial]);
        set (VITESSE2, 'XData', [Xc2initial, Xc2initial], 'YData', [Yc2initial, Yc2initial]);
        set (VITESSE3, 'XData', [Xc3initial, Xc3initial], 'YData', [Yc3initial, Yc3initial]);
        set (VITESSE4, 'XData', [Xc4initial, Xc4initial], 'YData', [Yc4initial, Yc4initial]);
        set (VITESSE_ROBOT, 'XData', [Xginitial, Xginitial], 'YData', [Yginitial, Yginitial]);
        %Remise à 0 des vecteurs pour la suite
        Xv1initial = Xc1initial;
        Xv2initial = Xc2initial;
        Xv3initial = Xc3initial;
        Xv4initial = Xc4initial;
        Yv1initial = Yc1initial;
        Yv2initial = Yc2initial;
        Yv3initial = Yc3initial;
        Yv4initial = Yc4initial;
        Xvinitial = Xginitial;
        Yvinitial = Yginitial;
        
        
        
        
    end 
    
    
    
    
    
    
    
    
    
    
    
    
    
arretacomparer = input(prompt0);
end

%% Arrêt du robot
promptstop = 'Arrêt du robot en cours';
%promptarret = 'Retour à la position initiale du robot';
disp(promptstop);
%disp(promptarret);