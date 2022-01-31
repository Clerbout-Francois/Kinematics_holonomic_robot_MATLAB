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

%thetaW
thetaW1initial = thetaW1;
thetaW2initial = thetaW2;
thetaW3initial = thetaW3;
thetaW4initial = thetaW4;

% Initialisation du robot
AXE1_2 = line([Xw1, Xw2],[Yw1, Yw2], 'Color','g');%avant du robot
AXE2_3 = line([Xw2, Xw3],[Yw2, Yw3], 'Color','b');
AXE3_4 = line([Xw3, Xw4],[Yw3, Yw4], 'Color','r');%arrière du robot
AXE4_1 = line([Xw4, Xw1],[Yw4, Yw1], 'Color','b');


%% On rentre les infos, le modèle cinématique inverse du robot
%theta l'angle du robot par rapport à l'axe des abscisses
%theta = 0;
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
    prompt1 = 'A vous de jouer ! Indiquez la pose désirée du robot (x, y et theta)';
    disp(prompt1);
    prompt2 = ' ';
    disp(prompt2);
    %prompt3 = 'Le déplacement représenté a lieu sur 10 secondes (sauf la rotation sur lui-même) afin d être plus représentatif';
    %disp(prompt3);
    prompt4 = ' ';
    disp(prompt4);
    prompt5 = 'Coordonnée x ?(en cm)';
    prompt6 = 'Coordonnée y ? (en cm)';
    prompt7 = 'Angle theta par rapport à l horizontal ? (en rad)';
    
    % On assigne ensuite les valeurs entrées à chaque vitesse
    x_desire = input(prompt5);
    y_desire = input(prompt6);
    theta_desire = input(prompt7);
    mat_pose = [x_desire ; y_desire ; theta_desire];
    
    %Le résultat est une matrice une colonne 4 lignes contenant la vitesse de chaque moteur afin d'atteindre la position désirée
    resultat_vitesses = -(sqrt(2)/r) * mat1 * mat2 * mat_pose;
    
    V1 = resultat_vitesses(1); %vitesse de rotation du moteur1
    V2 = resultat_vitesses(2); %vitesse de rotation du moteur2
    V3 = resultat_vitesses(3); %vitesse de rotation du moteur3
    V4 = resultat_vitesses(4); %vitesse de rotation du moteur4
    
    txt = ['Vitesse moteur 1 : ' num2str(V1) ' rad/s'];
    text(5,35,txt);
    txt = ['Vitesse moteur 2 : ' num2str(V2) ' rad/s'];
    text(5,30,txt);
    txt = ['Vitesse moteur 3 : ' num2str(V3) ' rad/s'];
    text(5,25,txt);
    txt = ['Vitesse moteur 4 : ' num2str(V4) ' rad/s'];
    text(5,20,txt);
    
    resultat_position = -(sqrt(2)/2) * r * J_plus * resultat_vitesses;
    
    
    vitesse_x = resultat_position(1); %vitesse de translation selon l'axe des x
    vitesse_y = resultat_position(2); %vitesse de translation selon l'axe des y
    vitesse_rotation = -resultat_position(3); %vitesse de rotation autour de l axe des z, le "-" permet d'obtenir le bon sens de rotation en fonction du sens de rotation de chacun des 4 moteurs
    pas_x = (vitesse_x)/50; %déterminé arbitrairement
    pas_y = (vitesse_y)/50; %déterminé arbitrairement
    pas_rotation = (vitesse_rotation)/50; %déterminé arbitrairement

    if vitesse_rotation == 0
               
        % Mise à jour des coordonnées des points
        if theta_initial ==0
            %W1 : coin du moteur 1
            Xw1 = Xw1 + vitesse_x;%pour 10 secondes
            Yw1 = Yw1 + vitesse_y;
            %W2 : coin du moteur 2
            Xw2 = Xw2 + vitesse_x;
            Yw2 = Yw2 + vitesse_y;
            %W3 : coin du moteur 3
            Xw3 = Xw3 + vitesse_x;
            Yw3 = Yw3 + vitesse_y;
            %W4 : coin du moteur 4
            Xw4 = Xw4 + vitesse_x;
            Yw4 = Yw4 + vitesse_y;
            %G : centre de gravité du robot
            Xg = Xg + vitesse_x;
            Yg = Yg + vitesse_y;
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
                % On rafraichit la figure
                drawnow;
            end
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
                Xw1 = Xw1 + formule_x * 5 * 10;%pour 1 seconde et *50 car pas = 0.02*déplacement total
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
                    % On rafraichit la figure
                    drawnow;
                end
               
                
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
                Xw1 = Xw1 + formule_x * 5 * 10;%pour 1 seconde et *50 car pas = 0.02*déplacement total
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
                    % On rafraichit la figure
                    drawnow;
                end
            
                
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
                    % On rafraichit la figure
                    drawnow;
                end
                
                
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
                    % On rafraichit la figure
                    drawnow;
                end
            
            
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
            % On rafraichit la figure
            drawnow;
         end
        
        
        
        
        
        
    end 
    
    
    
    
    
    
    
    
    
    
    
    
    
arretacomparer = input(prompt0);
end

%% Arrêt du robot
promptstop = 'Arrêt du robot en cours';
%promptarret = 'Retour à la position initiale du robot';
disp(promptstop);
%disp(promptarret);