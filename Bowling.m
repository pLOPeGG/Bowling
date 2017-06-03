%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         Pour modifier les conditions initiales        %
%                 aller à la ligne 79                   %
%     Pour ralentir l'affichage modifier le nombre      %
%   d'images affichées par secondes ligne 55            %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all;
global dt y_quille R_b R_bas_q R_haut_q l_0 H_q  L l distInterQuille Nb_q

%% Données Générales
g = 9.78033;             %accélération de pesanteur (m/s^2)

%%%Piste

L=18;                    %longueur piste (m)
l=2;                     %largeur piste (m)
distInterQuille=0.3;     %distance entre 2 quilles côtes à côtes (m)
f_boule = 0.19;                %coeff frottement piste pour la boule 
f_quille = 0.5;              %coeff frottement piste pour les quilles

%%%Collision
k_Boule_Quille=10000;         %raideur du ressort de contact boule - quille
k_Quille_Quille=10000;        %raideur du ressort de contact entre quilles
k_intra_Quille=50000;         %raideur du ressort de cohésion d'une quille
k_sol=100000;


%%%Boule

M_b=8;                      %masse boule (kg)
R_b=0.15;                   %rayon boule (m)
J_b = 0.4*M_b*R_b^2;        %moment d'inertie de la boule 

%%%Quille

Nb_q = 10;                  %nombre de quilles sur le terrain
M_q=1.4;                    %masse de la quille (kg)
M_bas_q = 2*M_q/3;          %masse de la boule inférieure (kg)
M_haut_q = M_q/3;           %masse de la boule supérieure (kg)
R_q=0.055;                  %rayon de la quille (m) dans le cadre d'une approximation par un cylindre
R_bas_q=R_q;                %rayon de la boule inférieure (m)
R_haut_q=3*R_q/4;           %rayon de la boule supérieure (m)
H_q=0.7;                    %hauteur d la quille (m)
l_0=M_haut_q*g/k_intra_Quille + H_q-R_bas_q-R_haut_q;    %longueur à vide du ressort inter-boule(m) calculée pour qu'avec la boule dessus, la hauteur soit de H_q
J_q=M_q*(3*R_q^2+H_q^2)/12; %moment d'inertie de la quille au centre (approximée par un cylindre)
yG = 0.146 ;                %position centre gravité (d'après caractéristiques techniques )


%   Points pour la forme de la quille 
x_pos_quille = [0 1 4 6 9 12 14 16 18 20 22 24 26 29 32 34 36 38 39 40];
y_pos_quille = 0.0005*[43 64 97.5 107.1 110 106.9 102.4 96.2 88.7 79.9 70.6 62.3 55.9 51.4 61.1 76.5 84.6 76.1 61.9 0];
x_quille = 0:0.5:40;
y_quille = spline(x_pos_quille,y_pos_quille,x_quille);






%% Variables d'intégration
%%%Temporel
N=1e4;                          %Nb intégrations
t_init=0;                       %temps de début de simulation (s)
t_tot=3;                        %temps de fin de simulation (s)
dt=(t_tot-t_init)/N;            %pas de temps (s)
t=linspace(t_init,t_tot,N);     %vecteur des instants calculés

%%%Affichage
imagesPerSec = 120;                              %nombre d'images par seconde
Nb_images = imagesPerSec * (t_tot-t_init);      %nombre d'images au total
pas_Affichage = floor(N/Nb_images)+1;             %pas d'iteration lors de l'affichage de la simulation



%%%Schéma Newmark
gamma=0.5;      %paramètre du schéma de newmark pour un schéma de Verlet
beta=0;         %paramètre du schéma de newmark pour un schéma de Verlet

%%%Cinématique Boule
A_b=zeros(3,N);     %accélération X,Y,Z
V_b=zeros(3,N);     %vitesse X,Y,Z
X_b=zeros(3,N);     %position X,Y,Z


%%%Init Boule
V_0=15;                 %vitesse initale (m/s)
alpha=0.05;             %angle d'attaque initial (rad)
Y_0=-0.7;               %positionnement initial sur la piste [-1;1] (m)

%%% Conditions initiales
V_b(:,1)=[V_0*cos(alpha),V_0*sin(alpha),0];
A_b(:,1)=-f_boule*V_b(1:3,1);
X_b(:,1)=[0,Y_0,R_b];

%%%Cinématique Quilles
A_q=zeros(3,N,2*Nb_q);        %accélération X, Y, Z pour toutes les quilles boule du haut et du bas
V_q=zeros(3,N,2*Nb_q);        %vitesse X, Y, Z pour toutes les quilles boule du haut et du bas
X_q=zeros(3,N,2*Nb_q);        %position X, Y, Z pour toutes les quilles boule du haut et du bas

Dir_q = zeros(3,N,Nb_q);        %direction des quilles (du bas vers le haut)

%%%Init Quille
X_q(:,1,:)=initQuilles(Nb_q);       %positionnement des quilles

for i=1:Nb_q
    Dir_q(:,1,i) = [0,0,1];         %direction initiale
end

peneQ_B = zeros(1,2*Nb_q);                %vecteur de penetration de la boule dans les quilles
dirContactQ_B = zeros(3,2*Nb_q);          %Matrice des directions du contact entre la boule et les quilles 

MatrixElasticForceQuilles = zeros(2*Nb_q, 2*Nb_q, 3);       %matrice des efforts exercés d'une quille sur une autre : la valeur (i,j) indique la force exercée par i sur j
                                                            %c'est une matrice "antisymétrique" (termes symétriques opposés), uniquement des 0 sur la diagonale
                                                            %les N_q premières sont les boules du bas, les N_q secondes en haut
                                                        
index_limite = -log(1 - L/M_b*f_boule/V_0/cos(alpha))*M_b/f_boule/dt;   %index du temps auquel il faut commencer à effectuer des calculs de collision
%% Intégration
for i=1:N-1
    elasticForceQuille=zeros(2*Nb_q,3);   %remise à 0 des forces de collision inter quilles
    elasticForceIntraQuille=zeros(2*Nb_q,3);    %remise à 0 des forces de collision intra quilles
    elasticForceBoule=zeros(1,3);
    MatrixElasticForceQuilles=zeros(2*Nb_q,2*Nb_q,3);
    Force_Boule=0;
    Force_Quille=zeros(2*Nb_q,3);
    %%% Prévisionnel
    [Prev_X_b, Prev_X_q] = PrevisionPositionEulerExplicit(X_b(:,i), V_b(:,i), A_b(:,i),reshape(X_q(:,i,:),3,2*Nb_q), reshape(V_q(:,i,:),3,2*Nb_q), reshape(A_q(:,i,:),3,2*Nb_q)); %calcul des positions prévisionnelles au temps i+1 par un schéma explicite
    
    %%%COLLISION
    if(i > index_limite)
        %%% Collision Boule-Quille
        for j=1:2*Nb_q
            r_q = (j<Nb_q+1)*R_bas_q + ~(j<Nb_q+1)*R_haut_q;    %rayon de la boule de quille (bas si j<Nb_q+1, haut sinon
            [peneQ_B(j), dirContactQ_B(:,j)] = collisionBoule_Boule(Prev_X_b, R_b,Prev_X_q(:,j), r_q);    

        end 
        elasticForceBoule = peneQ_B*dirContactQ_B'; %vecteur des forces des quilles vers la boule au temps i en haut et en bas


           %%% Collision Quille-Quille
        for j=1:2*Nb_q-1          %j désigne l'indice d'une quille
            for p=j+1:2*Nb_q      %p désigne l'indice d'une autre quille tq p>j
                %si j ou p > Nb_q, cela concerne une quille du haut -> r = R_haut_q
                if(j<Nb_q+1)    %boule du bas
                    r_j = R_bas_q;      
                else            %boule du haut
                    r_j = R_haut_q;
                end
                if(p<Nb_q+1)    %idem pour p
                    r_p = R_bas_q;
                else
                    r_p = R_haut_q;
                end
                [pene, dir]=collisionBoule_Boule(Prev_X_q(:,j), r_j, Prev_X_q(:,p), r_p);       %contact de j vers p
                MatrixElasticForceQuilles(j,p,:) = -pene*dir';       %sur une même ligne j de la matrice : tous les efforts exercés par cette quille sur les autres
                MatrixElasticForceQuilles(p,j,:) = pene*dir';      %sur une même colonne p de la matrice : tous les efforts subis par cette quille
            end
        end
        for j=1:2*Nb_q
            for p=1:2*Nb_q
                 elasticForceQuille(j,:) = elasticForceQuille(j,:) + reshape(MatrixElasticForceQuilles(p,j,:),1,3);     %somme sur chaque colonne de MatrixElasticForceQuilles
            end
        end
    end
        

    %
    
    
    for j=1:Nb_q        %ressort intra quille
        elasticForceIntraQuille(j,:)= k_intra_Quille*(norm(Prev_X_q(:,j+Nb_q)-Prev_X_q(:,j))-l_0)*(Prev_X_q(:,j+Nb_q)-Prev_X_q(:,j))/norm(Prev_X_q(:,j+Nb_q)-Prev_X_q(:,j));
        elasticForceIntraQuille(j+Nb_q,:)=-elasticForceIntraQuille(j,:);
    end
    
    
    if(Prev_X_b(3)>R_b)
       Force_Boule(3)=-M_b*g;
    elseif(Prev_X_b(3)<R_b)
        Force_Boule(3)=k_sol*(R_b-Prev_X_b(3));
    end
    %%%TODO : ajouter la réaction du mur !
    
    
    for j=1:2*Nb_q
        r_j = (j<Nb_q+1)*R_bas_q + ~(j<Nb_q+1)*R_haut_q;
        if(Prev_X_q(3,j)>r_j)
            Force_Quille(j,3)=-M_b*g;
        elseif(Prev_X_q(3,j)<r_j)
            Force_Quille(j,3)=k_sol*(r_j-Prev_X_q(3,j));
        end
    end

    
    %%% Schémas de Verlet
    
    V_b(1:3,i+1)=(V_b(1:3,i)+dt*((1-gamma)*A_b(1:3,i)+gamma*(k_Boule_Quille*elasticForceBoule'+Force_Boule')/M_b))/(1+dt*f_boule/M_b*gamma);          
    A_b(1:3,i+1)=(-f_boule*V_b(:,i+1)+k_Boule_Quille*elasticForceBoule'+Force_Boule')/M_b;
    X_b(1:3,i+1)=(X_b(1:3,i)+dt*V_b(1:3,i)+dt^2/2*((1-2*beta)*A_b(1:3,i))+2*beta*A_b(1:3,i+1));
    if(X_b(3,i+1)<R_b)
        X_b(3,i+1) = R_b;
    end
    
    
    for j=1:2*Nb_q
        m_j = (j<Nb_q+1)*M_bas_q + ~(j<Nb_q+1)*M_haut_q;
        r_j = (j<Nb_q+1)*R_bas_q + ~(j<Nb_q+1)*R_haut_q;
        if(Prev_X_q(3,j)>r_j)
            V_q(:,i+1,j)=(V_q(:,i,j)+dt*((1-gamma)*A_q(:,i,j)+gamma*(-k_Boule_Quille*peneQ_B(j)*dirContactQ_B(:,j) + k_Quille_Quille*elasticForceQuille(j,:)' + Force_Quille(j,:)' + elasticForceIntraQuille(j,:)')/m_j));
            A_q(:,i+1,j)=(-k_Boule_Quille*peneQ_B(j)*dirContactQ_B(:,j)+k_Quille_Quille*elasticForceQuille(j,:)'+ Force_Quille(j,:)' + elasticForceIntraQuille(j,:)')/m_j;
        else
            V_q(:,i+1,j)=(V_q(:,i,j)+dt*((1-gamma)*A_q(:,i,j)+gamma*(-k_Boule_Quille*peneQ_B(j)*dirContactQ_B(:,j) + k_Quille_Quille*elasticForceQuille(j,:)' + Force_Quille(j,:)' + elasticForceIntraQuille(j,:)')/m_j))/(1+dt*f_quille*gamma/m_j);
            A_q(:,i+1,j)=(-f_quille*V_q(:,i+1,j)-k_Boule_Quille*peneQ_B(j)*dirContactQ_B(:,j)+k_Quille_Quille*elasticForceQuille(j,:)'+ Force_Quille(j,:)' + elasticForceIntraQuille(j,:)')/m_j;
        end
        X_q(:,i+1,j)=(X_q(:,i,j)+dt*V_q(:,i,j)+dt^2/2*((1-2*beta)*A_q(:,i,j))+2*beta*A_q(:,i+1,j));
        if(X_q(3,i+1,j) < r_j)
            X_q(3,i+1,j) = r_j;
        end
                  
    end
    for j=1:Nb_q        %nouvelle direction de la quille
        Dir_q(:,i+1,j) = (X_q(:,i+1,j+Nb_q)-X_q(:,i+1,j))/(norm(X_q(:,i+1,j+Nb_q)-X_q(:,i+1,j)));
    end
end

%% Affichage
%%% Energie cinétique   %% Remarque Ec = Energie mécanique 
Ec=0.5*M_b*sum(V_b.^2);
for i=1:Nb_q
    Ec = Ec + 0.5*M_q*sum(V_q(:,:,i).^2);
end
figure(1)
plot(t,Ec,'-- k');
title('Evolution de l''énergie mécanique du système {boule + quilles} au cours du temps');
xlabel('Temps(s)'); 
ylabel('Ec(J)');



figure('units','normalized','outerposition',[0 0 1 1])

for i=1:pas_Affichage:N
    DrawBall(X_b(:,i), R_b);
    hold on
    for j=1:Nb_q
        %DrawCylinder(X_q(:,i,j),Dir_q(:,i,j));
        DrawCylinderDebug(X_q(:,i,j), X_q(:,i,j+Nb_q));
    end
    
    xlim([0 25]);
    ylim([-12 12]);
    zlim([0 25]);

   
    set(gca, 'CameraPosition', [10, 4, 5]);
    set(gca, 'CameraTarget', [18.1,0,0]);
    camzoom(15);
%%% Pour la vue en 3D
%     if(X_b(1,i)<17)
%         set(gca, 'CameraPosition', [X_b(1,i)-8, 0, 5]);
%         set(gca, 'CameraTarget', [X_b(1,i), 0, 0]);
%     else
%         set(gca, 'CameraPosition', [10, 0, 5]);
%         set(gca, 'CameraTarget', [17, 0, 0]);
%         set(gca, 'CameraUpVector', [0, 0, 1]);
%     end
%   
%     camzoom(15);
%     
    hold off
   
    drawnow;
end