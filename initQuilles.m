function [ pos_init ] = initQuilles( nbQuilles )
%initQuilles Summary of this function goes here
%   Detailed explanation goes here

global L Nb_q distInterQuille R_bas_q R_haut_q H_q  

pos_init=zeros(3,2*nbQuilles);

cmp_Quilles=0;
cmp_Lignes=0;
while(cmp_Quilles<nbQuilles)
    cmp_Lignes = cmp_Lignes + 1;
    for i=1:cmp_Lignes
        cmp_Quilles = cmp_Quilles + 1;
        pos_init(1,cmp_Quilles)= L + (cmp_Lignes-1) * distInterQuille * cos(pi/6);
        pos_init(2,cmp_Quilles)= (0.5 * distInterQuille * (cmp_Lignes-1)) - (i-1) * distInterQuille;
        pos_init(3,cmp_Quilles)= R_bas_q;
        pos_init(1, Nb_q + cmp_Quilles)= pos_init(1,cmp_Quilles);
        pos_init(2, Nb_q + cmp_Quilles)= pos_init(2,cmp_Quilles);
        pos_init(3, Nb_q + cmp_Quilles)= H_q - R_haut_q;
        if(cmp_Quilles == nbQuilles)
            break
        end
    end
    
    if(cmp_Quilles ==  nbQuilles)
        break
    end
    
end

        


end

