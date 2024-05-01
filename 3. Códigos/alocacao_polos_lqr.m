load('Amatrix.mat')
Amatrix = Expression1;
load('Bmatrix.mat')
Bmatrix = Expression1;
load('Cmatrix.mat')
Cmatrix = Expression1;
load('Dmatrix.mat')
Dmatrix = Expression1;

%Sistema original
sys = ss(Amatrix,Bmatrix,Cmatrix,Dmatrix);
pol_orig  = pole(sys);
zeros_org = tzero(sys);

%Alocacao de Polos
P = [-1.2539+60.3160i,-1.2539-60.3160i,-1.5917+3.3334i,-1.5917-3.3334i,-1.9718+0.0000i,-1.9770+0.0000i];
KAP = place(Amatrix,Bmatrix,P);
save('Kmatrix_ap.mat', 'KAP');
sysap = ss(Amatrix-Bmatrix*KAP, Bmatrix, Cmatrix, Dmatrix);
KSAP = ['K11 ='; "K12 ="; "K13 =";"K14 =";"K15 =";"K16 =";"K21 =";"K22 =";"K23 =";"K24 =";"K25 =";"K26 ="];
NUMS = [KAP(1,1); KAP(1,2); KAP(1,3); KAP(1,4);KAP(1,5);KAP(1,6);KAP(2,1);KAP(2,2);KAP(2,3);KAP(2,4);KAP(2,5);KAP(2,6)];
FINAL = [";";";";";";";";";";";";";";";";";";";";";";";";];
GANHOS = table(KSAP, NUMS, FINAL)


%LQR
Q = [0.014 0 0 0 0 0;
     0 1.4 0 0 0 0;
     0 0 2.987 0 0 0; 
     0 0 0 0.014 0 0; 
     0 0 0 0 2 0; 
     0 0 0 0 0 3];

R = [0.9951 0;
     0 0.5327];

[Klqr,S,Plqr] = lqr(sys,Q,R);
KSlqr = ['K11 ='; "K12 ="; "K13 =";"K14 =";"K15 =";"K16 =";"K21 =";"K22 =";"K23 =";"K24 =";"K25 =";"K26 ="];
NUMS = [Klqr(1,1); Klqr(1,2); Klqr(1,3); Klqr(1,4);Klqr(1,5);Klqr(1,6);Klqr(2,1);Klqr(2,2);Klqr(2,3);Klqr(2,4);Klqr(2,5);Klqr(2,6)];
FINAL = [";";";";";";";";";";";";";";";";";";";";";";";";];
GANHOSlqr = table(KSlqr, NUMS, FINAL)
Plqr;

KLQR = [Klqr(1,1) Klqr(1,2) Klqr(1,3) Klqr(1,4) Klqr(1,5) Klqr(1,6); Klqr(2,1) Klqr(2,2) Klqr(2,3) Klqr(2,4) Klqr(2,5) Klqr(2,6)];
save('Kmatrix_lqr.mat', 'KLQR');
syslqr = ss(Amatrix-Bmatrix*KLQR, Bmatrix, Cmatrix, Dmatrix);

%K Sensibilidade
   K11 =    -1.3725e+07     ; 
    K12 =     1.1443e+06     ; 
    K13 =     5.0974e+05     ; 
    K14 =          69024     ; 
    K15 =          41674     ; 
    K16 =      1.196e+05     ; 
    K21 =    -4.5653e+08     ; 
    K22 =    -9.4356e+07     ; 
    K23 =     3.1819e+08     ; 
    K24 =      1.671e+07     ; 
    K25 =    -3.3429e+07     ; 
    K26 =     8.1579e+07     ; 

KSen = [K11 K12 K13 K14 K15 K16; K21 K22 K23 K24 K25 K26];
save('Kmatrix_sen.mat', 'KSen');
syssen = ss(Amatrix-Bmatrix*KSen, Bmatrix, Cmatrix, Dmatrix);
pol_sen = pole(syssen)

%Pol Butterworth
Wn = 8.8;
[a,b] = butter(6,Wn,"s");
sysbt = tf(a,b);
pol_bt  = pole(sysbt)
Kbt = place(Amatrix,Bmatrix,pol_bt);
KSbt = ['K11 ='; "K12 ="; "K13 =";"K14 =";"K15 =";"K16 =";"K21 =";"K22 =";"K23 =";"K24 =";"K25 =";"K26 ="];
NUMS = [Kbt(1,1); Kbt(1,2); Kbt(1,3); Kbt(1,4);Kbt(1,5);Kbt(1,6);Kbt(2,1);Kbt(2,2);Kbt(2,3);Kbt(2,4);Kbt(2,5);Kbt(2,6)];
FINAL = [";";";";";";";";";";";";";";";";";";";";";";";";];
GANHOSbt = table(KSbt, NUMS, FINAL)

%% Plot dos graficos


% figure(1)
% pzmap(sys);
% title("Polos do sistema original")
% 
% 
figure(2)
pzmap(sysap);
title("Polos do sistema com alocação de polos")
% 
figure(3)
pzmap(syslqr);
title("Polos do sistema com LQR")

% figure(4)
% pzmap(syssen);
% title("Polos do sistema com análise de sensibilidade")

figure(5)
pzmap(sysbt);
title("Polos do sistema com alocação via Pol.Butterworth")