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
P = [-1.25+6.31i;-1.25-6.31i;-0.59+3.33i;-0.59-3.33i;-0.97+0.00i;-0.1+0.00i];
KAP = place(Amatrix,Bmatrix,P);
save('Kmatrix_ap.mat', 'KAP');
sysap = ss(Amatrix-Bmatrix*KAP, Bmatrix, Cmatrix, Dmatrix);
KSAP = ['K11 ='; "K12 ="; "K13 =";"K14 =";"K15 =";"K16 =";"K21 =";"K22 =";"K23 =";"K24 =";"K25 =";"K26 ="];
NUMS = [KAP(1,1); KAP(1,2); KAP(1,3); KAP(1,4);KAP(1,5);KAP(1,6);KAP(2,1);KAP(2,2);KAP(2,3);KAP(2,4);KAP(2,5);KAP(2,6)];
FINAL = [";";";";";";";";";";";";";";";";";";";";";";";";];
GANHOS = table(KSAP, NUMS, FINAL)

%LQR
r = 1;
Q = [0.01 0 0 0 0 0;
     0 10 0 0 0 0;
     0 0 10 0 0 0; 
     0 0 0 0.01/64 0 0; 
     0 0 0 0 10/64 0; 
     0 0 0 0 0 100000/64];
R = [12*r 0;
     0 1*r];
[Klqr,S,Plqr] = lqr(sys,Q,R);
KSlqr = ['K11 ='; "K12 ="; "K13 =";"K14 =";"K15 =";"K16 =";"K21 =";"K22 =";"K23 =";"K24 =";"K25 =";"K26 ="];
NUMS = [Klqr(1,1); Klqr(1,2); Klqr(1,3); Klqr(1,4);Klqr(1,5);Klqr(1,6);Klqr(2,1);Klqr(2,2);Klqr(2,3);Klqr(2,4);Klqr(2,5);Klqr(2,6)];
FINAL = [";";";";";";";";";";";";";";";";";";";";";";";";];
GANHOSlqr = table(KSlqr, NUMS, FINAL);
Plqr;

KLQR = [Klqr(1,1) Klqr(1,2) Klqr(1,3) Klqr(1,4) Klqr(1,5) Klqr(1,6); Klqr(2,1) Klqr(2,2) Klqr(2,3) Klqr(2,4) Klqr(2,5) Klqr(2,6)];
save('Kmatrix_lqr.mat', 'KLQR');
syslqr = ss(Amatrix-Bmatrix*KLQR, Bmatrix, Cmatrix, Dmatrix);

%K Sensibilidade
K11 =        -284.3 ; 
K12 =         -5160 ; 
K13 =    -7.4562e+03; 
K14 =         3.6209; 
K15 =          -9090; 
K16 =    -9.6314e+03; 
K21 =          52353; 
K22 =     2.6127e+03; 
K23 =     25.0393e+06; 
K24 =         -147.6; 
K25 =      5.262e+03; 
K26 =     7.107e+07;

KSen = [K11 K12 K13 K14 K15 K16; K21 K22 K23 K24 K25 K26];
save('Kmatrix_sen.mat', 'KSen');
syssen = ss(Amatrix-Bmatrix*KSen, Bmatrix, Cmatrix, Dmatrix);

%Pol Butterworth
Wn = 0.977;
[a,b] = butter(6,Wn,"s");
sysbt = tf(a,b);
pol_bt  = pole(sysbt);
Kbt = place(Amatrix,Bmatrix,pol_bt);
KSbt = ['K11 ='; "K12 ="; "K13 =";"K14 =";"K15 =";"K16 =";"K21 =";"K22 =";"K23 =";"K24 =";"K25 =";"K26 ="];
NUMS = [Kbt(1,1); Kbt(1,2); Kbt(1,3); Kbt(1,4);Kbt(1,5);Kbt(1,6);Kbt(2,1);Kbt(2,2);Kbt(2,3);Kbt(2,4);Kbt(2,5);Kbt(2,6)];
FINAL = [";";";";";";";";";";";";";";";";";";";";";";";";];
GANHOSbt = table(KSbt, NUMS, FINAL);

%% Plot dos graficos


figure(1)
pzmap(sys);
title("Polos do sistema original")


figure(2)
pzmap(sysap);
title("Polos do sistema com alocação de polos")

figure(3)
pzmap(syslqr);
title("Polos do sistema com LQR")

figure(4)
pzmap(syssen);
title("Polos do sistema com análise de sensibilidade")

figure(5)
pzmap(sysbt);
title("Polos do sistema com alocação via Pol.Butterworth")