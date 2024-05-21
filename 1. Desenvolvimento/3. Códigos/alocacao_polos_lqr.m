A = [0  0	0	1	0	0;
    0	0	0	0	1	0;
    0	0	0	0	0	1;
    (-13600000-kt)/4000	kt/4000	0	-2.425	0	0;
    0.0000121396*kt	-0.0000121396*kt	-0.669891*(0.0041+0.000041*ul)+0.00501874*(ul+uv)^2 0.0000121396*ct	-0.0000121396*ct	0;
    -2.2171*10^-7*kt	2.2171*10^-7*kt	(0.0041+0.000041*ul)+0.0000432299*ul^2+0.0000864599*ul*uv+0.0000432299*uv^2 	-2.2171*10^-7*ct	2.2171*10^-7*ct	0];

B = [0	0;
     0	0;
     0	0;
     0.000250000000000000	0;
     1.21396200179366e-05	-2.21709615514352e-07;
     -2.21709615514352e-07	6.33456044326720e-08];

C = [0	1	0	0	0	0;
     0	0	1	0	0	0];

D = [0	0  0;
     0	0  0];

%Sistema original
sys = ss(Amatrix,Bmatrix,Cmatrix,Dmatrix);
pol_orig  = pole(sys);
zeros_org = tzero(sys);

%Alocacao de Polos
P = [-1.1290+60.2891i,-1.1290-60.2891i,-0.8693+7.8727i,-0.8693-7.8727i,-3.0632+0.0000i,-0.0376+0.0000i];
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
GANHOSlqr = table(KSlqr, NUMS, FINAL);
Plqr;

KLQR = [Klqr(1,1) Klqr(1,2) Klqr(1,3) Klqr(1,4) Klqr(1,5) Klqr(1,6); Klqr(2,1) Klqr(2,2) Klqr(2,3) Klqr(2,4) Klqr(2,5) Klqr(2,6)];
save('Kmatrix_lqr.mat', 'KLQR');
syslqr = ss(Amatrix-Bmatrix*KLQR, Bmatrix, Cmatrix, Dmatrix);

%K Sensibilidade
    K11 =          -7781     ; 
    K12 =         -68556     ; 
    K13 =    -4.5172e+06     ; 
    K14 =         21.937     ; 
    K15 =         -78207     ; 
    K16 =    -4.6233e+06     ; 
    K21 =          52861     ; 
    K22 =     4.6575e+05     ; 
    K23 =     3.0688e+07   *10  ; 
    K24 =        -149.03     ; 
    K25 =     5.3132e+05     ; 
    K26 =     3.1409e+07   *10  ;


KSen = [K11 K12 K13 K14 K15 K16; K21 K22 K23 K24 K25 K26];
save('Kmatrix_sen.mat', 'KSen');
syssen = ss(Amatrix-Bmatrix*KSen, Bmatrix, Cmatrix, Dmatrix);
pol_sen = pole(syssen)

%Pol Butterworth
Wn = 8.8;
[a,b] = butter(6,Wn,"s");
sysbt = tf(a,b);
pol_bt  = pole(sysbt);
Kbt = place(Amatrix,Bmatrix,pol_bt);
KSbt = ['K11 ='; "K12 ="; "K13 =";"K14 =";"K15 =";"K16 =";"K21 =";"K22 =";"K23 =";"K24 =";"K25 =";"K26 ="];
NUMS = [Kbt(1,1); Kbt(1,2); Kbt(1,3); Kbt(1,4);Kbt(1,5);Kbt(1,6);Kbt(2,1);Kbt(2,2);Kbt(2,3);Kbt(2,4);Kbt(2,5);Kbt(2,6)];
FINAL = [";";";";";";";";";";";";";";";";";";";";";";";";];
GANHOSbt = table(KSbt, NUMS, FINAL);

%% Plot dos graficos


% figure(1)
% pzmap(sys);
% title("Polos e zeros do sistema original")
% 
% 
% figure(2)
% pzmap(sysap);
% title("Polos do sistema com alocação de polos")
% 
% figure(3)
% pzmap(syslqr);
% title("Polos do sistema com LQR")
% 
% figure(4)
% pzmap(syssen);
% title("Polos do sistema com análise de sensibilidade")
% 
% figure(5)
% pzmap(sysbt);
% title("Polos do sistema com alocação via Pol.Butterworth")