%ESTE CÓDIGO REMONTA O SISTEMA PARA ANÁLISE DE
%CONTROLABILIDADE/OBSERVABILIDADE E CALCULA OS GANHOS PARA UMA DETERMINADA
%ALOCAÇÃO DE POLOS

load('Amatrix.mat')
Amatrix = Expression1;
load('Bmatrix.mat')
Bmatrix = Expression1;
load('Cmatrix.mat')
Cmatrix = Expression1;
load('Dmatrix.mat')
Dmatrix = Expression1;

sys = ss(Amatrix,Bmatrix,Cmatrix,Dmatrix);

Q = [1 0 0 0 0 0;
     0 3 0 0 0 0;
     0 0 10 0 0 0; 
     0 0 0 8.688 0 0; 
     0 0 0 0 26.064 0; 
     0 0 0 0 0 86.88];

R = [1 0;
     0 2];

[Klqr,S,Plqr] = lqr(sys,Q,R);
KSlqr = ['K11 ='; "K12 ="; "K13 =";"K14 =";"K15 =";"K16 =";"K21 =";"K22 =";"K23 =";"K24 =";"K25 =";"K26 ="];
NUMS = [Klqr(1,1); Klqr(1,2); Klqr(1,3); Klqr(1,4);Klqr(1,5);Klqr(1,6);Klqr(2,1);Klqr(2,2);Klqr(2,3);Klqr(2,4);Klqr(2,5);Klqr(2,6)];
FINAL = [";";";";";";";";";";";";";";";";";";";";";";";";];
GANHOSlqr = table(KSlqr, NUMS, FINAL)
Plqr;
