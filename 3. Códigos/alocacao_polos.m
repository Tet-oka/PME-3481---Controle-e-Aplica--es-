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
Pol  = pole(sys);
P = [-1.49+79.59i,-1.49-79.59i,-1+8.68i,-1- 8.68i,-1+0.000i,-1+0.000i];
K = place(Amatrix,Bmatrix,P);
save('Kmatrix.mat', 'K');
KS = ['K11 ='; "K12 ="; "K13 =";"K14 =";"K15 =";"K16 =";"K21 =";"K22 =";"K23 =";"K24 =";"K25 =";"K26 ="];
NUMS = [K(1,1); K(1,2); K(1,3); K(1,4);K(1,5);K(1,6);K(2,1);K(2,2);K(2,3);K(2,4);K(2,5);K(2,6)];
FINAL = [";";";";";";";";";";";";";";";";";";";";";";";";];
%PARA SUBSTITUIR NA ODE, COPIE E COLE A TABELA EM UM OUTRO SCRIPT QUALQUER E DÊ O COMANDO CTRL+F, Find
%(") e Replace (). OU SEJA, SUBSTITUA AS ASPAS POR NADA (EXCLUA-AS DE UMA
%VEZ). BASTA ENTÃO COPIAR DENTRO DA ODE CONTROLADA POR ALOCAÇÃO DE POLOS
GANHOS = table(KS, NUMS, FINAL)



