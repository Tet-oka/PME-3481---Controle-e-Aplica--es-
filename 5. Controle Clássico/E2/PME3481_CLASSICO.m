ul = 300/3.6;
uv = 0;

kt = 6.7829e+06;
ct = 2.109e+5;

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

C = [0,0,1,0,0,0;
     0,0,0,0,1,0];

D = [0	0;
     0	0];

E = [0	0	0;
     0	0	0;
     0	0	0;
     3400	2.425	0;
     0	0	0.0682865615784204;
     0	0	-0.0195104461652630];

Dft = [0 0; 0 0];
[num, den] = ss2tf(A,B,C,D, 2);
numTT = num(1,:);
tf_q = tf(numTT,den);
G_p = tf_q;
pole(G_p)
zero(G_p)




%% ITAE

initial_params = [20e+6,450000,20000]; % Parametros iniciais para processo de minimização
options = optimoptions('fmincon','MaxFunctionEvaluations',2e+15,'MaxIterations',2e+15);
result = fmincon(@ITAE,initial_params,[],[],[],[],[],[],[],options);
%Parametros PID



Kp = result(1); 
Ki = result(2);
Kd = result(3);

Kp = 10.33*10^8;
Ki = 0;
Kd = 0;

s = tf([1,0],[1]);
t_d = Kd/Kp;
N=10000; 

G_c = Ki/s + Kp*(1+(t_d*s)/((t_d*s)/N+1)); %Funcao de transferencia do PID
T = feedback(G_c*G_p,1); %Funcao de transferencia ref-saida em malha fechada
p = pole(T)%Polos em malha fechada sem filtro

G_f = tf([Ki],[Kd,Kp,Ki]); %Funcao de transferencia do filtro (pre-compensador)
T_f = series(G_f,T); %Funcao de transferencia do sistema em malha fechada com o filtro
p_f = pole(T_f)%Polos em malha fechada com o filtro

Frt = feedback(G_c,G_p,-1); %Funcao de transferencia entre entrada de controle e referencia
Frt_f = series(G_f,Frt);% Com filtro

tempo = 10; 
T_sim = 1/100;
t = 0:T_sim:tempo;

[q3,t1] = step(T,t);
[u,t2] = step(Frt,t);
[q3f,t1f] = step(T_f,t);
[uf,t2f] = step(Frt_f,t);
q3_ref = 13;

clf(figure(1))
figure(1)
%plot(t,q3_ref*q3, "b") 
%hold on
plot(t,q3_ref*q3f, "r") 
hold on
plot(t,q3_ref*ones(length(t),1), "g") 
legend("Sem filtro", "Com filtro", "Referência")
grid on
grid minor
title('\theta')
xlabel('Tempo (s)')
ylabel('Ângulo (°)')
set(gca,'FontSize',22)

clf(figure(2))
figure(2)
plot(t,u*q3_ref*pi/180, "b") 
hold on
plot(t,uf*q3_ref*pi/180, "r") 
hold on
plot(t,4500000*ones(length(t),1), "c") 
hold on
plot(t,-4500000*ones(length(t),1), "c") 
legend("Sem filtro", "Com filtro", "Limites do atuador")
grid on
grid minor
title('Torque')
xlabel('Tempo (s)')
ylabel('Torque (Nm)')
set(gca,'FontSize',22)

