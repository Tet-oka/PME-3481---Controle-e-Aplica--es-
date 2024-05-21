%% Definição das matrizes do espaço de estados
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

D = [0	0  0;
     0	0  0];

% O LQR não funciona para impulso, assim, testa-se sem perturbações para
% analisar se converge pra 0, e não pra ~-10° por conta da perturbacao gravitacional 
Dlqr = [0	0;
        0	0];

E = [0	0	0;
     0	0	0;
     0	0	0;
     3400	2.425	0;
     0	0	0.0682865615784204;
     0	0	-0.0195104461652630];

% Ks advindos do LQR modificado
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


K = [K11, K12, K13, K14, K15, K16;
       K21, K22, K23, K24, K25, K26];

%% Condições de integração
%Integração
tempo = 10; 
T_sim = 1/100;
t = 0:T_sim:tempo;

kr = 13600000;
cr = 970000;

%Condições iniciais de integração:
q1_0 = (389300)/kr; %q1 de equilíbrio dinâmico a 80m/s
q2_0 = (389300)/kt + (389300)/kr; %q2 de equilíbrio dinâmico a 80m/s
theta_0 = 13*pi/180;
q3_0 = 0;
q1p_0 = -3; 
q2p_0 = -3; 
thetap_0 = 0;
q3p_0 = 0;
x0 =  [q1_0 q2_0 theta_0 q1p_0 q2p_0 thetap_0];


%% Perturbações
Amp = 0.05; %Amplitude da oscilação senoidal da pista
freq = 7.14; %Frequência da amplitude senoidal da pista

y_ext = Amp*sin(freq*(t));
yp_ext = Amp*freq*cos(freq*(t));
g = 9.81*ones(1, length(t)); %Gravidade modelada como perturbação constante pois não entra na matriz A

w = [y_ext; yp_ext; g];
%% Controle
% Sistema original sem controle
sysnc = ss(A, E, C, D);
[ync,t,xnc] = lsim(sysnc,w,t,x0);


%% Sistema controlado com alocação de polos
P = [-1.25+60.32i,-1.25-60.32i,-0.59-3.33i,-0.59+3.33i, -0.971775, -1];
KAP = place(A,B,P); %Matriz de ganhos da alocação de polos
sysap = ss(A-B*KAP, E, -KAP, D);
polap = pole(sysap);
[uAP,t,xAP] = lsim(sysap,w,t,x0);

%% Sistema controlado com LQR
Q = [0.02 0 0 0 0 0;
     0 5 0 0 0 0;
     0 0 20 0 0 0; 
     0 0 0 0.02 0 0; 
     0 0 0 0 10 0; 
     0 0 0 0 0 40];

R = 10e-08*[1 0;
     0 0.1];

sys_lqr = ss(A,B,C,Dlqr);
[KLQR,S,Plqr] = lqr(sys_lqr,Q,R);
syslqr = ss(A-B*KLQR, E, -K, D);
pollqr = pole(syslqr);
[ulqr,t,xlqr] = lsim(syslqr,w,t,x0);

%% Sistema controlado com análise de sensibilidade
    K11 =       0    ; 
    K12 =     -7000   ; 
    K13 =    0    ; 
    K14 =    0     ; 
    K15 =    -8000    ; 
    K16 =    1000    ; 
    K21 =     0     ; 
    K22 =     10000000    ; 
    K23 =     24000000 ; 
    K24 =     0     ; 
    K25 =      0 ; 
    K26 =     80000000;

KSEN = [K11 K12 K13 K14 K15 K16; K21 K22 K23 K24 K25 K26];

syssen = ss(A-B*KSEN, E, -KSEN, D);
[uSEN,t,xSEN] = lsim(syssen,w,t,x0);




%% Observador de Estados de Ordem Reduzida

n_ = 6; %Número de variáveis de estado
r_ = 2; %Número de entradas de controle
l_ = 3; %Números de entradas de distúrbio
m_ = 2; %Número de observações

V = [1,0,0,0,0,0;
     0,1,0,0,0,0;
     0,0,0,1,0,0;
     0,0,0,0,0,1];

T = [C;V];
inv_T = inv(T);
M = inv_T(:,1:m_);
N = inv_T(:,m_+1:n_);

A11 = C*A*M;
A12 = C*A*N;
A21 = V*A*M;
A22 = V*A*N;
B1 = C*B;
B2 = V*B;

p_obs = [-19.5, -23.5, -20.1, -20]; %Ao menos 5 vezes mais a esquerda que os polos do sistema
% p_obs = [-20, -21, -22, -23];

J = (place(A22.',A12.',p_obs)).';

F = A22 - J*A12;
G = A21 -J*A11 + F*J;
H = B2 - J*B1;
S = M + N*J;

[obs_autovec, obs_autoval] = eig(F);

% Novo sistema na forma de espaço de estados

A_obs = [A, zeros(n_,n_-m_);
        G*C, F];

B_obs = [B;H];

E_obs = [E;zeros(n_-m_,l_)];

C_obs = [C,zeros(m_,n_-m_)];

D_obs = zeros(n_-m_,m_);

A_obs_c = A_obs - B_obs*KSEN*[S*C,N];

sys_obs_u = ss(A_obs_c,E_obs,-KSEN*[S*C,N],0);

% x0_obs = [x0 0.9 0 0 -5];
x0_obs = [x0 0.9 0 0 -5]; %Erro não nulo e ajustado de acordo com os resultados observados
[u_obs,t,xdes] = lsim(sys_obs_u,w,t,x0_obs);


figure(12)
plot(t, u_obs(:,1)/1000, "r") 
hold on
grid on
grid minor
title('Esforços de força com y_{ext} = 0.05sen(7.14t)')
xlabel('Tempo (s)')
ylabel('Força (KN)')
set(gca,'FontSize',22)

xest = (S*C*xdes(:,1:6)' + N*xdes(:,7:10)')';

%xdes(:,1:6) -> Desempenho do sistema malha fechada com o estimador
%xest(:,1:6) -> Valores estimados


% 
%% Seguidor de referências LQ

% Referência 1 -> referência constante em zero 
    Q_ref1 = [0.014 0 0 0 0 0;
             0 5 0 0 0 0;
             0 0 20 0 0 0; 
             0 0 0 0.014 0 0; 
             0 0 0 0 10 0; 
             0 0 0 0 0 40];
    
    
    R_ref1 = [1*10^-8 0;
             0 1*10^-9];
    
    
    x_ref1 = [zeros(1,length(t)),
             zeros(1,length(t)),
             zeros(1,length(t)),
             zeros(1,length(t)),
             zeros(1,length(t)),
             zeros(1,length(t)),];
    
    [K_lq,p_lq,poles_lq] = lqr(sys_lqr,Q_ref1,R_ref1);
    
    A_c_lq = (A-B*K_lq);
    eta_ss1 = ss(A_c_lq,Q_ref1,eye(6),zeros(6,6));
    
    eta01 = p_lq*x_ref1(:,end);
    
    eta = lsim(eta_ss1,x_ref1,t,eta01);
    
    sys_ref1 = ss(A_c_lq,B*inv(R_ref1)*B',-K_lq,inv(R_ref1)*B');
    
    [ur1,t,xr1] = lsim(sys_ref1,flip(eta,2),t, x0);

% % Referência 2 -> Degrau na metade do tempo
%     Q_ref2 = [0.014 0 0 0 0 0;
%              0 15 0 0 0 0;
%              0 0 5.987 0 0 0; 
%              0 0 0 0.014 0 0; 
%              0 0 0 0 8 0; 
%              0 0 0 0 0 38.8];
%     
%     R_ref2 = [5*10^-8 0;
%              0 9*10^-13];
%     
%     
%     x_ref2 = [zeros(1,length(t)),
%                      zeros(1,length(t)),
%                      theta_0*ones(1, length(t)/2 + 1/2) zeros(1, length(t)/2 -1/2),
%                      zeros(1,length(t)),
%                      zeros(1,length(t)),
%                      zeros(1,length(t))];
% 
%     [K_lq,p_lq,poles_lq] = lqr(sys_lqr,Q_ref2,R_ref2);
%     
%     A_c_lq = (A-B*K_lq);
%     eta_ss2 = ss(A_c_lq,Q_ref2,eye(6),zeros(6,6));
%     
%     eta02 = p_lq*x_ref2(:,end);
%     
%     eta = lsim(eta_ss2,x_ref2,t,eta02);
%     
%     sys_ref2 = ss(A_c_lq,B*inv(R_ref2)*B',-K_lq,inv(R_ref2)*B');
%     
%     [ur2,t,xr2] = lsim(sys_ref2,flip(eta,2),t, x0);

    % Referência 3 -> Rampa em theta chegando a zero em 4 segundos
    Q_ref3 = [0.014 0 0 0 0 0;
             0 5 0 0 0 0;
             0 0 500.987 0 0 0; 
             0 0 0 0.014 0 0; 
             0 0 0 0 8 0; 
             0 0 0 0 0 3.8];
    
    R_ref3 = [2*10^-6 0;
             0 10*10^-12];

    x_ref3 = [zeros(1,length(t)),
             zeros(1,length(t)),
             theta_0*(1-t.'/5),
             zeros(1,length(t)),
             zeros(1,length(t)),
             zeros(1,length(t))];

    [K_lq,p_lq,poles_lq] = lqr(sys_lqr,Q_ref3,R_ref3);
    
    A_c_lq = (A-B*K_lq);
    eta_ss3 = ss(A_c_lq,Q_ref3,eye(6),zeros(6,6));
    
    eta03 = p_lq*x_ref3(:,end);
    
    eta = lsim(eta_ss3,x_ref3,t,eta03);
    
    sys_ref3 = ss(A_c_lq,B*inv(R_ref3)*B',-K_lq,inv(R_ref3)*B');
    
    [ur3,t,xr3] = lsim(sys_ref3,flip(eta,2),t, x0);



%% Seguidor de referências variáveis exógenas

    K11 =          -7.781     ; 
    K12 =         -685560     ; 
    K13 =    -4.5172e+02     ; 
    K14 =         21.937     ; 
    K15 =         -70000     ; 
    K16 =    -4.6233e+01     ; 
    K21 =          52861     ; 
    K22 =     4.6575e+05     ; 
    K23 =     3.0688e+08 ; 
    K24 =        -149.03     ; 
    K25 =     5.3132e+05     ; 
    K26 =     3.1409e+06 ;

K = [K11 K12 K13 K14 K15 K16; K21 K22 K23 K24 K25 K26];

K = KLQR;

%Rampa ou degrau
A_r = [0 0 0 0 0 0;
       0 0 0 0 1 0;
       0 0 0 0 0 1;
       0 0 0 0 0 0;
       0 0 0 0 0 0;
       0 0 0 0 0 0];
% Seno
% omega = 0.5;
% omega2 = 8;
% zeta = 0.4;
% ac = 0.001;

% 
% A_r = [0 0 0 0 0 0;
%       0 0 0 0 1 0;
%       0 0 0 0 0 1;
%       0 0 0 0 0 0;
%       %0 0 0 0 0 0;
%       0 -omega2^2 0 0 -2*zeta*omega2 0;
%       0 0 -omega^2 0 0 0];

M = [0 1 0 0 0 0;
     0 0 1 0 0 0];


A_cl_inv = inv(A-B*K);
N = inv(M*A_cl_inv*B)*M*A_cl_inv;
G_r = N*(A-A_r);
A_ex_cl = [[A-B*G_r,B*(K-G_r)];[(A_r - A +B*G_r),(A_r - B*(K-G_r))]];
E_ex_cl = [-B*N*E + E ; B*N*E-E];
K_ex = [-G_r,K-G_r];


seg_exo_ss = ss(A_ex_cl,E_ex_cl,K_ex,0);

x0_seg = [x0 -q1_0 0 0 -q1p_0 -q2p_0 -theta_0/5]; %RAMPA

% x0_seg = [x0 -q1_0 -q2_0 -theta_0 -q1p_0 -q2p_0 -thetap_0]; %Degrau
% x0_seg = [x0 -q1_0 0 0 -q1p_0 -q2p_0 -thetap_0]; %Seno theta
% x0_seg = [x0 -q1_0 0 0 -q1p_0 0 0]; %Seno theta e q2

[u,t_exo,xex] = lsim(seg_exo_ss,w,t,x0_seg);

uex = u - (N*E*w).';


% x_ref_ext = zeros(length(t));
% x_ref_exq = zeros(length(t));

x_ref_ext = theta_0*(1-t/5);
x_ref_exq = zeros(length(t));

%% Simulacao realista
tpos = 0:T_sim:6;
Amp = 0; %Amplitude da oscilação senoidal da pista
freq = 0; %Frequência da amplitude senoidal da pista

y_extpos = Amp*sin(freq*(tpos));
yp_extpos = Amp*freq*cos(freq*(tpos));
gpos = 9.81*ones(1, length(tpos)); %Gravidade modelada como perturbação constante pois não entra na matriz A

wpos = [y_extpos; yp_extpos];

A2 = [0	0	0	0	1	0	0	0;
0	0	0	0	0	1	0	0;
0	0	0	0	0	0	1	0;
0	0	0	0	0	0	0	1;
-(1455491/400)	95491/400	0	0	-(11357/400)	10387/400	0	0;
11.5922	-58.1397	-847.163+0.00501874*(ul+uv)^2+(-0.00274655-0.0000274655*ul) 	46.5474	1.26094	-5.40218	-75.3706	4.14124;
-0.211713	-5.13643	-97.3362+0.0000432299*ul^2+0.0000864599*ul*uv+0.0000432299*uv^2+(0.00078473 +7.8473*10^-6*ul) 	5.34814	-0.023029	-0.452786	-8.65982	0.475815;
0	28717/5	104530	-(45717/5)	0	25549/50	9299.84	-(102681/200)];

E2 = [0	0 0;
    0	0 0;
    0	0 0;
    0	0 0;
    3400 97/40 0 ;
    0	0 0.0682865615784204;
    0	0 -0.0195104461652630;
    3400	97/40 0];

E2 = [0	0;
    0	0;
    0	0;
    0	0;
    3400 97/40;
    0	0 ;
    0	0 ;
    3400	97/40 ];

sinal = xdes(:,1:6);
r = sinal(1,3);
j = 1;
while r > 0
    j = j+1;
    r = sinal(j,3);
end

sys_pos = ss(A2,E2,zeros(8,8), zeros(8,2));
x0pos = [sinal(j,1) sinal(j,2) sinal(j,3) 0 sinal(j,4) sinal(j,5) sinal(j,6) sinal(j,6)*18.2];

[urpos,t,xpos] = lsim(sys_pos,wpos,tpos, x0pos);
x3_tot = [sinal(1:j, 3); xpos(:,3)];

kt = 11486.8*10^3;
ct = 102.2*10^3;
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

C = [0,1,0,0,0,0;
     0,0,1,0,0,0];

D = [0	0  0;
     0	0  0];
t = 0:0.01:10;
Amp = 0.05; %Amplitude da oscilação senoidal da pista
freq = 7.14; %Frequência da amplitude senoidal da pista

y_ext = Amp*sin(freq*(t));
yp_ext = Amp*freq*cos(freq*(t));
g = 9.81*ones(1, length(t)); %Gravidade modelada como perturbação constante pois não entra na matriz A

w = [y_ext; yp_ext; g];

sysnc = ss(A, E, C, D);
[ync,t,xnc1] = lsim(sysnc,w,t,x0);
w = w.';

sinal = xnc1;
r = sinal(1,3);
k = 1;
while r > 0
    k = k+1;
    r = sinal(k,3);
end
x0pos = [sinal(k,1) sinal(k,2) sinal(k,3) 0 sinal(k,4) sinal(k,5) sinal(k,6) sinal(k,6)*18.2];
[urpos,t,xpos] = lsim(sys_pos,wpos,tpos, x0pos);
x3_totnc = [sinal(1:k, 3); xpos(:,3)];


if length(x3_tot) > length(x3_totnc)
    x3_tot = x3_tot(1:length(x3_totnc));
else
    x3_totnc = x3_totnc(1:length(x3_tot));
end
temp = 0:T_sim:length(x3_tot)*T_sim;
temp = temp(1:length(temp)-1);
% 
% 
% clf(figure(10))
% figure(10)
% plot(temp,180/pi*x3_tot, "b") 
% hold on
% plot(temp,180/pi*x3_totnc(1:length(x3_tot)), "r")
% legend("Sistema controlado", "Sistema não controlado")
% grid on
% grid minor
% title('Comparação dos sistemas em \theta com vento de 3,6 m/s (7 nós)')
% xlabel('Tempo (s)')
% ylabel('Ângulo (°)')
% set(gca,'FontSize',22)



