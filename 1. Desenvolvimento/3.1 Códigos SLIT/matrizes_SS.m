%% Definição das matrizes do espaço de estados
ul = 300/3.6;
uv = 3;

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

C = [0,1,0,0,0,0;
     0,0,1,0,0,0];

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
Amp = 0; %Amplitude da oscilação senoidal da pista
freq = 0; %Frequência da amplitude senoidal da pista

y_ext = Amp*sin(freq*(t));
yp_ext = Amp*freq*cos(freq*(t));
g = 9.81*ones(1, length(t)); %Gravidade modelada como perturbação constante pois não entra na matriz A

w = [y_ext; yp_ext; g];
%% Controle
% Sistema original sem controle
sysnc = ss(A, B, C, zeros(2,2));
% pzmap(sysnc)
% bode(sysnc)

% [b1,a1] = ss2tf(A, B, C, zeros(2,2), 1); %q2
% G11 = tf(b1(1,:), a1) ; %FORCA
% G12 = tf(b1(2,:), a1) ; %TORQUE
% 
% [b2,a2] = ss2tf(A, B, C, zeros(2,2), 2); %THETA
% G21 = tf(b2(1,:), a2) ; %FORCA
% G22 = tf(b2(2,:), a2) ; %TORQUE
% 
% plotoptions = bodeoptions;
% plotoptions . Grid = 'on';
% plotoptions . FreqUnits = 'Hz';
% plotoptions . FreqScale = 'log';
% plotoptions . Title . FontSize = 12;
% plotoptions . XLabel . FontSize = 10;
% plotoptions . YLabel . FontSize = 10;
% 
% 
% figure(101)
% bodeplot(G11,  plotoptions)
% hold on
% bodeplot(G12, plotoptions)
% legend("Força", "Torque", 'FontSize', 10)
% title("Diagrama de Bode para q_2")
% 
% figure(102)
% bodeplot(G21,  plotoptions)
% hold on
% bodeplot(G22, plotoptions)
% legend("Força", "Torque", 'FontSize', 10)
% title("Diagrama de Bode para \theta")

sysnc = ss(A, E, C, zeros(2,3));
[ync,t,xnc] = lsim(sysnc,w,t,x0);

%Sistema controlado com alocação de polos
P = [-1.64+71.56i,-1.64-71.56i,-0.85+7.38i,-0.85-7.38i,-0.9660,-0.1];
KAP1 = place(A,B,P); %Matriz de ganhos da alocação de polos
sysap1 = ss(A-B*KAP1, E, -KAP1, D);
[uAP1,t,xAP1] = lsim(sysap1,w,t,x0);

P = [-1.64+71.56i,-1.64-71.56i,-0.85+7.38i,-0.85-7.38i,-0.9660,-1];
KAP2 = place(A,B,P); %Matriz de ganhos da alocação de polos
sysap2 = ss(A-B*KAP2, E, -KAP2, D);
[uAP2,t,xAP2] = lsim(sysap2,w,t,x0);

P = [-1.64+71.56i,-1.64-71.56i,-0.85+7.38i,-0.85-7.38i,-0.9660,-10];
KAP3 = place(A,B,P); %Matriz de ganhos da alocação de polos
sysap3 = ss(A-B*KAP3, E, -KAP3, D);
[uAP3,t,xAP3] = lsim(sysap3,w,t,x0);

% 
% Sistema controlado com LQR
    Q = [0.02 0 0 0 0 0;
             0 5 0 0 0 0;
             0 0 20 0 0 0; 
             0 0 0 0.02 0 0; 
             0 0 0 0 10 0; 
             0 0 0 0 0 40];
    
    
    R = [1*10^-8 0;
             0 1*10^-9];
% 
sys_lqr = ss(A,B,C,Dlqr);
[KLQR,S,Plqr] = lqr(sys_lqr,Q,R);
syslqr = ss(A-B*KLQR, E, -KLQR, D);
pollqr = pole(syslqr);
[ulqr,t,xlqr] = lsim(syslqr,w,t,x0);
% % 
% Sistema controlado com análise de sensibilidade
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
    K26 = 80000000;

    KSEN = [K11 K12 K13 K14 K15 K16; K21 K22 K23 K24 K25 K26];

syssen = ss(A-B*KSEN, E, -KSEN, D);
[uSEN,t,xSEN] = lsim(syssen,w,t,x0);

%% Gráficos
% clf(figure(1))
% figure(1)
% plot(t,(180/pi*xnc(:,3)), "b")
% grid on
% grid minor
% title('Variação de \theta sem controle')
% xlabel('Tempo (s)')
% ylabel('Ângulo (°)')
% 
% clf(figure(2))
% figure(2)
% plot(t,(180/pi*xSEN(:,3)), "b")
% grid on
% grid minor
% title('Variação de \theta com análise sensibilidade')
% xlabel('Tempo (s)')
% ylabel('Ângulo (°)')
%
% clf(figure(3))
% figure(3)
% plot(t,(180/pi*xlqr(:,3)), "b")
% grid on
% grid minor
% title('Variação de \theta com LQR')
% xlabel('Tempo (s)')
% ylabel('Ângulo (°)')
% 
% clf(figure(4))
% figure(4)
% nexttile
% plot(t, uSEN(:,1)/1000, "b")
% grid on
% grid minor
% title('Esforços de força com análise sensibilidade')
% xlabel('Tempo (s)')
% ylabel('Força (KN)')
% nexttile
% plot(t, uSEN(:,2), "b")
% grid on
% grid minor
% title('Esforços de torque com análise sensibilidade')
% xlabel('Tempo (s)')
% ylabel('Torque (Nm)')
% nexttile
% plot(t, ulqr(:,1)/1000, "b")
% grid on
% grid minor
% title('Esforços de força com LQR')
% xlabel('Tempo (s)')
% ylabel('Força (KN)')
% nexttile
% plot(t, ulqr(:,2), "b")
% grid on
% grid minor
% title('Esforços de torque com LQR')
% xlabel('Tempo (s)')
% ylabel('Torque (Nm)')
% nexttile
% plot(t, uAP(:,1)/1000, "b")
% grid on
% grid minor
% title('Esforços de força com alocação de polos')
% xlabel('Tempo (s)')
% ylabel('Força (KN)')
% nexttile
% plot(t, uAP(:,2), "b")
% grid on
% grid minor
% title('Esforços de torque com alocação de polos')
% xlabel('Tempo (s)')
% ylabel('Torque (Nm)')

%% Gráficos Polos
% % figure(1)
% % pzmap(sys);
% title("Polos e zeros do sistema original")
% figure(2)
% pzmap(sysnc);
% % title("Polos do sistema com alocação de polos")
% % figure(3)
% % pzmap(syslqr);
% % title("Polos do sistema com LQR")
% figure(40)
% pzmap(syssen);
% title("Polos do sistema com análise de sensibilidade")
% % figure(5)
% % pzmap(sysbt);
% % title("Polos do sistema com alocação via Pol.Butterworth")
% %% Observador de Estados de Ordem Reduzida
% n_ = 6; %Número de variáveis de estado
% r_ = 2; %Número de entradas de controle
% l_ = 3; %Números de entradas de distúrbio
% m_ = 2; %Número de observações
% 
% V = [1,0,0,0,0,0;
%      0,1,0,0,0,0;
%      0,0,0,1,0,0;
%      0,0,0,0,0,1];
% 
% T = [C;V];
% inv_T = inv(T);
% M = inv_T(:,1:m_);
% N = inv_T(:,m_+1:n_);
% 
% A11 = C*A*M;
% A12 = C*A*N;
% A21 = V*A*M;
% A22 = V*A*N;
% B1 = C*B;
% B2 = V*B;
% 
% p_obs = [-10, -11, -12, -13]; %Ao menos 5 vezes mais a esquerda que os polos do sistema
% 
% J = (place(A22.',A12.',p_obs)).';
% 
% F = A22 - J*A12;
% G = A21 -J*A11 + F*J;
% H = B2 - J*B1;
% S = M + N*J;
% 
% [obs_autovec, obs_autoval] = eig(F);
% 
% % Novo sistema na forma de espaço de estados
% 
% A_obs = [A, zeros(n_,n_-m_);
%         G*C, F];
% 
% B_obs = [B;H];
% 
% E_obs = [E;zeros(n_-m_,l_)];
% 
% C_obs = [C,zeros(m_,n_-m_)];
% 
% D_obs = zeros(n_-m_,m_);
% 
% A_obs_c = A_obs - B_obs*K*[S*C,N];
% 
% sys_obs_u = ss(A_obs_c,E_obs,-KSEN*[S*C,N],0);
% 
% x0_obs = [x0 -1 0.1 -1 -2.3]; %Erro não nulo e ajustado de acordo com os resultados observados
% [u_obs,t,xdes] = lsim(sys_obs_u,w,t,x0_obs);
% xest = (S*C*xdes(:,1:6)' + N*xdes(:,7:10)')';
% 
% xdes(:,1:6) -> Desempenho do sistema malha fechada com o estimador
% xest(:,1:6) -> Valores estimados
% 
% clf(figure(5))
% figure(5)
% plot(t,(180/pi*xSEN(:,3)), "b") %Por aqui o sistema com realimentação de estados que funciona com o mesmo K (xSEN ou xlqr com KSEN ou Klqr acima)
% hold on
% plot(t,(180/pi*xdes(:,3)), "r")
% grid on
% grid minor
% legend('Controlador com realimentação de estados', 'Controlador com observador de estados')
% title('Desempenho do controle em \theta')
% xlabel('Tempo (s)')
% ylabel('Ângulo (°)')
% 
% clf(figure(6))
% figure(6)
% plot(t,(xSEN(:,2)), "b") %Por aqui o sistema com realimentação de estados que funciona com o mesmo K (xSEN ou xlqr com KSEN ou Klqr acima)
% hold on
% plot(t,(xdes(:,2)), "r")
% grid on
% grid minor
% legend('Controlador com realimentação de estados', 'Controlador com observador de estados')
% title('Desempenho do controle em q_2')
% xlabel('Tempo (s)')
% ylabel('Ângulo (°)')
% 
% clf(figure(7))
% figure(7)
% plot(t,xdes(:,2), "b") 
% hold on
% plot(t,(xest(:,2)), "r")
% grid on
% grid minor
% legend('q_2 real', 'q_2 estimado')
% title('Desempenho da estimação em q_2')
% xlabel('Tempo (s)')
% ylabel('Ângulo (°)')
% 
% clf(figure(8))
% figure(8)
% plot(t,xdes(:,6), "b") 
% hold on
% plot(t,(xest(:,6)), "r")
% grid on
% grid minor
% legend('\theta ponto real', '\theta ponto estimado')
% title('Desempenho da estimação em \theta ponto')
% xlabel('Tempo (s)')
% ylabel('Velocidade angular (rad/s)')
% 
% 
%% Seguidor de referências LQ
% 
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
% 
% % Referência 2 -> Degraus
%     Q_ref2 = [0.014 0 0 0 0 0;
%              0 15 0 0 0 0;
%              0 0 60.987 0 0 0; 
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
%                      0.5*theta_0*ones(1, 50) zeros(1, 25)  0.5*theta_0*ones(1, 50) zeros(1, 26),
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
%     sys_ref2 = ss(A_c_lq,-B*inv(R_ref2)*B',-K_lq,inv(R_ref2)*B');
%     
%     [ur2,t,xr2] = lsim(sys_ref2,flip(eta,2),t, x0);
% % 
%     %Referência 3 -> Rampa em theta chegando a zero em 5 segundos
%     Q_ref3 = [0.014 0 0 0 0 0;
%              0 5 0 0 0 0;
%              0 0 500.987 0 0 0; 
%              0 0 0 0.014 0 0; 
%              0 0 0 0 8 0; 
%              0 0 0 0 0 3.8];
%     
%     R_ref3 = [2*10^-6 0;
%              0 10*10^-12];
% 
%     x_ref3 = [zeros(1,length(t)),
%              zeros(1,length(t)),
%              theta_0*(1-t/5),
%              zeros(1,length(t)),
%              zeros(1,length(t)),
%              zeros(1,length(t))];
% 
%     [K_lq,p_lq,poles_lq] = lqr(sys_lqr,Q_ref3,R_ref3);
%     
%     A_c_lq = (A-B*K_lq);
%     eta_ss3 = ss(A_c_lq,Q_ref3,eye(6),zeros(6,6));
%     
%     eta03 = p_lq*x_ref3(:,end);
%     
%     eta = lsim(eta_ss3,x_ref3,t,eta03);
%     
%     sys_ref3 = ss(A_c_lq,B*inv(R_ref3)*B',-K_lq,inv(R_ref3)*B');
%     
%     [ur3,t,xr3] = lsim(sys_ref3,flip(eta,2),t, x0);
% 
% 
% clf(figure(5))
% figure(5)
% plot(t, 180/pi*xr3(:,3), "b") 
% hold on
% plot(t, 180/pi*x_ref3(3,:), "r")
% grid on
% grid minor
% legend('\theta do sistema', 'Referência')
% title('Seguidor LQ para \theta com degraus em 15 segundos')
% xlabel('Tempo (s)')
% ylabel('Ângulo (°)')
% set(gca,'FontSize',18)
% 
% clf(figure(6))
% figure(6)
% plot(t,xr3(:,2), "b") 
% hold on
% plot(t,x_ref3(2,:), "r")
% grid on
% grid minor
% legend('q_2 do sistema', 'Referência')
% title('Seguidor LQ para q_2')
% xlabel('Tempo (s)')
% ylabel('Posição (m)')
% set(gca,'FontSize',18)
% 
% clf(figure(7))
% figure(7)
% plot(t,ur3(:,2), "b") 
% hold on
% plot(t,-4500000*ones(length(t)), "r")
% hold on
% plot(t,4500000*ones(length(t)), "r")
% grid on
% grid minor
% legend('Torque de atuação', 'Limites do atuador')
% title('Esforços de torque no seguidor LQ')
% xlabel('Tempo (s)')
% ylabel('Torque (Nm)')
% set(gca,'FontSize',18)
% 
% clf(figure(8))
% figure(8)
% plot(t,ur3(:,1)/1000, "b") 
% hold on
% plot(t,-60*ones(length(t)), "r")
% hold on
% plot(t,60*ones(length(t)), "r")
% grid on
% grid minor
% legend('Força de atuação', 'Limites do atuador')
% title('Esforços de força no seguidor LQ')
% xlabel('Tempo (s)')
% ylabel('Força (KN)')
% set(gca,'FontSize',18)

%% Seguidor de referências variáveis exógenas

% K = KSEN;
% 
%     Q = [0.014 0 0 0 0 0;
%              0 5 0 0 0 0;
%              0 0 20 0 0 0; 
%              0 0 0 0.014 0 0; 
%              0 0 0 0 10 0; 
%              0 0 0 0 0 40];
%     
%     
%     R = [1*10^-8 0;
%              0 1*10^-9];
% 
%     
% sys_lqr = ss(A,B,C,Dlqr);
% [KLQR,S,Plqr] = lqr(sys_lqr,Q,R);
% K = KLQR;
% 
% %Rampa ou degrau
% A_r = [0 0 0 0 0 0;
%        0 0 0 0 1 0;
%        0 0 0 0 0 1;
%        0 0 0 0 0 0;
%        0 0 0 0 0 0;
%        0 0 0 0 0 0];
% % Seno
% omega = 0.5;
% omega2 = 8;
% zeta = 0.4;
% ac = 0.001;
% % 
% % 
% % A_r = [0 0 0 0 0 0;
% %       0 0 0 0 1 0;
% %       0 0 0 0 0 1;
% %       0 0 0 0 0 0;
% %       0 0 0 0 0 0;
% %       %0 -omega2^2 0 0 -2*zeta*omega2 0;
% %       0 0 -omega^2 0 0 0];
% 
% M = [0 1 0 0 0 0;
%      0 0 1 0 0 0];
% % 
% % 
% A_cl_inv = inv(A-B*K);
% N = inv(M*A_cl_inv*B)*M*A_cl_inv;
% G_r = N*(A-A_r);
% A_ex_cl = [[A-B*G_r,B*(K-G_r)];[(A_r - A +B*G_r),(A_r - B*(K-G_r))]];
% E_ex_cl = [-B*N*E + E ; B*N*E-E];
% K_ex = [-G_r,K-G_r];
% 
% 
% seg_exo_ss = ss(A_ex_cl,E_ex_cl,K_ex,0);
% 
% x0_seg = [x0 0 0 0 -q1p_0 -q2p_0 -theta_0/5]; %RAMPA
% 
% % x0_seg = [x0 -q1_0 -q2_0 -theta_0 -q1p_0 -q2p_0 -thetap_0]; %Degrau
%  %x0_seg = [x0 -q1_0 0 0 -q1p_0 -q2p_0 -thetap_0]; %Seno theta
% % x0_seg = [x0 -q1_0 0 0 -q1p_0 0 0]; %Seno theta e q2
% 
% [uex,t_exo,xex] = lsim(seg_exo_ss,w,t,x0_seg);
% 
% u = K*(xex(:,7:12))' - (N*(A-A_r))*((xex(:,1:6))'+(xex(:,7:12))') -N*E*(w);
%
% % Degrau constante
% % % x_ref_ext = zeros(length(t));
% % % x_ref_exq = zeros(length(t));
% 
% %Rampa
% x_ref_ext = theta_0*(1-t/5);
% x_ref_exq = zeros(length(t));
% % 
% % 
% clf(figure(9))
% figure(9)
% plot(t,180/pi*xex(:,3), "b")
% hold on
% plot(t,180/pi*x_ref_ext, "r")
% grid on
% grid minor
% legend('\theta do sistema', 'Referência')
% title('Seguidor de var.exógenas para \theta')
% xlabel('Tempo (s)')
% ylabel('Ângulo (°)')
% set(gca,'FontSize',18)
%
% clf(figure(10))
% figure(10)
% plot(t,xex(:,2), "b") 
% hold on
% plot(t,x_ref_exq, "r")
% % hold on
% % plot(t, xnc(:,2), 'g')
% grid on
% grid minor
% legend('q_2 do sistema', 'Referência')
% title('Seguidor de var.exógenas q_2')
% xlabel('Tempo (s)')
% ylabel('Posição (m)')
% set(gca,'FontSize',18)
%
% clf(figure(11))
% figure(11)
% plot(t,u(2,:), "b") 
% hold on
% plot(t,-4500000*ones(length(t)), "r")
% hold on
% plot(t, 4500000*ones(length(t)), "r")
% grid on
% grid minor
% legend('Torque de atuação', 'Limites do atuador')
% title('Esforços de torque no seguidor de var.exógenas')
% xlabel('Tempo (s)')
% ylabel('Torque (Nm)')
% set(gca,'FontSize',18)
% 
% clf(figure(12))
% figure(12)
% plot(t,u(1,:)/1000, "b") 
% hold on
% plot(t,-60*ones(length(t)), "r")
% hold on
% plot(t, 60*ones(length(t)), "r")
% grid on
% grid minor
% legend('Força de atuação', 'Limites do atuador')
% title('Esforços de força no seguidor de var.exógenas')
% xlabel('Tempo (s)')
% ylabel('Força (KN)')
% set(gca,'FontSize',18)
%% Simulacao realista
tpos = 0:T_sim:10;
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

sinal = xSEN;
r = sinal(1,3);
j = 1;
while r > 0
    j = j+1;
    r = sinal(j,3);
end

sys_pos = ss(A2,E2,zeros(8,8), zeros(8,2));
x0pos = [sinal(j,1) sinal(j,2) sinal(j,3) 0 sinal(j,4) sinal(j,5) sinal(j,6) -sinal(j,6)*18.2];

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
sysnc = ss(A, E, C, zeros(2,3));
[ync,t,xnc] = lsim(sysnc,w,t,x0);

sinal = xnc;
r = sinal(1,3);
k = 1;
while r > 0
    k = k+1;
    r = sinal(k,3);
end
x0pos = [sinal(k,1) sinal(k,2) sinal(k,3) 0 sinal(k,4) sinal(k,5) sinal(k,6) -sinal(k,6)*18.2];
x3_totnc = [sinal(1:k, 3); xpos(:,3)];


if length(x3_tot) > length(x3_totnc)
    x3_tot = x3_tot(1:length(x3_totnc));
else
    x3_totnc = x3_totnc(1:length(x3_tot));
end

TT2 = length(tpos) + k;
Tpos2 = T_sim.*(1:TT2);

temp = 0:T_sim:length(x3_tot)*T_sim;
temp = temp(1:length(temp)-1);
% 
% 
clf(figure(10))
figure(10)
plot(temp,180/pi*x3_tot, "b") 
hold on
plot(temp,180/pi*x3_totnc(1:length(x3_tot)), "r")
legend("Sistema controlado", "Sistema não controlado")
grid on
grid minor
title('Comparação dos sistemas com vento para \theta vento')
xlabel('Tempo (s)')
ylabel('Ângulo (°)')

% clf(figure(10))
% figure(10)
% plot(temp,x3_tot, "b") 
% hold on
% plot(temp,x3_totnc(1:length(x3_tot)), "r")
% legend("Sistema controlado", "Sistema não controlado")
% grid on
% grid minor
% title('Comparação dos sistemas com vento para  vento')
% xlabel('Tempo (s)')
% ylabel('Posição (m)')


