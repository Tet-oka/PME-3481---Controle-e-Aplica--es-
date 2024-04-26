% Verificar qual constante está sendo usada nas simulações, a nova para
% controle ou a mesma de modelagem.
%% Definição dos parâmetros iniciais da integração e cálculo das ODES
t = 6; 
T_sim = 1/100;
tempo = 0:T_sim:t;
%Passo máximo ODE
max_step = odeset('MaxStep', T_sim);

phi = 13*pi/180; 
g = 9.81;
rho = 1.2923; 
M = 88000; 
WL = 285; %Wing loading para Concorde pousando com 110 ton com A = (25.6^2/1.7)
S = M/WL; 
C_pav = 1; 
m = 4000;
Dpo = 5;
Dgo = 3.5;
D_fo = 18.2;
D_co = 29.2;
Joz = 16864415*M/88000;
kr = 13600000;
cr = 970000;
kt = 11486800;              %kt = 954910;
ct = 102196;         %ct = 103870; 
m_f = 1000; 
k_tf = 5743400;
c_tf = 51098;
k_rf = 3400000;
c_rf = 2*2425;
uv = 0.7;


%q1 e q2 definidos a partir da posição de equilíbrio estático do sistema em
%repouso

%Condições iniciais de integração:
q1_0 = (389300)/kr; %q1 de equilíbrio dinâmico a 80m/s
q2_0 = (389300)/kt + (389300)/kr; %q2 de equilíbrio dinâmico a 80m/s
theta_0 = 13*pi/180;
q3_0 = 0;
q1p_0 = -3; 
q2p_0 = -3; %Condição de velocidade inicial [m/s];
thetap_0 = 0;
q3p_0 = 0;
pos_ini = 0;
vel_ini = 300/3.6;

x0o =  [q1_0 q2_0 0 q3_0 q1p_0 q2p_0 thetap_0 q3p_0 pos_ini vel_ini];

x0c = [q1_0 q2_0 theta_0 q1p_0 q2p_0 thetap_0];

x0 =  [q1_0 q2_0 theta_0 q3_0 q1p_0 q2p_0 thetap_0 q3p_0 pos_ini vel_ini];


  [t, y] = ode45(@f, tempo, x0o, max_step);
  [t, ycn] = ode45(@fcn, tempo, x0, max_step);
% [t, yL] = ode45(@fL, tempo, x0o, max_step);
  [t, yLT] = ode45(@fLT, tempo, x0, max_step);
  [t, yAP] = ode45(@fAP,tempo, x0, max_step);
 %[t, yLLT] = ode45(@fLLT, tempo, x0, max_step);


%% Condições para a troca da ODE entre modelo incial e meio carro

% h=1;
% while y(h,9)~=y(h+1,9)
%     h=h+1;
% end
% 
% j=1;
% while yL(j,9)~=yL(j+1,9)
%     j=j+1;
% end
% 
% y0=y(h-1,:);
% yL0=yL(j-1,:);
% 
% y=y(1:h-1,:);
% yL=yL(1:j-1,:);
% 
% tempofr = tempo(1:(h-1));
% tempofrL = tempo(1:(j-1));
% tempot = tempo(h:end);
% tempotL = tempo(j:end);
% 
% yL0(3)=yL0(3)+13*pi/180;
% yL0(8)=yL0(7)*D_fo*cos(yL0(3));
% 
% xx0=y0;
% xxL0=yL0;
% 
% [tt, yy] = ode45(@ff, tempot, xx0, max_step);
% [tt, yyL] = ode45(@ffL, tempotL, xxL0, max_step);
% 
% 
% for i = 1:length(yyL)
%     yyL(i,3)=yyL(i,3)-13*pi/180;
% end
% 
% yc = [y;yy];
% ycL = [yL;yyL];
% 
% min1 = (min(yc(:,1)));
% max1 = (max(yc(:,1)));
% min2 = (min(yc(:,2)));
% max2 = (max(yc(:,2)));
% min3 = (min(yc(:,3)));
% max3 = (max(yc(:,3)));
% min4 = (min(yc(:,4)));
% max4 = (max(yc(:,4)));
% min5 = (min(yc(:,5)));
% max5 = (max(yc(:,5)));
% min6 = (min(yc(:,6)));
% max6 = (max(yc(:,6)));
% min7 = (min(yc(:,7)));
% max7 = (max(yc(:,7)));
% min8 = (min(yc(:,8)));
% max8 = (max(yc(:,8)));
% min9 = (min(yc(:,9)));
% max9 = (max(yc(:,9)));
% min10 = (min(yc(:,10)));
% max10 = (max(yc(:,10)));








%% Cálculo das acelerações


% dq2 = diff(yc(:,6))./diff(tempo); 
% dq2L = diff(ycL(:,6))./diff(tempo);
% 
% dtheta = diff(yc(:,7))./diff(tempo);
% dthetaL = diff(ycL(:,7))./diff(tempo);
% 
% G_2ponto = dq2 + (Dgo.*dtheta.*cos(yc(2:end,3)));
% 
% Cockpit_2ponto = dq2 + (D_co.*dtheta.*cos(yc(2:end,3)));
% Cockpit_2pontoL = dq2L + (D_co.*dthetaL.*cos(ycL(2:end,3)));
% rmscockpit = rms(Cockpit_2ponto(1,:).')
% 
% CL = (-0.00165*((180*(y(:,3)+ phi)/pi).^2)) + (0.07378*180*(y(:,3)+ phi)/pi) + 0.21999;
% CD = (0.00017*((180*(y(:,3)+ phi)/pi).^2)) + (0.01111*180*(y(:,3)+ phi)/pi) + 0.15714;
% L = CL.*0.5.*S.*rho.*(y(:,10)).^2;
% 
% 
% 
% figure(20)
% plot(tempo, CL, "b")
% title("CL")
% xlabel('Tempo (s)')
% 
% figure(21)
% plot(tempo, L, "b")
% title("L")
% xlabel('Tempo (s)')
% 
% figure(22)
% plot(tempo, CD, "b")
% title("CD")
% xlabel('Tempo (s)')
% a_q1 = -(((cr + ct).*y(:,4))/m) + (ct.*y(:,5))/m + (-(g*m) + kt.*(-y(:,1) + y(:,2)) + kr.*(-y(:,1) + y_ext + cr*yponto_ext))/m;
% a_q2 = -0.5*(-2*g*M*Joz + S*rho*CL*Joz.*((y(:,8) + uv).*(y(:,8) + uv)) + 2*Joz*kt.*(y(:,1) - y(:,2)) + M*cos(phi)*Dgo*(Dgo*(-((2*g*M - S*rho*CL.*((y(:,8) + uv).*(y(:,8) + uv)))*urol*(sin(phi) + cos(phi).*y(:,3))) + 2*g*M*(cos(phi) - sin(phi).*y(:,3))) - S*rho*Dpo.*((y(:,8) + uv).*(y(:,8) + uv))*(cos(phi)*(CL + CD.*y(:,3)) + sin(phi)*(CD - CL.*y(:,3)))))/(M*(M*(cos(phi)*cos(phi))*(Dgo*Dgo) - Joz)) + (ct*Joz.*y(:,4))/(M*(-(M*(cos(phi)*cos(phi))*(Dgo*Dgo)) + Joz)) + (ct*Joz.*y(:,5))/(M*M*(cos(phi)*cos(phi))*(Dgo*Dgo) - M*Joz);
%% Cálculo dos esforços dos atuadores e relacionados
%Para fazer alocação de polos, Trocar Klqr para K
% K11 = Klqr(1,1);
% K12 = Klqr(1,2);
% K13 = Klqr(1,3);
% K14 = Klqr(1,4);
% K15 = Klqr(1,5);
% K16 = Klqr(1,6);
% 
% K21 = Klqr(2,1);
% K22 = Klqr(2,2);
% K23 = Klqr(2,3);
% K24 = Klqr(2,4);
% K25 = Klqr(2,5);
% K26 = Klqr(2,6);

K11 =        -284.3    ; 
    K12 =         -5160     ; 
    K13 =    -7.4562e+03     ; 
    K14 =         3.6209     ; 
    K15 =         -9090     ; 
    K16 =    -7.6314e+03     ; 
    K21 =          52353     ; 
    K22 =     2.6127e+03     ; 
    K23 =     5.0393e+06      ; 
    K24 =         -147.6     ; 
    K25 =      5.262e+03     ; 
    K26 =     5.1107e+07;
Fa = -1*(K11*ycn(:,1)+K12*ycn(:,2)+K13*ycn(:,3)+K14*ycn(:,5)+K15*ycn(:,6)+K16*ycn(:,7));
Ta = -1*(K21*ycn(:,1)+K22*ycn(:,2)+K23*ycn(:,3)+K24*ycn(:,5)+K25*ycn(:,6)+K26*ycn(:,7));

AE = 32;
DT = 24.7;
Tmax = 1.3*AE.*0.5.*(y(:,10)+uv).^2.*rho.*DT.*cos(y(:,3)) + 0.25.*AE.*0.5.*(y(:,10)+uv).^2.*rho.*DT.*sin(y(:,3));

% 
figure(101)
plot(tempo, Fa/1000, "b")
grid on
grid minor
legend('Força ativa')
title('Força no atuador do trem de pouso')
xlabel('Tempo (s)')
ylabel('Força (KN)')

figure(102)
plot(tempo, (Ta), "b")
hold on
plot(tempo, (Tmax), "r")
hold on
plot(tempo, -(Tmax), "r")
grid on
grid minor
legend('Torque necessário do elevador', "Torque máximo fornecido", "-Torque máximo fornecido")
title('Controle de torque')
xlabel('Tempo (s)')
ylabel('Torque (N.m)')
% 

%% Plot dos gráficos
 
% figure(1)
% plot(tempo, yc(:,1), "b")
% hold on
% plot(tempo, ycL(:,1), "r")
% grid on
% grid minor
% ylim([11/10*min1 11/10*max1]);
% p = patch([0 0 T_sim*length(tempofr) T_sim*length(tempofr)],[11/10*min1 11/10*max1 11/10*max1 11/10*min1],'');
% set(p,'FaceAlpha',0.1)
% set(p,'EdgeColor','none')
% legend("Não-linear", "Linear", "Free-roll")
% title('Variação de q1')
% xlabel('Tempo (s)')
% ylabel('Posição (m)')
% 
figure(2)
plot(tempo, (y(:,2)), "r")
hold on
% plot(tempo, (yLT(:,2)), "c")
% hold on
plot(tempo, (yAP(:,2)), "b")
hold on
plot(tempo, (ycn(:,2)), "g")
% grid on
grid minor
%ylim([11/10*min2 11/10*max2]);
%p = patch([0 0 T_sim*length(tempofr) T_sim*length(tempofr)],[11/10*min2 11/10*max2 11/10*max2 11/10*min2],'');
%set(p,'FaceAlpha',0.1)
%set(p,'EdgeColor','none')
legend('Original (não linear)','Controlado Linear', 'Controlado Não Linear')
title('Variação de q2')
xlabel('Tempo (s)')
ylabel('Posição (m)')
% 
figure(3)
plot(tempo, (180/pi*y(:,3))+13, "r")
hold on
% plot(tempo, (180/pi*yLT(:,3)), "c")
%hold on
plot(tempo, (180/pi*ycn(:,3)), "b")
hold on
plot(tempo, (180/pi*ycn(:,3)), "g")
grid on
grid minor
%ylim([11/10*min2 11/10*max2]);
%p = patch([0 0 T_sim*length(tempofr) T_sim*length(tempofr)],[11/10*min2 11/10*max2 11/10*max2 11/10*min2],'');
%set(p,'FaceAlpha',0.1)
%set(p,'EdgeColor','none')
legend('Original (não linear)','Controlado Linear', 'Controlado Não Linear')
title('Variação do ângulo de arfagem')
xlabel('Tempo (s)')
ylabel('Ângulo (°)')

% Fs = 1/T_sim;            % Sampling frequency                    
% T = 1/Fs;             % Sampling period       
% L = length(ycL(:,3)); % Length of signal
% t = (0:L-1)*T;        % Time vector
% 
% YT = fft(yc(:,3));
% YQ = fft(yc(:,2));

% figure(400)
% plot(Fs/L*(0:L-1), abs(YT), "b")
% hold on
% plot(Fs/L*(0:L-1), abs(YQ), "r")
% grid on 
% grid minor
% legend("Theta", "q2")
% title('Transformada de Fourier')
% xlabel("f (Hz)")
% ylabel("abs(fft(X))")

% figure(4)
% plot(tempo, 180*yc(:,3)/pi, "b")
% hold on
% plot(tempo, 180*ycL(:,3)/pi, "r")
% grid on
% grid minor
% ylim([11/10*min2*180/pi 11/10*max3*180/pi]);
% p = patch([0 0 T_sim*length(tempofr) T_sim*length(tempofr)],[11/10*min3*180/pi 11/10*max3*180/pi 11/10*max3*180/pi 11/10*min3*180/pi],'');
% set(p,'FaceAlpha',0.1)
% set(p,'EdgeColor','none')
% legend("Não-linear", "Linear", "Free-roll")
% title('Variação de theta em graus para Phi=13')
% xlabel('Tempo (s)')
% ylabel('Ângulo (graus)')
% 
% figure(5)
% plot(tempo, yc(:,5), "b")
% hold on
% plot(tempo, ycL(:,5), "r")
% grid on
% grid minor
% ylim([11/10*min5 11/10*max5]);
% p = patch([0 0 T_sim*length(tempofr) T_sim*length(tempofr)],[11/10*min5 11/10*max5 11/10*max5 11/10*min5],'');
% set(p,'FaceAlpha',0.1)
% set(p,'EdgeColor','none')
% legend("Não-linear", "Linear", "Free-roll")
% title('Velocidade de q1')
% xlabel('Tempo (s)')
% ylabel('Velocidade (m/s)')
% 
% figure(6)
% plot(tempo, yc(:,6), "b")
% hold on
% plot(tempo, ycL(:,6), "r")
% grid on
% grid minor
% ylim([11/10*min6 11/10*max6]);
% p = patch([0 0 T_sim*length(tempofr) T_sim*length(tempofr)],[11/10*min6 11/10*max6 11/10*max6 11/10*min6],'');
% set(p,'FaceAlpha',0.1)
% set(p,'EdgeColor','none')
% legend("Não-linear", "Linear", "Free-roll")
% title('Velocidade de q2')
% xlabel('Tempo (s)')
% ylabel('Velocidade (m/s)')
% 
% figure(7)
% plot(tempo, y(:,7), "b")
% hold on
% plot(tempo, ycn(:,7), "r")
% grid on
% grid minor
% ylim([11/10*min7 11/10*max7]);
% p = patch([0 0 T_sim*length(tempofr) T_sim*length(tempofr)],[11/10*min7 11/10*max7 11/10*max7 11/10*min7],'');
% set(p,'FaceAlpha',0.1)
% set(p,'EdgeColor','none')
% legend("Não-linear", "Linear", "Free-roll")
% title('Velocidade angular de theta')
% xlabel('Tempo (s)')
% ylabel('Velocidade (rad/s)')

% dy=diff(y(:,7))./diff(tempo);
% figure(78)
% plot(tempo(2:end),Joz.*dy)
% grid on
% grid minor
% % ylim([11/10*min7 11/10*max7]);
% % p = patch([0 0 T_sim*length(tempofr) T_sim*length(tempofr)],[11/10*min7 11/10*max7 11/10*max7 11/10*min7],'');
% % set(p,'FaceAlpha',0.1)
% % set(p,'EdgeColor','none')
% legend("Não-linear")
% title('Torque do movimento natural')
% xlabel('Tempo (s)')
% ylabel('Torque resultante externo sem forças (N.m)')


% 
% figure(8)
% plot(tempo, yc(:,9), "b")
% hold on
% plot(tempo, ycL(:,9), "r")
% grid on
% grid minor
% ylim([11/10*min9 11/10*max9]);
% p = patch([0 0 T_sim*length(tempofr) T_sim*length(tempofr)],[11/10*min9 11/10*max9 11/10*max9 11/10*min9],'');
% set(p,'FaceAlpha',0.1)
% set(p,'EdgeColor','none')
% legend("Não-linear", "Linear", "Free-roll")
% title("Deslocamento longitudinal")
% xlabel('Tempo (s)')
% ylabel('Posição (m)')
% 
% figure(9)
% plot(tempo, yc(:,10), "b")
% hold on
% plot(tempo, ycL(:,10), "r")
% grid on
% grid minor
% set(p,'FaceAlpha',0.1)
% set(p,'EdgeColor','none')
% legend("Não-linear", "Linear", "Free-roll")
% title("Velocidade longitudinal")
% xlabel('Tempo (s)')
% ylabel('Velocidade (m/s)')
% 
% figure(10)
% plot(tempo, (180/pi)*abs(yc(:,3)-ycL(:,3)), "b")
% grid on
% grid minor
% title("Diferença entre modelo linear e não linear para theta")
% p = patch([0 0 T_sim*length(tempofr) T_sim*length(tempofr)],[-0 0.5 0.5 0],'');
% set(p,'FaceAlpha',0.1)
% set(p,'EdgeColor','none')
% legend("", "Free-roll")
% xlabel('Tempo (s)')
% ylabel('Diferença (graus)')
% 
% figure(11)
% plot(tempo, abs(yc(:,2)-ycL(:,2)), "b")
% hold on
% plot(tempo, abs(yc(:,1)-ycL(:,1)), "b")
% grid on
% grid minor
% legend("q2", "q1")
% title("Diferença entre modelo linear e não linear para q1")
% xlabel('Tempo (s)')
% ylabel('Diferença (m)')
% 
% figure(12)
% plot(tempo, yc(:,1), "r")
% hold on
% plot(tempo, yc(:,2), "g")
% xlabel('Tempo (s)')
% ylabel('Posição (m)')
% legend('Variação de q1','Variação de q2')
% 
% figure(13)
% plot(tempo, yc(:,4), "b")
% hold on
% plot(tempo, ycL(:,4), "r")
% grid on
% grid minor
% legend("Não-linear", "Linear")
% title('Variação de q3')
% xlabel('Tempo (s)')
% ylabel('Posição (m)')
% 
% figure(14)
% plot(tempo, yc(:,8), "b")
% hold on
% plot(tempo, ycL(:,8), "r")
% grid on
% grid minor
% legend("Não-linear", "Linear")
% title('Variação de q3ponto')
% xlabel('Tempo (s)')
% ylabel('Velcidade (m/s)')
% 
% figure(15)
% plot(tempo(2:end), (dq2(:,1)./g), "b")
% grid on
% grid minor
% hold on
% plot(tempo, ycL(:,8), "r")
% legend("Não-linear", "Linear")
% p = patch([0 0 T_sim*length(tempofr) T_sim*length(tempofr)],[-3 3 3 -3],'');
% set(p,'FaceAlpha',0.1)
% set(p,'EdgeColor','none')
% title('Carga inercial em q2',['C_{pav}=',int2str(C_pav)])
% legend("", "Free-roll")
% xlabel('Tempo (s)')
% ylabel('Aceleração (m/s^2)')


%% Plot dos gráficos antes do toque

% figure(101)
% plot(tempofr, y(:,1), "b")
% hold on
% plot(tempofrL, yL(:,1), "r")
% grid on
% grid minor
% legend("Não-linear", "Linear")
% title('Variação de q1 durante o free-roll')
% xlabel('Tempo (s)')
% ylabel('Posição (m)')
% 
% figure(102)
% plot(tempofr, y(:,2), "b")
% hold on
% plot(tempofrL, yL(:,2), "r")
% grid on
% grid minor
% legend("Não-linear", "Linear")
% title('Variação de q2 durante o free-roll')
% xlabel('Tempo (s)')
% ylabel('Posição (m)')
% 
% figure(103)
% plot(tempofr, y(:,3), "b")
% hold on
% plot(tempofrL, yL(:,3), "r")
% grid on
% grid minor
% legend("Não-linear", "Linear")
% title('Variação de theta em rad durante o free-roll')
% xlabel('Tempo (s)')
% ylabel('Ângulo (rad)')
% 
% figure(104)
% plot(tempofr, 180*y(:,3)/pi, "b")
% hold on
% plot(tempofrL, 180*yL(:,3)/pi, "r")
% grid on
% grid minor
% legend("Não-linear", "Linear")
% title('Variação de theta em graus para Phi=13 durante o free-roll')
% xlabel('Tempo (s)')
% ylabel('Ângulo (graus)')
% 
% figure(105)
% plot(tempofr, y(:,5), "b")
% hold on
% plot(tempofrL, yL(:,5), "r")
% grid on
% grid minor
% legend("Não-linear", "Linear")
% title('Velocidade de q1 durante o free-roll')
% xlabel('Tempo (s)')
% ylabel('Velocidade (m/s)')
% 
% figure(106)
% plot(tempofr, y(:,6), "b")
% hold on
% plot(tempofrL, yL(:,6), "r")
% grid on
% grid minor
% legend("Não-linear", "Linear")
% title('Velocidade de q2 durante o free-roll')
% xlabel('Tempo (s)')
% ylabel('Velocidade (m/s)')
% 
% figure(107)
% plot(tempofr, y(:,7), "b")
% hold on
% plot(tempofrL, yL(:,7), "r")
% grid on
% grid minor
% legend("Não-linear", "Linear")
% title('Velocidade angular de theta durante o free-roll')
% xlabel('Tempo (s)')
% ylabel('Velocidade (rad/s)')
% 
% figure(108)
% plot(tempofr, y(:,9), "b")
% hold on
% plot(tempofrL, yL(:,9), "r")
% grid on
% grid minor
% legend("Não-linear", "Linear")
% title("Deslocamento longitudinal durante o free-roll")
% xlabel('Tempo (s)')
% ylabel('Posição (m)')
% 
% figure(109)
% plot(tempofr, y(:,10), "b")
% hold on
% plot(tempofrL, yL(:,10), "r")
% grid on
% grid minor
% legend("Não-linear", "Linear")
% title("Velocidade longitudinal durante o free-roll")
% xlabel('Tempo (s)')
% ylabel('Velocidade (m/s)')
% 
% 
% figure(110)
% plot(tempofrL, (180/pi)*abs(y(1:(j-1),3)-yL(1:(j-1),3)), "b")
% grid on
% grid minor
% title("Diferença entre modelo linear e não linear para theta durante o free-roll")
% xlabel('Tempo (s)')
% ylabel('Diferença (graus)')
% 
% figure(111)
% plot(tempofrL, abs(y(1:(j-1),1)-yL(1:(j-1),1)), "m")
% hold on
% plot(tempofrL, abs(y(1:(j-1),2)-yL(1:(j-1),2)), "g")
% grid on
% grid minor
% legend("q1", "q2")
% title("Diferença entre modelo linear e não linear para q1 e q2 durante o free-roll")
% xlabel('Tempo (s)')
% ylabel('Diferença (m)')
% 
% figure(112)
% plot(tempofr, y(:,1), "m")
% hold on
% plot(tempofr, y(:,2), "g")
% title("Variação de q1 e q2 durante o free-roll")
% xlabel('Tempo (s)')
% ylabel('Posição (m) durante o free-roll')
% legend('Variação de q1','Variação de q2')
% 
% Não fazem nada durante o free-roll
% figure(113)
% plot(tempofr, y(:,4), "b")
% hold on
% plot(tempofrL, yL(:,4), "r")
% grid on
% grid minor
% legend("Não-linear", "Linear")
% title('Variação de q3 durante o free-roll')
% xlabel('Tempo (s)')
% ylabel('Posição (m)')
% 
% figure(114)
% plot(tempofr, y(:,8), "b")
% hold on
% plot(tempofrL, yL(:,8), "r")
% grid on
% grid minor
% legend("Não-linear", "Linear")
% title('Variação de q3ponto durante o free-roll')
% xlabel('Tempo (s)')
% ylabel('Velcidade (m/s)')
% 
% figure(115)
% plot(tempofr(2:end), (dq2(1:(h-2),1)./g), "b")
% hold on
% plot(tempofrL(2:end), (dq2L(1:(j-2),1)./g), "r")
% grid on
% grid minor
% title('Carga inercial em q2 durante o free-roll',['C_{pav}=',int2str(C_pav)])
% legend("Não-linear", "Linear")
% xlabel('Tempo (s)')
% ylabel('Aceleração (m/s^2)')
% 
% figure(201)
% plot(tempofr(2:end), (dq2(1:(h-2),1)./g),   "m")
% hold on
% grid on
% grid minor
% title('Carga inercial em q2 durante o free-roll')
% legend("ct=c_{t0}", "ct=c_{t0}*0.6", "ct=c_{t0}*1.4", "ct=c_{t0}*1.8")
% xlabel('Tempo (s)')
% ylabel('Carga inercial')
% 
% 
% figure(202)
% plot(tempofr(2:end), (Cockpit_2ponto(1:(h-2),1)),   "m")
% hold on
% grid on
% grid minor
% title('Aceleração vertical no cockpit durante o free-roll')
% legend("ct=c_{t0}", "ct=c_{t0}*0.6", "ct=c_{t0}*1.4", "ct=c_{t0}*1.8")
% xlabel('Tempo (s)')
% ylabel('Aceleração (m/s^2)')
% 
% figure(203)
% plot(tempofr(2:end), (dtheta(1:(h-2),1).*(180/pi)),   "m")
% hold on
% grid on
% grid minor
% title('Aceleração angular de arfagem do avião')
% legend("ct=c_{t0}", "ct=c_{t0}*0.6", "ct=c_{t0}*1.4", "ct=c_{t0}*1.8")
% xlabel('Tempo (s)')
% ylabel('Aceleração angular (graus/s^2)')
% 
% figure(204)
% plot(tempofr, (phi*180/pi)+180*y(:,3)/pi,   "m")
% hold on
% grid on
% grid minor
% legend("ct=c_{t0}", "ct=c_{t0}*0.6", "ct=c_{t0}*1.4", "ct=c_{t0}*1.8")
% title("Variação de theta em graus para Phi= "+ phi*180/pi +" durante o free-roll")
% xlabel('Tempo (s)')
% ylabel('Ângulo (graus)')
% 
% figure(205)
% plot(tempofr,  y(:,2), "r")
% hold on
% grid on
% grid minor
% legend("phi_0=13°", "phi_0=11°", "phi_0=9°", "phi_0=7°")
% title("Variação de q_2 durante o free-roll para diferentes ângulos de arfagem iniciais")
% xlabel('Tempo (s)')
% ylabel('Posição (m)')

% rho_oil = 912;
% zeta = 0.3;
% A = 0.09;
% Ao = 6.4*10^-3;
% yo = 0.9;
% n = 1.1;
% po = 40*10^6; 
% km = 0.7*104;
% kn = 0.1*10^5;
% 
% figure(206)
% plot(tempofr,  (y(:,6)-y(:,5)).*abs(y(:,6)-y(:,5))*(0.5*rho_oil*A^3/((zeta*Ao)^2)), "b")
% hold on
% plot(tempofr, (y(:,6)-y(:,5))*ct, "r")
% grid on
% grid minor
% legend("Força óleo (Fl)", "Força c linear")
% title("Comparação no tempo para Fl")
% xlabel('Tempo (s)')
% ylabel('Força (N)')
% 
% figure(207)
% plot((y(:,6)-y(:,5)),  (y(:,6)-y(:,5)).*abs(y(:,6)-y(:,5))*(0.5*rho_oil*A^3/((zeta*Ao)^2)), "b")
% hold on
% plot((y(:,6)-y(:,5)), (y(:,6)-y(:,5))*ct, "r")
% grid on
% grid minor
% legend("Força óleo (Fl)", "Força c linear")
% title("Comparação na velocidade para Fl")
% xlabel('Tempo (s)')
% ylabel('Força (N)')
% 
% 
% figure(208)
% plot(tempofr,  (y(:,6)-y(:,5)), "b")
% grid on
% grid minor
% legend("q2p-q1p")
% title("Vel no pistao")
% xlabel('Tempo (s)')
% ylabel('Vel (m/s)')
% 
% figure(209)
% plot(tempofr,  po*A*(1./((1-((y(:,2)-y(:,1))))./yo)).^n, "b")
% hold on
% plot(tempofr, (y(:,2)-y(:,1))*kt, "r")
% grid on
% grid minor
% legend("Força mola óleo (Fa)", "Força K linear")
% title("Comparação no tempo para Fa")
% xlabel('Tempo (s)')
% ylabel('Força (N)')
% 
% figure(210)
% plot((y(:,2)-y(:,1)),  po*A*(1./((1-((y(:,2)-y(:,1))))./yo)).^n, "b")
% hold on
% plot((y(:,2)-y(:,1)), (y(:,2)-y(:,1))*kt, "r")
% grid on
% grid minor
% legend("Força mola óleo (Fa)", "Força K linear")
% title("Comparação no deslocamento para Fa")
% xlabel('Pos (m)')
% ylabel('Força (N)')
% 
% kteste = 11.8*10^3;
% 
% figure(211)
% plot(tempofr, kn*sign(y(:,6)-y(:,5)).*(y(:,6)-y(:,5)).^2, "b")
% hold on
% plot(tempofr, km*(y(:,6)-y(:,5)), "r")
% hold on
% plot(tempofr, kteste*(y(:,6)-y(:,5)), "g")
% grid on
% grid minor
% legend("Força atrito com modulo (Fseal+Fow)", "Força atrito sem modulo", "Força de amortecedor com " + kteste + " Ns/m")
% title("Comparação no tempo para (Fseal+Fow)")
% xlabel('Tempo (s)')
% ylabel('Força (N)')
%% Funções ODE
%f -> simulacao nao linear
%ff -> simulacao nao linear após o toque do trem da frente (talvez nao seja
%utilizada em controle) 
%fL -> simulacao nao linear em CD e CL e linear no resto (usada
%equivocadamente em modelagem) linearizada em 13 graus
%ffL -> simulacao nao linear em CD e CL e linear no resto após o toque do trem da frente (talvez nao seja
%utilizada em controle) linearizada em 13 graus
%fLT -> simulacao nao linear em CD e CL e linear no resto linearizada em 0
%grau
%fLLT -> simulacao TOTALMENTE linear linearizada em 0 grau - A QUE DEVEMOS
%USAR
%fAP -> simulacao TOTALMENTE linear linearizada em 0 grau -e controlada com
%alocacao de polos

function dydt = f(t, y_0)
    phi = 13*pi/180; 
    g = 9.81;
    rho = 1.2923; 
    C_pav = 1; 
    M = 88000; 
    m = 4000;
    WL = 285;
    S = M/WL; 
   
%     CL = (0.04264*((180/pi)*(y_0(3) + phi)))-0.0158;
%     CD = (0.01394*((180/pi)*(y_0(3) + phi))) - 0.0248;
    CL = (0.04264*((180/pi)*(y_0(3) + phi)));
    CD = (0.01394*((180/pi)*(y_0(3) + phi)));
    urol = (0.0041+0.000041*y_0(10))*C_pav;
    Dpo = 5;
    Dgo = 3.5;
    D_fo = 18.2;
    uv = 0.7;  %estoura em 3.9
    y_ext = 0;
    yponto_ext = 0;
    Joz = 16864415*M/88000;
    kr = 13600000;
    cr = 9700;
    kt = 11486800;      %kt = 954910;
    ct = 102196;        %ct = 103870; 
    m_f = 1000; 
    k_tf = 5743400;
    c_tf = 51098;
    k_rf = 3400000;
    c_rf = 2*2425;
    
    %Os dados que mudam com o chaveamento devem ser inseridos dentro do if/else
    if -y_0(3) < phi
        dydt1 = y_0(5); %q1' SEM MG a partir de 0
        dydt2 = y_0(6); %q2'
        dydt3 = y_0(7); %theta'
        dydt4 = y_0(8); %q3'
        dydt5 = -(((cr + ct)*y_0(5))/m) + (ct*y_0(6))/m + (kt*(-y_0(1) + y_0(2)) + kr*(-y_0(1) + y_ext) + cr*yponto_ext)/m;
        dydt6 = -0.5*(S*rho*CL*Joz*((y_0(10) + uv)*(y_0(10) + uv)) + M*cos(phi + y_0(3))*Dgo*(-(S*rho*(sin(phi + y_0(3))*CD + cos(phi + y_0(3))*CL)*Dpo*((y_0(10) + uv)*(y_0(10) + uv))) + Dgo*(2*g*M*cos(phi + y_0(3)) + sin(phi + y_0(3))*(-2*g*M + S*rho*CL*((y_0(10) + uv)*(y_0(10) + uv)))*urol)) + 2*Joz*kt*(y_0(1) - y_0(2)))/(M*(M*(cos(phi + y_0(3))*cos(phi + y_0(3)))*(Dgo*Dgo) - Joz)) + (ct*Joz*y_0(5))/(M*(-(M*(cos(phi + y_0(3))*cos(phi + y_0(3)))*(Dgo*Dgo)) + Joz)) + (ct*Joz*y_0(6))/(M*M*(cos(phi + y_0(3))*cos(phi + y_0(3)))*(Dgo*Dgo) - M*Joz) - (sin(phi + y_0(3))*Dgo*Joz*(y_0(7)*y_0(7)))/(M*(cos(phi + y_0(3))*cos(phi + y_0(3)))*(Dgo*Dgo) - Joz);
        dydt7 = -((sec(phi + y_0(3))*(S*rho*Dpo*((y_0(10) + uv)*(y_0(10) + uv))*(CL + CD*tan(phi + y_0(3))) + Dgo*(-2*g*M + 2*g*M*urol*tan(phi + y_0(3)) - S*rho*CL*((y_0(10) + uv)*(y_0(10) + uv))*(1 + urol*tan(phi + y_0(3))) + 2*kt*(-y_0(1) + y_0(2)))))/(2*M*(Dgo*Dgo) - 2*(sec(phi + y_0(3))*sec(phi + y_0(3)))*Joz)) + (sec(phi + y_0(3))*ct*Dgo*y_0(5))/(M*(Dgo*Dgo) - sec(phi + y_0(3))*sec(phi + y_0(3))*Joz) + (sec(phi + y_0(3))*ct*Dgo*y_0(6))/(-(M*(Dgo*Dgo)) + sec(phi + y_0(3))*sec(phi + y_0(3))*Joz) + (M*(Dgo*Dgo)*tan(phi + y_0(3))*(y_0(7)*y_0(7)))/(M*(Dgo*Dgo) - sec(phi + y_0(3))*sec(phi + y_0(3))*Joz);
        dydt8 = 0;
        dydt9 = y_0(10);
        dydt10 = (-((M+m)*g - CL*S*rho*((y_0(10)+uv)^2)/2)*(urol) - CD*S*rho*((y_0(10)+uv)^2)/2)/(M+m);
        dydt = [dydt1; dydt2; dydt3; dydt4; dydt5; dydt6; dydt7; dydt8; dydt9; dydt10];
    else
        dydt1 = 0;
        dydt2 = 0;
        dydt3 = 0;
        dydt4 = 0;
        dydt5 = 0;
        dydt6 = 0;
        dydt7 = 0;
        dydt8 = 0;
        dydt9 = 0;
        dydt10 = 0;
        dydt = [dydt1; dydt2; dydt3; dydt4; dydt5; dydt6; dydt7; dydt8; dydt9; dydt10];
end
end
% 
function dyLdt = fL(t, yL_0)
phi = 13*pi/180; 
g = 9.81;
rho = 1.2923; 
C_pav = 1; 
M = 88000; 
m = 4000;
WL = 285;
S = M/WL; 

% CL = (0.04264*((180/pi)*(yL_0(3) + phi)))-0.0158;
% CD = (0.01394*((180/pi)*(yL_0(3) + phi))) - 0.0248;
CL = (0.04264*((180/pi)*(yL_0(3) + phi)));
CD = (0.01394*((180/pi)*(yL_0(3) + phi))) ;
urol = (0.0041+0.000041*yL_0(10))*C_pav;
Dpo = 5;
Dgo = 3.5;
D_fo = 18.2;
uv = 0.7;  %estoura em 3.9
y_ext = 0;
yponto_ext = 0;
Joz = 16864415*M/88000;
kr = 13600000;
cr = 9700;
kt = 11486800;      %kt = 954910;
ct = 102196;        %ct = 103870; 
m_f = 1000; 
k_tf = 5743400;
c_tf = 51098;
k_rf = 3400000;
c_rf = 2*2425;
vel_ini = 300/3.6;
% Os dados que mudam com o chaveamento devem ser inseridos dentro do if/else
if -yL_0(3) < phi
    dyLdt1 = yL_0(5);
    dyLdt2 = yL_0(6);
    dyLdt3 = yL_0(7);
    dyLdt4 = yL_0(8);
    dyLdt5 = -(((cr+ct)*yL_0(5))/m)+(ct*yL_0(6))/m+(kt*(-yL_0(1)+yL_0(2))+kr*(-yL_0(1)+y_ext)+cr*yponto_ext)/m;
    dyLdt6 = -0.5*(S*rho*CL*Joz*((yL_0(10)+uv)*(yL_0(10)+uv))+2*Joz*kt*(yL_0(1)-yL_0(2))+M*cos(phi)*Dgo*(Dgo*(-((2*g*M-S*rho*CL*((yL_0(10)+uv)*(yL_0(10)+uv)))*urol*(sin(phi)+cos(phi)*yL_0(3)))+2*g*M*(cos(phi)-sin(phi)*yL_0(3)))-S*rho*Dpo*((yL_0(10)+uv)*(yL_0(10)+uv))*(cos(phi)*(CL+CD*yL_0(3))+sin(phi)*(CD-CL*yL_0(3)))))/(M*(M*(cos(phi)*cos(phi))*(Dgo*Dgo)-Joz))+(ct*Joz*yL_0(5))/(M*(-(M*(cos(phi)*cos(phi))*(Dgo*Dgo))+Joz))+(ct*Joz*yL_0(6))/(M*M*(cos(phi)*cos(phi))*(Dgo*Dgo)-M*Joz);
    dyLdt7 = -((sec(phi)*(S*rho*Dpo*((yL_0(10)+uv)*(yL_0(10)+uv))*(CD*(tan(phi)+yL_0(3))+CL*(1-tan(phi)*yL_0(3)))+Dgo*(-(S*rho*CL*((yL_0(10)+uv)*(yL_0(10)+uv))*(1+urol*(tan(phi)+yL_0(3))))+2*(-(g*M)+kt*(-yL_0(1)+yL_0(2))+g*M*tan(phi)*yL_0(3)+g*M*urol*(tan(phi)+yL_0(3))))))/(2*M*(Dgo*Dgo)-2*(sec(phi)*sec(phi))*Joz))+(sec(phi)*ct*Dgo*yL_0(5))/(M*(Dgo*Dgo)-sec(phi)*sec(phi)*Joz)+(sec(phi)*ct*Dgo*yL_0(6))/(-(M*(Dgo*Dgo))+sec(phi)*sec(phi)*Joz);
    dyLdt8 = 0;
    dyLdt9 = yL_0(10);
    dyLdt10 = (-((M+m)*g - CL*S*rho*((yL_0(10)+uv)^2)/2)*(urol) - CD*S*rho*((yL_0(10)+uv)^2)/2)/(M+m);
    dyLdt = [dyLdt1; dyLdt2; dyLdt3; dyLdt4; dyLdt5; dyLdt6; dyLdt7; dyLdt8; dyLdt9; dyLdt10];
else
    dyLdt1 = 0;
    dyLdt2 = 0;
    dyLdt3 = 0;
    dyLdt4 = 0;
    dyLdt5 = 0;
    dyLdt6 = 0;
    dyLdt7 = 0;
    dyLdt8 = 0;
    dyLdt9 = 0;
    dyLdt10 = 0;
    dyLdt = [dyLdt1; dyLdt2; dyLdt3; dyLdt4; dyLdt5; dyLdt6; dyLdt7; dyLdt8; dyLdt9; dyLdt10];
end
end
%
function dydt = ff(t, y_0)
phi = 13*pi/180; 
g = 9.81;
rho = 1.2923; 
M = 88000; 
m = 4000;
WL = 285;
S = M/WL; 
C_pav = 1; 
CL = 0.0398*180*(y_0(3)+ phi)/pi -0.00633;
CL = (0.04264*((180/pi)*(y_0(3) + phi)))-0.0158;
CD = (0.01394*((180/pi)*(y_0(3) + phi))) - 0.0248;
urol = (0.0041+0.000041*y_0(10))*C_pav;
Dpo = 5;
Dgo = 3.5;
D_fo = 18.2;
uv = 0.7;  %estoura em 3.9 
y_ext = 0;
yponto_ext = 0;
Joz = 16864415*M/88000;
kr = 13600000;
cr = 9700;
kt = 11486800;      %kt = 954910;
ct = 102196;        %ct = 103870; 
m_f = 1000; 
k_tf = 5743400;
c_tf = 51098;
k_rf = 3400000;
c_rf = 2*2425;

    dydt1 = y_0(5);
    dydt2 = y_0(6);
    dydt3 = y_0(7);
    dydt4 = y_0(8);
    dydt5 = -(((cr + ct)*y_0(5))/m) + (ct*y_0(6))/m + (kt*(-y_0(1) + y_0(2)) + kr*(-y_0(1) + y_ext) + cr*yponto_ext)/m;
    dydt6 = (ct*Joz*y_0(5))/(M*(-(M*(cos(phi + y_0(3))*cos(phi + y_0(3)))*(Dgo*Dgo)) + Joz)) + ((-(M*(cos(phi + y_0(3))*cos(phi + y_0(3)))*c_tf*D_fo*Dgo) + (ct + c_tf)*Joz)*y_0(6))/(M*(M*(cos(phi + y_0(3))*cos(phi + y_0(3)))*(Dgo*Dgo) - Joz)) + (cos(phi + y_0(3))*c_tf*D_fo*(-(M*(cos(phi + y_0(3))*cos(phi + y_0(3)))*D_fo*Dgo) + Joz)*y_0(7))/(M*(M*(cos(phi + y_0(3))*cos(phi + y_0(3)))*(Dgo*Dgo) - Joz)) - (sin(phi + y_0(3))*Dgo*Joz*(y_0(7)*y_0(7)))/(M*(cos(phi + y_0(3))*cos(phi + y_0(3)))*(Dgo*Dgo) - Joz) - (M*cos(phi + y_0(3))*(Dgo*Dgo)*(2*g*M*cos(phi + y_0(3)) + sin(phi + y_0(3))*(-2*g*M + S*rho*CL*((y_0(10) + uv)*(y_0(10) + uv)))*urol) + Joz*(S*rho*CL*((y_0(10) + uv)*(y_0(10) + uv)) + 2*kt*(y_0(1) - y_0(2)) - 2*k_tf*(sin(phi + y_0(3))*D_fo + y_0(2) - y_0(4))) + M*cos(phi + y_0(3))*Dgo*(-(S*rho*(sin(phi + y_0(3))*CD + cos(phi + y_0(3))*CL)*Dpo*((y_0(10) + uv)*(y_0(10) + uv))) + 2*cos(phi + y_0(3))*D_fo*k_tf*(sin(phi + y_0(3))*D_fo + y_0(2) - y_0(4))))/(2.*M*(M*(cos(phi + y_0(3))*cos(phi + y_0(3)))*(Dgo*Dgo) - Joz)) + (c_tf*(M*(cos(phi + y_0(3))*cos(phi + y_0(3)))*D_fo*Dgo - Joz)*y_0(8))/(M*(M*(cos(phi + y_0(3))*cos(phi + y_0(3)))*(Dgo*Dgo) - Joz));
    dydt7 = (sec(phi + y_0(3))*ct*Dgo*y_0(5))/(M*(Dgo*Dgo) - sec(phi + y_0(3))*sec(phi + y_0(3))*Joz) + (cos(phi + y_0(3))*(c_tf*D_fo - (ct + c_tf)*Dgo)*y_0(6))/(M*(cos(phi + y_0(3))*cos(phi + y_0(3)))*(Dgo*Dgo) - Joz) + (c_tf*D_fo*(D_fo - Dgo)*y_0(7))/(M*(Dgo*Dgo) - sec(phi + y_0(3))*sec(phi + y_0(3))*Joz) + (M*(Dgo*Dgo)*tan(phi + y_0(3))*(y_0(7)*y_0(7)))/(M*(Dgo*Dgo) - sec(phi + y_0(3))*sec(phi + y_0(3))*Joz) - (sec(phi + y_0(3))*(S*rho*Dpo*((y_0(10) + uv)*(y_0(10) + uv))*(CL + CD*tan(phi + y_0(3))) + Dgo*(-2*g*M + 2*g*M*urol*tan(phi + y_0(3)) - S*rho*CL*((y_0(10) + uv)*(y_0(10) + uv))*(1 + urol*tan(phi + y_0(3))) + 2*kt*(-y_0(1) + y_0(2)) + 2*k_tf*(sin(phi + y_0(3))*D_fo + y_0(2) - y_0(4))) - 2*D_fo*k_tf*(sin(phi + y_0(3))*D_fo + y_0(2) - y_0(4))))/(2*M*(Dgo*Dgo) - 2*(sec(phi + y_0(3))*sec(phi + y_0(3)))*Joz) + (cos(phi + y_0(3))*c_tf*(D_fo - Dgo)*y_0(8))/(-(M*(cos(phi + y_0(3))*cos(phi + y_0(3)))*(Dgo*Dgo)) + Joz);
    dydt8 = (c_tf*y_0(6))/m_f + (cos(phi + y_0(3))*c_tf*D_fo*y_0(7))/m_f - ((c_rf + c_tf)*y_0(8))/m_f + (k_tf*(sin(phi + y_0(3))*D_fo + y_0(2) - y_0(4)) + k_rf*(-y_0(4) + y_ext) + c_rf*yponto_ext)/m_f;
    dydt9 = y_0(10);
    dydt10 = (-((M+m)*g - CL*S*rho*((y_0(10)+uv)^2)/2)*(urol) - CD*S*rho*((y_0(10)+uv)^2)/2)/(M+m);
    dydt = [dydt1; dydt2; dydt3; dydt4; dydt5; dydt6; dydt7; dydt8; dydt9; dydt10];

end
%
function dyLdt = ffL(t, yL_0)
phi = 13*pi/180; 
g = 9.81;
rho = 1.2923; 
M = 88000; 
m = 4000;
WL = 285;
S = M/WL; 
C_pav = 1; 
CL = 0.0398*180*(yL_0(3)+ phi)/pi -0.00633;
CL = (0.04264*((180/pi)*(yL_0(3) + phi)))-0.0158;
CD = 0.0088*180*(yL_0(3)+ phi)/pi -0.00767;
CD = (0.01394*((180/pi)*(yL_0(3) + phi))) - 0.0248;
urol = (0.0041+0.000041*yL_0(10))*C_pav;
Dpo = 5;
Dgo = 3.5;
D_fo = 18.2;
uv = 0.7;  %estoura em 3.9
y_ext = 0;
yponto_ext = 0;
Joz = 16864415*M/88000;
kr = 13600000;
cr = 9700;
kt = 11486800;      %kt = 954910;
ct = 102196;        %ct = 103870;  
m_f = 1000; 
k_tf = 5743400;
c_tf = 51098;
k_rf = 3400000;
c_rf = 2*2425;
vel_ini = 300/3.6;

    dyLdt1 = yL_0(5);
    dyLdt2 = yL_0(6);
    dyLdt3 = yL_0(7); 
    dyLdt4 = yL_0(8);
    dyLdt5 = -(((cr+ct)*yL_0(5))/m)+(ct*yL_0(6))/m+(kt*(-yL_0(1)+yL_0(2))+kr*(-yL_0(1)+y_ext)+cr*yponto_ext)/m;
    dyLdt6 = (ct*Joz*yL_0(5))/(M*(-(M*(Dgo*Dgo))+Joz))+((-(M*c_tf*D_fo*Dgo)+(ct+c_tf)*Joz)*yL_0(6))/(M*(M*(Dgo*Dgo)-Joz))+(c_tf*D_fo*(-(M*D_fo*Dgo)+Joz)*yL_0(7))/(M*(M*(Dgo*Dgo)-Joz))+(M*(Dgo*Dgo)*(-2*g*M+(2*g*M-S*rho*CL*((yL_0(10)+uv)*(yL_0(10)+uv)))*urol*yL_0(3))-Joz*(S*rho*CL*((yL_0(10)+uv)*(yL_0(10)+uv))-2*(kt*(-yL_0(1)+yL_0(2))+k_tf*(yL_0(2)+D_fo*yL_0(3)-yL_0(4))))+M*Dgo*(S*rho*Dpo*((yL_0(10)+uv)*(yL_0(10)+uv))*(CL+CD*yL_0(3))-2*D_fo*k_tf*(yL_0(2)+D_fo*yL_0(3)-yL_0(4))))/(2.*M*(M*(Dgo*Dgo)-Joz))+(c_tf*(M*D_fo*Dgo-Joz)*yL_0(8))/(M*(M*(Dgo*Dgo)-Joz));
    dyLdt7 = (ct*Dgo*yL_0(5))/(M*(Dgo*Dgo)-Joz)+((c_tf*D_fo-(ct+c_tf)*Dgo)*yL_0(6))/(M*(Dgo*Dgo)-Joz)+(c_tf*D_fo*(D_fo-Dgo)*yL_0(7))/(M*(Dgo*Dgo)-Joz)+(-(S*rho*Dpo*((yL_0(10)+uv)*(yL_0(10)+uv))*(CL+CD*yL_0(3)))+Dgo*(S*rho*CL*((yL_0(10)+uv)*(yL_0(10)+uv))*(1+urol*yL_0(3))-2*(-(g*M)+kt*(-yL_0(1)+yL_0(2))+D_fo*k_tf*yL_0(3)+g*M*urol*yL_0(3)+k_tf*(yL_0(2)-yL_0(4))))+2*D_fo*k_tf*(yL_0(2)+D_fo*yL_0(3)-yL_0(4)))/(2*M*(Dgo*Dgo)-2*Joz)+(c_tf*(D_fo-Dgo)*yL_0(8))/(-(M*(Dgo*Dgo))+Joz);
    dyLdt8 = (c_tf*yL_0(6))/m_f+(c_tf*D_fo*yL_0(7))/m_f-((c_rf+c_tf)*yL_0(8))/m_f+(k_tf*(yL_0(2)+D_fo*yL_0(3)-yL_0(4))+k_rf*(-yL_0(4)+y_ext)+c_rf*yponto_ext)/m_f;
    dyLdt9 = yL_0(10);
    dyLdt10 = (-((M+m)*g - CL*S*rho*((yL_0(10)+uv)^2)/2)*(urol) - CD*S*rho*((yL_0(10)+uv)^2)/2)/(M+m);
    dyLdt = [dyLdt1; dyLdt2; dyLdt3; dyLdt4; dyLdt5; dyLdt6; dyLdt7; dyLdt8; dyLdt9; dyLdt10];

end
%
function dyLTdt = fLT(t, yLT_0)

    phi = 13*pi/180; 
    g = 9.81;
    rho = 1.2923; 
    M = 88000; 
    m = 4000;
    WL = 285;
    S = M/WL; 
    C_pav = 1; 
%     CL = (0.04264*((180/pi)*(yLT_0(3))))-0.0158;
%     CD = (0.01394*((180/pi)*(yLT_0(3)))) - 0.0248;
    CL = (0.04264*((180/pi)*(yLT_0(3))));
    CD = (0.01394*((180/pi)*(yLT_0(3))));
    urol = (0.0041+0.000041*yLT_0(10))*C_pav;
    Dpo = 5;
    Dgo = 3.5;
    D_fo = 18.2;
    uv = 0.7;  %estoura em 3.9 
    y_ext = 0;
    yponto_ext = 0;
    Joz = 16864415*M/88000;
    kr = 13600000;
    cr = 9700;
    kt = 11486800;      %kt = 954910;
    ct = 102196;        %ct = 103870; 
    m_f = 1000; 
    k_tf = 5743400;
    c_tf = 51098;
    k_rf = 3400000;
    c_rf = 2*2425;    
    
    
    dydt1 = yLT_0(5);
    dydt2 = yLT_0(6);
    dydt3 = yLT_0(7);
    dydt4 = yLT_0(8);
    dydt5 = -(((cr + ct)*yLT_0(5))/m) + (ct*yLT_0(6))/m + (0 + kt*(-yLT_0(1) + yLT_0(2))) + kr*(-yLT_0(1) + y_ext) + cr*yponto_ext/m;
    dydt6 = (-(Joz*(2*0 + S*rho*CL*((yLT_0(10) + uv)*(yLT_0(10) + uv)) + 2*kt*(yLT_0(1) - yLT_0(2)))) + M*(Dgo*Dgo)*(-2*g*M + (2*g*M - S*rho*CL*((yLT_0(10) + uv)*(yLT_0(10) + uv)))*urol*yLT_0(3)) + M*Dgo*(2*0 + S*rho*Dpo*((yLT_0(10) + uv)*(yLT_0(10) + uv))*(CL + CD*yLT_0(3))))/(2.*M*(M*(Dgo*Dgo) - Joz)) + (ct*Joz*yLT_0(5))/(M*(-(M*(Dgo*Dgo)) + Joz)) + (ct*Joz*yLT_0(6))/(M*M*(Dgo*Dgo) - M*Joz);
    dydt7 = (-2*0 - S*rho*Dpo*((yLT_0(10) + uv)*(yLT_0(10) + uv))*(CL + CD*yLT_0(3)) + Dgo*(2*g*M + 2*0 + 2*kt*(yLT_0(1) - yLT_0(2)) - 2*g*M*urol*yLT_0(3) + S*rho*CL*((yLT_0(10) + uv)*(yLT_0(10) + uv))*(1 + urol*yLT_0(3))))/(2*M*(Dgo*Dgo) - 2*Joz) + (ct*Dgo*yLT_0(5))/(M*(Dgo*Dgo) - Joz) + (ct*Dgo*yLT_0(6))/(-(M*(Dgo*Dgo)) + Joz);
    dydt8 = 0;
    dydt9 = yLT_0(10);
    dydt10 = (-((M+m)*g - CL*S*rho*((yLT_0(10)+uv)^2)/2)*(urol) - CD*S*rho*((yLT_0(10)+uv)^2)/2)/(M+m);
    dyLTdt = [dydt1; dydt2; dydt3; dydt4; dydt5; dydt6; dydt7; dydt8; dydt9; dydt10];
end
%
function dyAPdt = fAP(t, yAP_0)
phi = 13*pi/180; 
g = 9.81;
rho = 1.2923; 
M = 88000; 
m = 4000;
WL = 285;
S = M/WL; 
C_pav = 1; 
al = 0.04264;
ad = 0.01394;
% CL = (0.04264*((180/pi)*(yAP_0(3))))-0.0158;
% CD = (0.01394*((180/pi)*(yAP_0(3)))) - 0.0248;
CL = (0.04264*((180/pi)*(yAP_0(3))));
CD = (0.01394*((180/pi)*(yAP_0(3))));
urol = (0.0041+0.000041*yAP_0(10))*C_pav;
Dpo = 5;
Dgo = 3.5;
D_fo = 18.2;
uv = 0.7;  %estoura em 3.9 
y_ext = 0;
yponto_ext = 0;
Joz = 16864415*M/88000;
kr = 13600000;
cr = 9700;
kt = 11486800;      %kt = 954910;
ct = 102196;        %ct = 103870; 
m_f = 1000; 
k_tf = 5743400;
c_tf = 51098;
k_rf = 3400000;
c_rf = 2*2425;  
K11 =        -284.3    ; 
    K12 =         -5160     ; 
    K13 =    -7.4562e+03     ; 
    K14 =         3.6209     ; 
    K15 =         -9090     ; 
    K16 =    -7.6314e+03     ; 
    K21 =          52353     ; 
    K22 =     2.6127e+03     ; 
    K23 =     5.0393e+06      ; 
    K24 =         -147.6     ; 
    K25 =      5.262e+03     ; 
    K26 =     5.1107e+07;


if yAP_0(3) > 0
    dydt1 = yAP_0(5);
    dydt2 = yAP_0(6);
    dydt3 = yAP_0(7);
    dydt4 = yAP_0(8);
    dydt5 = -(((K11 + kr + kt)*yAP_0(1) + (K12 - kt)*yAP_0(2) + K13*yAP_0(3) + (cr + K14)*yAP_0(5) + K15*yAP_0(6) + K16*yAP_0(7) - kr*y_ext - cr*yponto_ext)/m);
    dydt6 = ((Joz*(K13*pi - 90*al*rho*S*((yAP_0(10) + uv)*(yAP_0(10) + uv))) + Dgo*M*(-(K23*pi) + Dgo*g*M*pi*urol + 90*al*Dpo*rho*S*((yAP_0(10) + uv)*(yAP_0(10) + uv))))*yAP_0(3) + pi*(-(Dgo*Dgo*g*(M*M)) + (Joz*(K11 - kt) - Dgo*K21*M)*yAP_0(1) + Joz*K12*yAP_0(2) + Joz*kt*yAP_0(2) - Dgo*K22*M*yAP_0(2) - (ct*Joz - Joz*K14 + Dgo*K24*M)*yAP_0(5) + (Joz*(ct + K15) - Dgo*K25*M)*yAP_0(6) + Joz*K16*yAP_0(7) - Dgo*K26*M*yAP_0(7)))/(M*(-Joz + Dgo*Dgo*M)*pi);
    dydt7 = ((-(K23*pi) + Dgo*pi*(K13 + g*M*urol) - 90*al*Dgo*rho*S*((yAP_0(10) + uv)*(yAP_0(10) + uv)) + 90*al*Dpo*rho*S*((yAP_0(10) + uv)*(yAP_0(10) + uv)))*yAP_0(3) + pi*(-(Dgo*g*M) - (K21 + Dgo*(-K11 + kt))*yAP_0(1) + Dgo*K12*yAP_0(2) - K22*yAP_0(2) + Dgo*kt*yAP_0(2) - (ct*Dgo - Dgo*K14 + K24)*yAP_0(5) + (Dgo*(ct + K15) - K25)*yAP_0(6) + Dgo*K16*yAP_0(7) - K26*yAP_0(7)))/((Joz - Dgo*Dgo*M)*pi);
    dydt8 = 0;
    dydt9 = yAP_0(10);
    dydt10 = (-((M+m)*g - CL*S*rho*((yAP_0(10)+uv)^2)/2)*(urol) - CD*S*rho*((yAP_0(10)+uv)^2)/2)/(M+m);
    dyAPdt = [dydt1; dydt2; dydt3; dydt4; dydt5; dydt6; dydt7; dydt8; dydt9; dydt10];
else
    dydt1 = 0;
    dydt2 = 0;
    dydt3 = 0;
    dydt4 = 0;
    dydt5 = 0;
    dydt6 = 0;
    dydt7 = 0;
    dydt8 = 0;
    dydt9 = 0;
    dydt10 = 0;
    dyAPdt = [dydt1; dydt2; dydt3; dydt4; dydt5; dydt6; dydt7; dydt8; dydt9; dydt10];
end
end
%
function dycndt = fcn(t, ycnl_0)
phi = 13*pi/180; 
    g = 9.81;
    rho = 1.2923; 
    C_pav = 1; 
    M = 88000; 
    m = 4000;
    WL = 285;
    S = M/WL; 
   
%     CL = (0.04264*((180/pi)*(y_0(3) + phi)))-0.0158;
%     CD = (0.01394*((180/pi)*(y_0(3) + phi))) - 0.0248;
    CL = (0.04264*((180/pi)*(ycnl_0(3))));
    CD = (0.01394*((180/pi)*(ycnl_0(3))));
    urol = (0.0041+0.000041*ycnl_0(10))*C_pav;
    al = 0.04264;
    ad = 0.01394;
    Dpo = 5;
    Dgo = 3.5;
    D_fo = 18.2;
    uv = 0.7;  %estoura em 3.9 
    y_ext = 0;
    yponto_ext = 0;
    Joz = 16864415*M/88000;
    kr = 13600000;
    cr = 9700;
    kt = 11486800;      %kt = 954910;
    ct = 102196;        %ct = 103870; 
    m_f = 1000; 
    k_tf = 5743400;
    c_tf = 51098;
    k_rf = 3400000;
    c_rf = 2*2425;  
    K11 =        -284.3    ; 
    K12 =         -5160     ; 
    K13 =    -7.4562e+03     ; 
    K14 =         3.6209     ; 
    K15 =         -9090     ; 
    K16 =    -7.6314e+03     ; 
    K21 =          52353     ; 
    K22 =     2.6127e+03     ; 
    K23 =     5.0393e+06      ; 
    K24 =         -147.6     ; 
    K25 =      5.262e+03     ; 
    K26 =     5.1107e+07;

    dycndt1 = ycnl_0(5);
    dycndt2 = ycnl_0(6);
    dycndt3 = ycnl_0(7);
    dycndt4 = ycnl_0(8);
    dycndt5 = -((cr*ycnl_0(5))/m) + (-(K11*ycnl_0(1)+K12*ycnl_0(2)+K13*ycnl_0(3)+K14*ycnl_0(5)+K15*ycnl_0(6)+K16*ycnl_0(7)) - (kr + kt)*ycnl_0(1) + kt*ycnl_0(2) + kr*y_ext + cr*yponto_ext)/m;
    dycndt6 = -((Joz*pi*-(K11*ycnl_0(1)+K12*ycnl_0(2)+K13*ycnl_0(3)+K14*ycnl_0(5)+K15*ycnl_0(6)+K16*ycnl_0(7)) + Joz*kt*pi*(ycnl_0(1) - ycnl_0(2)) + 90*al*Joz*rho*S*((ycnl_0(10) + uv)*(ycnl_0(10) + uv))*ycnl_0(3) + Dgo*M*cos(ycnl_0(3))*(Dgo*g*M*pi*(cos(ycnl_0(3)) - urol*sin(ycnl_0(3))) - pi*(-(K21*ycnl_0(1)+K22*ycnl_0(2)+K23*ycnl_0(3)+K24*ycnl_0(5)+K25*ycnl_0(6)+K26*ycnl_0(7))) - 90*rho*S*((ycnl_0(10) + uv)*(ycnl_0(10) + uv))*(al*Dpo*cos(ycnl_0(3)) + (ad*Dpo - al*Dgo*urol)*sin(ycnl_0(3)))*ycnl_0(3)))/(M*pi*(-Joz + Dgo*Dgo*M*(cos(ycnl_0(3))*cos(ycnl_0(3)))))) + (ct*Joz*ycnl_0(5))/(Joz*M - Dgo*Dgo*(M*M)*(cos(ycnl_0(3))*cos(ycnl_0(3)))) + (ct*Joz*ycnl_0(6))/(-(Joz*M) + Dgo*Dgo*(M*M)*(cos(ycnl_0(3))*cos(ycnl_0(3)))) + (Dgo*Joz*sin(ycnl_0(3))*(ycnl_0(7)*ycnl_0(7)))/(Joz - Dgo*Dgo*M*(cos(ycnl_0(3))*cos(ycnl_0(3))));
    dycndt7 = (sec(ycnl_0(3))*(Dgo*pi*-(K11*ycnl_0(1)+K12*ycnl_0(2)+K13*ycnl_0(3)+K14*ycnl_0(5)+K15*ycnl_0(6)+K16*ycnl_0(7)) - pi*sec(ycnl_0(3))*(-(K21*ycnl_0(1)+K22*ycnl_0(2)+K23*ycnl_0(3)+K24*ycnl_0(5)+K25*ycnl_0(6)+K26*ycnl_0(7))) + Dgo*g*M*pi*(1 - urol*tan(ycnl_0(3))) + Dgo*kt*pi*ycnl_0(1) - Dgo*kt*pi*ycnl_0(2) + 90*al*Dgo*rho*S*(ycnl_0(10)*ycnl_0(10))*ycnl_0(3) - 90*al*Dpo*rho*S*(ycnl_0(10)*ycnl_0(10))*ycnl_0(3) + 180*al*Dgo*rho*S*ycnl_0(10)*uv*ycnl_0(3) - 180*al*Dpo*rho*S*ycnl_0(10)*uv*ycnl_0(3) + 90*al*Dgo*rho*S*(uv*uv)*ycnl_0(3) - 90*al*Dpo*rho*S*(uv*uv)*ycnl_0(3) - 90*rho*S*(ad*Dpo - al*Dgo*urol)*((ycnl_0(10) + uv)*(ycnl_0(10) + uv))*tan(ycnl_0(3))*ycnl_0(3)))/(pi*(Dgo*Dgo*M - Joz*(sec(ycnl_0(3))*sec(ycnl_0(3))))) + (ct*Dgo*sec(ycnl_0(3))*ycnl_0(5))/(Dgo*Dgo*M - Joz*(sec(ycnl_0(3))*sec(ycnl_0(3)))) + (ct*Dgo*cos(ycnl_0(3))*ycnl_0(6))/(Joz - Dgo*Dgo*M*(cos(ycnl_0(3))*cos(ycnl_0(3)))) + (ycnl_0(7)*ycnl_0(7))/(cot(ycnl_0(3)) - (2*Joz*csc(2*ycnl_0(3)))/(Dgo*Dgo*M));
    dycndt8 = 0;
    dycndt9 = ycnl_0(10);
    dycndt10 = (-((M+m)*g - CL*S*rho*((ycnl_0(10)+uv)^2)/2)*(urol) - CD*S*rho*((ycnl_0(10)+uv)^2)/2)/(M+m);
    dycndt = [dycndt1; dycndt2; dycndt3; dycndt4; dycndt5; dycndt6; dycndt7; dycndt8; dycndt9; dycndt10];
end
%
function dyLLdt = fLLT(t, yLLT_0)
phi = 13*pi/180; 
g = 9.81;
rho = 1.2923; 
M = 88000; 
m = 4000;
WL = 285;
S = M/WL; 
C_pav = 1; 
al = 0.04264;
CL = (0.04264*((180/pi)*(yLLT_0(3))))-0.0158;
CD = (0.01394*((180/pi)*(yLLT_0(3)))) - 0.0248;
urol = (0.0041+0.000041*yLLT_0(10))*C_pav;
Dpo = 5;
Dgo = 3.5;
D_fo = 18.2;
uv = 0.7;  %estoura em 3.9 
y_ext = 0.15*sin(5*t);
yponto_ext = 5*0.15*cos(5*t);
Joz = 16864415*M/88000;
kr = 13600000;
cr = 9700;
kt = 11486800;      %kt = 954910;
ct = 102196;        %ct = 103870; 
m_f = 1000; 
k_tf = 5743400;
c_tf = 51098;
k_rf = 3400000;
c_rf = 2*2425;   

dydt1 = yLLT_0(5);
dydt2 = yLLT_0(6);
dydt3 = yLLT_0(7);
dydt4 = yLLT_0(8);
dydt5 = -((cr*yLLT_0(5))/m) + (0 - (kr + kt)*yLLT_0(1) + kt*yLLT_0(2) + kr*y_ext + cr*yponto_ext)/m;
dydt6 = -((-(Joz*0) + Dgo*M*(-(Dgo*g*M) + 0) + Joz*kt*(-yLLT_0(1) + yLLT_0(2)) + Dgo*Dgo*g*(M*M)*urol*yLLT_0(3) - (90*al*(Joz - Dgo*Dpo*M)*rho*S*((yLLT_0(10) + uv)*(yLLT_0(10) + uv))*yLLT_0(3))/pi)/(M*(Joz - Dgo*Dgo*M))) + (ct*Joz*yLLT_0(5))/(Joz*M - Dgo*Dgo*(M*M)) + (ct*Joz*yLLT_0(6))/(M*(-Joz + Dgo*Dgo*M));
dydt7 = (-(Dgo*g*M*pi) - Dgo*pi*0 + pi*0 - Dgo*kt*pi*yLLT_0(1) + Dgo*kt*pi*yLLT_0(2) + 90*al*(-Dgo + Dpo)*rho*S*(yLLT_0(10)*yLLT_0(10))*yLLT_0(3) + Dgo*g*M*pi*urol*yLLT_0(3) - 180*al*Dgo*rho*S*yLLT_0(10)*uv*yLLT_0(3) + 180*al*Dpo*rho*S*yLLT_0(10)*uv*yLLT_0(3) - 90*al*Dgo*rho*S*(uv*uv)*yLLT_0(3) + 90*al*Dpo*rho*S*(uv*uv)*yLLT_0(3))/((Joz - Dgo*Dgo*M)*pi) + (ct*Dgo*yLLT_0(5))/(-Joz + Dgo*Dgo*M) + (ct*Dgo*yLLT_0(6))/(Joz - Dgo*Dgo*M);
dydt8 = 0;
dydt9 = yLLT_0(10);
dydt10 = (-((M+m)*g - CL*S*rho*((yLLT_0(10)+uv)^2)/2)*(urol) - CD*S*rho*((yLLT_0(10)+uv)^2)/2)/(M+m);
dyLLdt = [dydt1; dydt2; dydt3; dydt4; dydt5; dydt6; dydt7; dydt8; dydt9; dydt10];
end