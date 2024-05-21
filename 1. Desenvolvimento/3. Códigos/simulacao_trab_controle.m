% Verificar qual constante está sendo usada nas simulações, a nova para
% controle ou a mesma de modelagem.
%% Definição dos parâmetros iniciais da integração e cálculo das ODES
t = 50; 
T_sim = 1/100;
tempo = 0:T_sim:t;
%Passo máximo ODE
max_step = odeset('MaxStep', T_sim);

AE = 32;
DT = 24.7;

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
uv = 0;


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

x0ob =  [q1_0 q2_0 theta_0 q3_0 q1p_0 q2p_0 thetap_0 q3p_0 pos_ini vel_ini 0 0 0 0];

  [t, y] = ode45(@f, tempo, x0o, max_step);
  [t, yobs] = ode45(@fob, tempo, x0ob, max_step);
  [t, yAP] = ode45(@fAP,tempo, x0, max_step);
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

K11 =          0     ; 
K12 =     -6855    ; 
K13 =    0    ; 
K14 =    0     ; 
K15 =    -7820     ; 
K16 =    0    ; 
K21 =     0     ; 
K22 =     0    ; 
K23 =     3.0688e+07 ; 
K24 =     0     ; 
K25 =     0    ; 
K26 =     7.509e+07; 

Fa = -1*(K11*yAP(:,1)+K12*yAP(:,2)+K13*yAP(:,3)+K14*yAP(:,5)+K15*yAP(:,6)+K16*yAP(:,7));
Ta = -1*(K21*yAP(:,1)+K22*yAP(:,2)+K23*yAP(:,3)+K24*yAP(:,5)+K25*yAP(:,6)+K26*yAP(:,7));
Tmax = 1.3*AE.*0.5.*(y(:,10)+uv).^2.*rho.*DT.*cos(y(:,3)) + 0.25.*AE.*0.5.*(y(:,10)+uv).^2.*rho.*DT.*sin(y(:,3));

% 
figure(101)
plot(tempo, Fa/1000, "b")
% hold on
% plot(tempo, Fa2/1000, "g")
% hold on
% plot(tempo, Fa1/1000, "c")
grid on
grid minor
legend('Força ativa')
title('Força no atuador do trem de pouso')
xlabel('Tempo (s)')
ylabel('Força (KN)')
% 
figure(102)
plot(tempo, (Ta), "b")
% hold on
% plot(tempo, Ta2, "g")
% hold on
% plot(tempo, Ta1, "c")
hold on
plot(tempo, (Tmax), "r")
hold on
plot(tempo, -(Tmax), "r")
grid on
grid minor
legend('Torque ativo', 'Torque máximo fornecido')
title('Controle de torque')
xlabel('Tempo (s)')
ylabel('Torque (N.m)')


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
plot(tempo, (1*y(:,2)), "r")
hold on
plot(tempo, (1*yobs(:,2)), "b")
hold on
plot(tempo, (1*yAP(:,2)), "g")
grid on
grid minor
% hold on
% plot(tempo, (1*ycn1(:,2)), "c")
% ylim([11/10*min2 11/10*max2]);
% p = patch([0 0 T_sim*length(tempofr) T_sim*length(tempofr)],[11/10*min2 11/10*max2 11/10*max2 11/10*min2],'');
% set(p,'FaceAlpha',0.1)
% set(p,'EdgeColor','none')
legend('Original (não linear)','Controlado', 'Sistema não controlado com novas constantes')
title('Variação de q2 para vento de 7m/s')
xlabel('Tempo (s)')
ylabel('Posição (m)')
% 
figure(3)
plot(tempo, (180/pi*y(:,3))+13, "r")
hold on
%plot(tempo, (180/pi*yobs(:,3)), "b")
%hold on
plot(tempo, (180/pi*yAP(:,3)), "g")
% hold on
% plot(tempo, (180/pi*ycn1(:,3)), "c")
grid on
grid minor
% ylim([-5 15]);
%p = patch([0 0 T_sim*length(tempofr) T_sim*length(tempofr)],[11/10*min2 11/10*max2 11/10*max2 11/10*min2],'');
%set(p,'FaceAlpha',0.1)
%set(p,'EdgeColor','none')
legend('Original (não linear)','Controlado')
title('Variação do ângulo de arfagem para vento de 7m/s')
xlabel('Tempo (s)')
ylabel('Ângulo (°)')


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
%fAP -> simulacao do controle linear APLICADO AO NÃO LINEAR

function dydt = f(t, y_0)
    phi = 13*pi/180; 
    g = 9.81;
    rho = 1.2923; 
    C_pav = 1; 
    M = 88000; 
    m = 4000;
    WL = 285;
    S = M/WL; 
    al = 0.0398;   
    ad = 0.0088;
    CL = (al*((180/pi)*(y_0(3) + phi)));
    CD = (ad*((180/pi)*(y_0(3) + phi)));
    urol = (0.0041+0.000041*y_0(10))*C_pav;
    Dpo = 5;
    Dgo = 3.5;
    D_fo = 18.2;
    uv = 0;  %Começa a subir em 3.6
    y_ext = 0;
    yponto_ext = 0;
    Joz = 16864415*M/88000;
    kr = 13600000;
    cr = 9700;
    kt = 11486800;  % Constante de modelagem
    %kt = 954910;   % Constante para controle
    ct = 102196;    % Constante de modelagem       
    %ct = 103870;   % Constante para controle
    m_f = 1000; 
    k_tf = 5743400;
    c_tf = 51098;
    k_rf = 3400000;
    c_rf = 2*2425;
    
    %Os dados que mudam com o chaveamento devem ser inseridos dentro do if/else
%     if -y_0(3) < phi
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
%     else
%         dydt1 = 0;
%         dydt2 = 0;
%         dydt3 = 0;
%         dydt4 = 0;
%         dydt5 = 0;
%         dydt6 = 0;
%         dydt7 = 0;
%         dydt8 = 0;
%         dydt9 = 0;
%         dydt10 = 0;
%         dydt = [dydt1; dydt2; dydt3; dydt4; dydt5; dydt6; dydt7; dydt8; dydt9; dydt10];
% end
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

al = 0.0398;   
ad = 0.0088;
CL = (al*((180/pi)*(yL_0(3) + phi)));
CD = (ad*((180/pi)*(yL_0(3) + phi))) ;
urol = (0.0041+0.000041*yL_0(10))*C_pav;
Dpo = 5;
Dgo = 3.5;
D_fo = 18.2;
uv = 0;  %Começa a subir em 3.6
y_ext = 0;
yponto_ext = 0;
Joz = 16864415*M/88000;
kr = 13600000;
cr = 9700;
kt = 11486800;  % Constante de modelagem
%kt = 954910;   % Constante para controle
ct = 102196;    % Constante de modelagem       
%ct = 103870;   % Constante para controle
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
al = 0.0398;   
ad = 0.0088;
CL = (al*((180/pi)*(y_0(3) + phi)));
CD = (ad*((180/pi)*(y_0(3) + phi)));
urol = (0.0041+0.000041*y_0(10))*C_pav;
Dpo = 5;
Dgo = 3.5;
D_fo = 18.2;
uv = 0;  %Começa a subir em 3.6 
y_ext = 0;
yponto_ext = 0;
Joz = 16864415*M/88000;
kr = 13600000;
cr = 9700;
kt = 11486800;  % Constante de modelagem
%kt = 954910;   % Constante para controle
ct = 102196;    % Constante de modelagem       
%ct = 103870;   % Constante para controle
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
CL = (cl*((180/pi)*(yL_0(3) + phi)));
CD = (cd*((180/pi)*(yL_0(3) + phi)));
urol = (0.0041+0.000041*yL_0(10))*C_pav;
Dpo = 5;
Dgo = 3.5;
D_fo = 18.2;
uv = 0;  %Começa a subir em 3.6
y_ext = 0;
yponto_ext = 0;
Joz = 16864415*M/88000;
kr = 13600000;
cr = 9700;
kt = 11486800;  % Constante de modelagem
%kt = 954910;   % Constante para controle
ct = 102196;    % Constante de modelagem       
%ct = 103870;   % Constante para controle 
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
    al = 0.0398;   
    ad = 0.0088;
    CL = (al*((180/pi)*(yLT_0(3))));
    CD = (ad*((180/pi)*(yLT_0(3))));
    urol = (0.0041+0.000041*yLT_0(10))*C_pav;
    Dpo = 5;
    Dgo = 3.5;
    D_fo = 18.2;
    uv = 0;  %Começa a subir em 3.6 
    y_ext = 0;
    yponto_ext = 0;
    Joz = 16864415*M/88000;
    kr = 13600000;
    cr = 9700;
    kt = 11486800;  % Constante de modelagem
    %kt = 954910;   % Constante para controle
    ct = 102196;    % Constante de modelagem       
    %ct = 103870;   % Constante para controle
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
al = 0.0398;   
ad = 0.0088;
CL = (al*((180/pi)*(yAP_0(3))));
CD = (ad*((180/pi)*(yAP_0(3))));
urol = (0.0041+0.000041*yAP_0(10))*C_pav;
Dpo = 5;
Dgo = 3.5;
D_fo = 18.2;
uv = 0;  %Começa a subir em 3.6 
y_ext = 0;
yponto_ext = 0;
Joz = 16864415*M/88000;
kr = 13600000;
cr = 9700;
kt = 954910;   % Constante para controle   
ct = 103870;   % Constante para controle
m_f = 1000; 
k_tf = 5743400;
c_tf = 51098;
k_rf = 3400000;
c_rf = 2*2425;  
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

    dydt1 = yAP_0(5);
    dydt2 = yAP_0(6);
    dydt3 = yAP_0(7);
    dydt4 = yAP_0(8);
    dydt5 = -(((K11 + kr + kt)*yAP_0(1) + (K12 - kt)*yAP_0(2) + K13*yAP_0(3) + (cr + K14)*yAP_0(5) + K15*yAP_0(6) + K16*yAP_0(7) - kr*y_ext - cr*yponto_ext)/m);
    dydt6 = ((Joz*(K13*pi - 90*al*rho*S*((yAP_0(10) + uv)*(yAP_0(10) + uv))) + Dgo*M*(-(K23*pi) + Dgo*g*M*pi*urol + 90*al*Dpo*rho*S*((yAP_0(10) + uv)*(yAP_0(10) + uv))))*yAP_0(3) + pi*(-(Dgo*Dgo*g*(M*M)) + (Joz*(K11 - kt) - Dgo*K21*M)*yAP_0(1) + Joz*K12*yAP_0(2) + Joz*kt*yAP_0(2) - Dgo*K22*M*yAP_0(2) - (ct*Joz - Joz*K14 + Dgo*K24*M)*yAP_0(5) + (Joz*(ct + K15) - Dgo*K25*M)*yAP_0(6) + Joz*K16*yAP_0(7) - Dgo*K26*M*yAP_0(7)))/(M*(-Joz + Dgo*Dgo*M)*pi);
    dydt7 = ((-(K23*pi) + Dgo*pi*(K13 + g*M*urol) - 90*al*Dgo*rho*S*((yAP_0(10) + uv)*(yAP_0(10) + uv)) + 90*al*Dpo*rho*S*((yAP_0(10) + uv)*(yAP_0(10) + uv)))*yAP_0(3) + pi*(-(Dgo*g*M) - (K21 + Dgo*(-K11 + kt))*yAP_0(1) + Dgo*K12*yAP_0(2) - K22*yAP_0(2) + Dgo*kt*yAP_0(2) - (ct*Dgo - Dgo*K14 + K24)*yAP_0(5) + (Dgo*(ct + K15) - K25)*yAP_0(6) + Dgo*K16*yAP_0(7) - K26*yAP_0(7)))/((Joz - Dgo*Dgo*M)*pi);
    dydt8 = 0;
    dydt9 = yAP_0(10);
    dydt10 = 0;
    dyAPdt = [dydt1; dydt2; dydt3; dydt4; dydt5; dydt6; dydt7; dydt8; dydt9; dydt10];
end
%
function dyobdt = fob(t, yobs)
    phi = 13*pi/180; 
    g = 9.81;
    rho = 1.2923; 
    C_pav = 1; 
    M = 88000; 
    m = 4000;
    WL = 285;
    S = M/WL; 
    al = 0.0398;   
    ad = 0.0088;
    CL = (al*((180/pi)*(yobs(3))));
    CD = (ad*((180/pi)*(yobs(3))));
    urol = (0.0041+0.000041*yobs(10))*C_pav;
    uv = 0;  %Começa a subir em 3.6 
    y_ext = 0;
    yponto_ext = 0;

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

    dydt1 = 0. + 1.*yobs(5);
    dydt2 = 0. + 1.*yobs(6);
    dydt3 = 0. + 1.*yobs(7);
    dydt4 = yobs(8);
    dydt5 = 0. - 3638.7275*yobs(1) + 238.7275*yobs(2) + (0.8608957101079225*K11 - 0.00025*K12 + 35.455509149016116*K14 + 1.1558515814319275*K15 - 0.021223857291119298*K16)*yobs(2) + (47.800234227518814*K11 - 0.00025*K13 + 1913.4930528952211*K14 + 64.24518267273318*K15 - 1.17958159584876*K16)*yobs(3) - 2.425*yobs(5) - 0.00025*K11*yobs(11) - 0.00025*K14*yobs(12) - 0.00025*K15*yobs(13) - (K16*yobs(14))/4000. + 3400*y_ext + (97*yponto_ext)/40.;
    dydt6 = 0.6698911690843045 + 11.592244551327834*yobs(1) + (-11.592244551327834 + 0.0418037871831275*K11 - 0.000012139620017936595*K12 + 1.7216656344461203*K14 + 0.05612639598285879*K15 - 0.0010305982513164056*K16 - 0.0007634754275439303*K21 + 2.2170961551435205e-7*K22 - 0.031443309205175816*K24 - 0.0010250536388437136*K25 + 0.00001882213295898215*K26)*yobs(2) + (-0.002746553793245648 + 2.3211067211617813*K11 - 0.000012139620017936595*K13 + 92.91631427643773*K14 + 3.1196484225196195*K15 - 0.05727868941502081*K16 - 0.042391086208316664*K21 + 2.2170961551435205e-7*K23 - 1.6969592361871328*K24 - 0.05697509899608393*K25 + 0.0010460983283337377*K26 - 0.00002746553793245648*yobs(10) + 0.005018738669957259*(yobs(10)*yobs(10)) + 0.010037477339914518*yobs(10)*uv + 0.005018738669957259*(uv*uv))*yobs(3) + 1.2609423312630743*yobs(5) - 1.260942331263074*yobs(6) + (-0.000012139620017936595*K11 + 2.2170961551435205e-7*K21)*yobs(11) + (-0.000012139620017936595*K14 + 2.2170961551435205e-7*K24)*yobs(12) + (-0.000012139620017936595*K15 + 2.2170961551435205e-7*K25)*yobs(13) + (-0.000012139620017936595*K16 + 2.2170961551435205e-7*K26)*yobs(14);
    dydt7 = -0.19139747688122982 - 0.2117127289508099*yobs(1) + (0.2117127289508099 - 0.0007634754275439303*K11 + 2.2170961551435205e-7*K12 - 0.031443309205175816*K14 - 0.0010250536388437136*K15 + 0.00001882213295898215*K16 + 0.00021813583644112293*K21 - 6.334560443267201e-8*K22 + 0.008983802630050233*K24 + 0.000292872468241061*K25 - 5.377752273994899e-6*K26)*yobs(2) + (0.0007847296552130423 - 0.042391086208316664*K11 + 2.2170961551435205e-7*K13 - 1.6969592361871328*K14 - 0.05697509899608393*K15 + 0.0010460983283337377*K16 + 0.012111738916661905*K21 - 6.334560443267201e-8*K23 + 0.4848454960534665*K24 + 0.016278599713166836*K25 - 0.0002988852366667822*K26 + 7.847296552130423e-6*yobs(10) + 0.000043229937614868696*(yobs(10)*yobs(10)) + 0.0000864598752297374*yobs(10)*uv + 0.0000432299376148687*(uv*uv))*yobs(3) - 0.023028977763475747*yobs(5) + 0.023028977763475747*yobs(6) + (2.2170961551435205e-7*K11 - 6.334560443267201e-8*K21)*yobs(11) + (2.2170961551435205e-7*K14 - 6.334560443267201e-8*K24)*yobs(12) + (2.2170961551435205e-7*K15 - 6.334560443267201e-8*K25)*yobs(13) + (2.2170961551435205e-7*K16 - 6.334560443267201e-8*K26)*yobs(14);
    dydt8 = 0;
    dydt9 = yobs(10);
    dydt10 = 0;
    dydt11 = 169180.8714156337*yobs(2) + 9.560018412184715e6*yobs(3) + 1.*yobs(12) + 3443.58284043169*yobs(13) + 191200.93691007525*yobs(14);
    dydt12 = 6.960771335563102e6*yobs(2) + (0.8608957101079225*K11 - 0.00025*K12 + 35.455509149016116*K14 + 1.1558515814319275*K15 - 0.021223857291119298*K16)*yobs(2) + 3.826974209152756e8*yobs(3) + (47.800234227518814*K11 - 0.00025*K13 + 1913.4930528952211*K14 + 64.24518267273318*K15 - 1.17958159584876*K16)*yobs(3) + (-3638.7275 - 0.00025*K11)*yobs(11) + (-2.425 - 0.00025*K14)*yobs(12) + (141822.03659606446 - 0.00025*K15)*yobs(13) + (7.653972211580885e6 - 0.00025*K16)*yobs(14);
    dydt13 = (227673.3588659011 + 0.0418037871831275*K11 - 0.000012139620017936595*K12 + 1.7216656344461203*K14 + 0.05612639598285879*K15 - 0.0010305982513164056*K16 - 0.0007634754275439303*K21 + 2.2170961551435205e-7*K22 - 0.031443309205175816*K24 - 0.0010250536388437136*K25 + 0.00001882213295898215*K26)*yobs(2) + (1.284899807247856e7 + 2.3211067211617813*K11 - 0.000012139620017936595*K13 + 92.91631427643773*K14 + 3.1196484225196195*K15 - 0.05727868941502081*K16 - 0.042391086208316664*K21 + 2.2170961551435205e-7*K23 - 1.6969592361871328*K24 - 0.05697509899608393*K25 + 0.0010460983283337377*K26 - 0.00002746553793245648*yobs(10) + 0.005018738669957259*(yobs(10)*yobs(10)) + 0.010037477339914518*yobs(10)*uv + 0.005018738669957259*(uv*uv))*yobs(3) + (11.592244551327834 - 0.000012139620017936595*K11 + 2.2170961551435205e-7*K21)*yobs(11) + (1.2609423312630743 - 0.000012139620017936595*K14 + 2.2170961551435205e-7*K24)*yobs(12) + (4622.145383396446 - 0.000012139620017936595*K15 + 2.2170961551435205e-7*K25)*yobs(13) + (256980.7306909327 - 0.000012139620017936595*K16 + 2.2170961551435205e-7*K26)*yobs(14);
    dydt14 = (-4169.47279306513 - 0.0007634754275439303*K11 + 2.2170961551435205e-7*K12 - 0.031443309205175816*K14 - 0.0010250536388437136*K15 + 0.00001882213295898215*K16 + 0.00021813583644112293*K21 - 6.334560443267201e-8*K22 + 0.008983802630050233*K24 + 0.000292872468241061*K25 - 5.377752273994899e-6*K26)*yobs(2) + (-235289.6159881083 - 0.042391086208316664*K11 + 2.2170961551435205e-7*K13 - 1.6969592361871328*K14 - 0.056975098996083925*K15 + 0.0010460983283337377*K16 + 0.012111738916661906*K21 - 6.334560443267201e-8*K23 + 0.4848454960534665*K24 + 0.016278599713166836*K25 - 0.0002988852366667822*K26 + 7.847296552130423e-6*yobs(10) + 0.000043229937614868696*(yobs(10)*yobs(10)) + 0.0000864598752297374*yobs(10)*uv + 0.0000432299376148687*(uv*uv))*yobs(3) + (-0.2117127289508099 + 2.2170961551435205e-7*K11 - 6.334560443267201e-8*K21)*yobs(11) + (-0.023028977763475747 + 2.2170961551435205e-7*K14 - 6.334560443267201e-8*K24)*yobs(12) + (-84.87240018671372 + 2.2170961551435205e-7*K15 - 6.334560443267201e-8*K25)*yobs(13) + (-4718.3263833950405 + 2.2170961551435205e-7*K16 - 6.334560443267201e-8*K26)*yobs(14);

    dyobdt = [dydt1; dydt2; dydt3; dydt4; dydt5; dydt6; dydt7; dydt8; dydt9; dydt10; dydt11; dydt12; dydt13; dydt14];
end