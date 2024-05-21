%%F ACIMA TOMADA COMO A DECOMPOSIÇÃO EM FRAÇÃO PARCIAL DO TERMO (2,1) DA FUNÇÃO DE TRANSFERÊNCIA, OU SEJA, A RESPOSTA À IMPULSO DE POSIÇÃO NA COORDENADA THETA
syms s
Ft = -(0.00204899/(-0.230479+s))+0.00204966/(0.230479 +s)+(-1.34555-0.00584555*s)/(74.6604 +0.377331*s+s^2)+(1.51282 +0.00584488*s)/(6328.12 +28.833*s+s^2);
fq=ilaplace(Ft)
passo = 0.001;
t_t = 0:passo:20;

T1 = 0.00204966.*exp((-0.230479.*t_t));
T2 = -0.00204899.*exp((0.230479.*t_t));
T3 = +0.00292244.*exp((-14.4165.*t_t)).*cos(78.2322.*t_t)+0.00292244.*exp((-14.4165.*t_t)).*cos(78.2322.*t_t).*cos(156.464.*t_t)+0.0091302.*exp((-14.4165.*t_t)).*sin(78.2322.*t_t)-0.0091302.*exp((-14.4165.*t_t)).*cos(156.464.*t_t).*sin(78.2322.*t_t)+0.0091302.*exp((-14.4165.*t_t)).*cos(78.2322.*t_t).*sin(156.464.*t_t)+0.00292244.*exp((-14.4165.*t_t)).*sin(78.2322.*t_t).*sin(156.464.*t_t); 
T4 = -0.00292278.*exp((-0.188665.*t_t)).*cos(8.63857.*t_t) -0.00292278.*exp((-0.188665.*t_t)).*cos(8.63857.*t_t).*cos(17.2771.*t_t)-0.0778166.*exp((-0.188665.*t_t)).*sin(8.63857.*t_t)+0.0778166.*exp((-0.188665.*t_t)).*cos(17.2771.*t_t).*sin(8.63857.*t_t)-0.0778166.*exp((-0.188665.*t_t)).*cos(8.63857.*t_t).*sin(17.2771.*t_t)-0.00292278.*exp((-0.188665.*t_t)).*sin(8.63857.*t_t).*sin(17.2771.*t_t);


FUNt = matlabFunction(fq);
transt = feval(FUNt, t_t);

figure(1)
plot(t_t, transt, "b")
grid on
grid minor
title("Transformada inversa de Laplace para theta- Impulso")
xlabel('Tempo (s)')
ylabel('Amplitude de Theta (radianos)')


figure(2)
plot(t_t, T1, "r")
hold on
plot(t_t, T2, "g")
hold on
plot(t_t, T3, "b")
hold on
plot(t_t, T4, "m")
grid on
grid minor
legend( "Termo com exp(-0.230479) - T1", "Termo com exp(+0.230479) - T2", "Termo com exp(-14.4165) - T3", "Termo com exp(-0.188665) - T4")
title("Termos da transformada inversa de Laplace para theta - Impulso")
xlabel('Tempo (s)')
ylabel('Amplitude de Theta (radianos)')

figure(3)
plot(t_t, T2, "r", "LineWidth", 1.5)
hold on
plot(t_t, transt, "b")
grid on
grid minor
title("Região de atuação do modelo na transformada inversa de Laplace para theta - Impulso")
xlabel('Tempo (s)')
ylabel('Amplitude de Theta (radianos)')
p = patch([0 0  2 2],[-0.7 0.2 0.2 -0.7],'');
set(p,'FaceAlpha',0.1)
set(p,'EdgeColor','none')
legend("Termo com exponencial positiva", "Resposta do sistema", "Região de free-roll")
% 
% [M1, I1] = max(transt(1:(1/passo)));
% [M2, I2] = max(transt((1/passo):(2/passo)));
% [M3, I3] = max(transt((2/passo):(3/passo)));
% [M4, I4] = max(transt((3/passo):(4/passo)));
% [M5, I5] = max(transt((4/passo):(5/passo)));
% [M7, I7] = max(transt((5/passo):(7/passo)));
% [M10, I10] = max(transt((7/passo):(10/passo)));
% [M20, I20] = max(transt((10/passo):(20/passo)));
% [M50, I50] = max(transt((20/passo):(50/passo)));

% rm1 = T2(I1)/rms(transt(1:(1/passo)))
% rm2 = T2(I2)/rms(transt((1/passo):(2/passo)))
% rm3 = T2(I3)/rms(transt((2/passo):(3/passo)))
% rm4 = T2(I4)/rms(transt((3/passo):(4/passo)))
% rm5 = T2(I5)/rms(transt((4/passo):(5/passo)))


% R1 = T2(I1)/M1
% R2 = T2(I2)/M2
% R3 = T2(I3)/M3
% R4 = T2(I4)/M4
% R5 = T2(I5)/M5
% R7 = T2(I7)/M7
% R10 = T2(I10)/M10
% R20 = T2(I20)/M20


