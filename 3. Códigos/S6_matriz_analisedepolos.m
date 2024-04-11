
%%Análise de polos para variação de velocidade  longitudinal 
resp = [];
intervalovel = 0:0.01:120;

for n = intervalovel
    u = n;
    a = 13*pi/180;
    v = 0;
    
    %primeira coluna
    p11 = 0;
    p21 = 0;
    p31 = 0;
    p41 = -(62717/10);
    p51 = -(96859081111/(44*(-16864415+1078000*(cos(a))^2)));
    p61 = (8.04076*10^7*sec(a))/(2156000-33728830*sec(a)^2);
    
    %segunda coluna
    p12 = 0;
    p22 = 0;
    p32 = 0;
    p42 = 28717/10;
    p52 = 96859081111/(44*(-16864415+1078000*cos(a)^2));
    p62 = -(8.04076*10^7*sec(a))/(2156000-33728830*sec(a)^2);
    
    %terceira coluna
    p13 = 0;
    p23 = 0;
    p33 = 0;
    p43 = 0;
    p53 =  (3491.48*(-0.0248+0.796411*a)*(u+v)^2*cos(a)^2)/(-16864415+1.078*10^6*cos(a)^2)+(1.05752*10^7*cos(a)*sin(a))/(-16864415+1.078*10^6*cos(a)^2)-(3491.48*(-0.0158+2.44309*a)*(u+v)^2*cos(a)*sin(a))/(-16864415+1.078*10^6*cos(a)^2)+(6.125*(0.0041 +0.000041*u)*(1.72656*10^6-399.026*(-0.0158+2.44309*a)*(u+v)^2)*cos(a)^2*1)/(-16864415+1.078*10^6*cos(a)^2);
    p63 = -((1995.13*(-0.0248+0.796411*a)*(u+v)^2*sec(a))/(2.156*10^6-33728830*sec(a)^2))-(6.04296*10^6*(0.0041 +0.000041*u)*sec(a)*1)/(2.156*10^6-33728830*sec(a)^2)+(1396.59*(-0.0158+2.44309*a)*(0.0041 +0.000041*u)*(u+v)^2*sec(a)*1)/(2.156*10^6-33728830*sec(a)^2)-(6.04296*10^6*sec(a)*tan(a))/(2.156*10^6-33728830*sec(a)^2)+(1995.13*(-0.0158+2.44309*a)*(u+v)^2*sec(a)*tan(a))/(2.156*10^6-33728830*sec(a)^2);
    %quarta coluna 
    p14 = 1;
    p24 = 0;
    p34 = 0;
    p44 = -(13987/500);
    p54 = 86173787767/(4400*(16864415 - 1078000*cos(a)^2));
    p64 = (224831*sec(a))/(1078000 - 16864415*sec(a)^2);
    
    %quinta coluna
    p15 = 0;
    p25 = 1;
    p35 = 0;
    p45 = 25549/1000;
    p55 = 1723475755340/(-1484068520000 +9.4864*10^10*cos(a)^2);
    p65 = (357686*sec(a))/(-1078000 + 16864415*sec(a)^2);
    
    %sexta coluna
    p16 = 0;
    p26 = 0;
    p36 = 1;
    p46 = 0;
    p56 = 0;
    p66 = 0;
    
    Polos = [p11 p12 p13 p14 p15 p16; p21 p22 p23 p24 p25 p26; p31 p32 p33 p34 p35 p36; p41 p42 p43 p44 p45 p46; p51 p52 p53 p54 p55 p56; p61 p62 p63 p64 p65 p66];
    
    z = eig(Polos);

    resp = cat(2, resp, z);
end

resp_rel = resp(3:6,:);
resp_imag = imag(resp_rel);
resp_real = real(resp);
relf = abs(resp_rel);
for i=1:length(relf)
    relf(4,i) = -relf(4,i);
end

LineWidth = 5;
figure(1)
plot(intervalovel, resp_real(3:4,:), "b", "LineWidth", 1.25)
hold on
plot(intervalovel, relf(3,:), "r", "LineWidth", 1.25)
hold on
plot(intervalovel, relf(4,:), "c", "LineWidth", 1.25)
grid on
grid minor
p = patch([280/3.6 280/3.6 300/3.6 300/3.6],[-0.3 0.3 0.3 -0.3],'');
set(p,'FaceAlpha',0.25)
set(p,'EdgeColor','none')
legend( "", "Parte real do complexo conjugado", "Real positivo", "Real negativo", "Velocidade de aproximação")
title("Polos relevantes do sistema para Phi " + 180*a/pi + "° e velocidade vento " + v + "m/s")
xlabel('Velocidade longitudinal (m/s)')


%%%%%%%%%%%%%%%%%%%%%%Análise de polos para variação de velocidade do vento
resp = [];
intervalovento = -20:0.01:40;

for n = intervalovento
    u = 83.35;
    a = 13*pi/180;
    v = n;
        %primeira coluna
    p11 = 0;
    p21 = 0;
    p31 = 0;
    p41 = -(62717/10);
    p51 = -(96859081111/(44*(-16864415+1078000*(cos(a))^2)));
    p61 = (8.04076*10^7*sec(a))/(2156000-33728830*sec(a)^2);
    
    %segunda coluna
    p12 = 0;
    p22 = 0;
    p32 = 0;
    p42 = 28717/10;
    p52 = 96859081111/(44*(-16864415+1078000*cos(a)^2));
    p62 = -(8.04076*10^7*sec(a))/(2156000-33728830*sec(a)^2);
    
    %terceira coluna
    p13 = 0;
    p23 = 0;
    p33 = 0;
    p43 = 0;
    p53 =  (3491.48*(-0.0248+0.796411*a)*(u+v)^2*cos(a)^2)/(-16864415+1.078*10^6*cos(a)^2)+(1.05752*10^7*cos(a)*sin(a))/(-16864415+1.078*10^6*cos(a)^2)-(3491.48*(-0.0158+2.44309*a)*(u+v)^2*cos(a)*sin(a))/(-16864415+1.078*10^6*cos(a)^2)+(6.125*(0.0041 +0.000041*u)*(1.72656*10^6-399.026*(-0.0158+2.44309*a)*(u+v)^2)*cos(a)^2*1)/(-16864415+1.078*10^6*cos(a)^2);
    p63 = -((1995.13*(-0.0248+0.796411*a)*(u+v)^2*sec(a))/(2.156*10^6-33728830*sec(a)^2))-(6.04296*10^6*(0.0041 +0.000041*u)*sec(a)*1)/(2.156*10^6-33728830*sec(a)^2)+(1396.59*(-0.0158+2.44309*a)*(0.0041 +0.000041*u)*(u+v)^2*sec(a)*1)/(2.156*10^6-33728830*sec(a)^2)-(6.04296*10^6*sec(a)*tan(a))/(2.156*10^6-33728830*sec(a)^2)+(1995.13*(-0.0158+2.44309*a)*(u+v)^2*sec(a)*tan(a))/(2.156*10^6-33728830*sec(a)^2);
    %quarta coluna 
    p14 = 1;
    p24 = 0;
    p34 = 0;
    p44 = -(13987/500);
    p54 = 86173787767/(4400*(16864415 - 1078000*cos(a)^2));
    p64 = (224831*sec(a))/(1078000 - 16864415*sec(a)^2);
    
    %quinta coluna
    p15 = 0;
    p25 = 1;
    p35 = 0;
    p45 = 25549/1000;
    p55 = 1723475755340/(-1484068520000 +9.4864*10^10*cos(a)^2);
    p65 = (357686*sec(a))/(-1078000 + 16864415*sec(a)^2);
    
    %sexta coluna
    p16 = 0;
    p26 = 0;
    p36 = 1;
    p46 = 0;
    p56 = 0;
    p66 = 0;
    
    Polos = [p11 p12 p13 p14 p15 p16; p21 p22 p23 p24 p25 p26; p31 p32 p33 p34 p35 p36; p41 p42 p43 p44 p45 p46; p51 p52 p53 p54 p55 p56; p61 p62 p63 p64 p65 p66];
    
    z = eig(Polos);
    resp = cat(2, resp, z);
end

resp_rel = resp(3:6,:);
resp_imag = imag(resp_rel);
resp_real = real(resp);

relf = abs(resp_rel);
for i=1:length(relf)
    relf(4,i) = -relf(4,i);
end

% plot(intervalo, resp_imag(1:2,:), "b--", "LineWidth", 1.25)
% hold on
figure(2)
plot(intervalovento, resp_real(3:4,:), "b", "LineWidth", 1.25)
hold on
plot(intervalovento, relf(3,:), "r", "LineWidth", 1.25)
hold on
plot(intervalovento, relf(4,:), "c", "LineWidth", 1.25)
grid on
grid minor
legend( "", "Parte real do complexo conjugado", "Real positivo", "Real negativo" )
title("Polos relevantes do sistema para Phi " + 180*a/pi + "° e velocidade longitudinal " + u + "m/s")
xlabel('Velocidade do vento (m/s)')

%%%%%%%%%%%%%%%%%%%%%%Análise de polos para variação de ângulo
resp = [];
intervaloang = 0*pi/180:0.001:14*pi/180;

for n = intervaloang
    u = 83.35;
    a = n;
    v = 0;
        %primeira coluna
    p11 = 0;
    p21 = 0;
    p31 = 0;
    p41 = -(62717/10);
    p51 = -(96859081111/(44*(-16864415+1078000*(cos(a))^2)));
    p61 = (8.04076*10^7*sec(a))/(2156000-33728830*sec(a)^2);
    
    %segunda coluna
    p12 = 0;
    p22 = 0;
    p32 = 0;
    p42 = 28717/10;
    p52 = 96859081111/(44*(-16864415+1078000*cos(a)^2));
    p62 = -(8.04076*10^7*sec(a))/(2156000-33728830*sec(a)^2);
    
    %terceira coluna
    p13 = 0;
    p23 = 0;
    p33 = 0;
    p43 = 0;
    p53 =  (3491.48*(-0.0248+0.796411*a)*(u+v)^2*cos(a)^2)/(-16864415+1.078*10^6*cos(a)^2)+(1.05752*10^7*cos(a)*sin(a))/(-16864415+1.078*10^6*cos(a)^2)-(3491.48*(-0.0158+2.44309*a)*(u+v)^2*cos(a)*sin(a))/(-16864415+1.078*10^6*cos(a)^2)+(6.125*(0.0041 +0.000041*u)*(1.72656*10^6-399.026*(-0.0158+2.44309*a)*(u+v)^2)*cos(a)^2*1)/(-16864415+1.078*10^6*cos(a)^2);
    p63 = -((1995.13*(-0.0248+0.796411*a)*(u+v)^2*sec(a))/(2.156*10^6-33728830*sec(a)^2))-(6.04296*10^6*(0.0041 +0.000041*u)*sec(a)*1)/(2.156*10^6-33728830*sec(a)^2)+(1396.59*(-0.0158+2.44309*a)*(0.0041 +0.000041*u)*(u+v)^2*sec(a)*1)/(2.156*10^6-33728830*sec(a)^2)-(6.04296*10^6*sec(a)*tan(a))/(2.156*10^6-33728830*sec(a)^2)+(1995.13*(-0.0158+2.44309*a)*(u+v)^2*sec(a)*tan(a))/(2.156*10^6-33728830*sec(a)^2);
    %quarta coluna 
    p14 = 1;
    p24 = 0;
    p34 = 0;
    p44 = -(13987/500);
    p54 = 86173787767/(4400*(16864415 - 1078000*cos(a)^2));
    p64 = (224831*sec(a))/(1078000 - 16864415*sec(a)^2);
    
    %quinta coluna
    p15 = 0;
    p25 = 1;
    p35 = 0;
    p45 = 25549/1000;
    p55 = 1723475755340/(-1484068520000 +9.4864*10^10*cos(a)^2);
    p65 = (357686*sec(a))/(-1078000 + 16864415*sec(a)^2);
    
    %sexta coluna
    p16 = 0;
    p26 = 0;
    p36 = 1;
    p46 = 0;
    p56 = 0;
    p66 = 0;
    
    Polos = [p11 p12 p13 p14 p15 p16; p21 p22 p23 p24 p25 p26; p31 p32 p33 p34 p35 p36; p41 p42 p43 p44 p45 p46; p51 p52 p53 p54 p55 p56; p61 p62 p63 p64 p65 p66];
    
    z = eig(Polos);
    resp = cat(2, resp, z);
end

resp_rel = resp(3:6,:);
resp_imag = imag(resp_rel);
resp_real = real(resp);

relf = abs(resp_rel);
for i=1:length(relf)
    relf(4,i) = -relf(4,i);
end

% plot(intervalo, resp_imag(1:2,:), "b--")
% hold on
figure(3)
plot(intervaloang*180/pi, resp_real(3:4,:), "b", "LineWidth", 1.25)
hold on
plot(intervaloang*180/pi, relf(3,:), "r", "LineWidth", 1.25)
hold on
plot(intervaloang*180/pi, relf(4,:), "c", "LineWidth", 1.25)
grid on
grid minor
% p = patch([9 9 13 13],[-0.25 0.25 0.25 -0.25],'');
% set(p,'FaceAlpha',0.25)
% set(p,'EdgeColor','none')
%xlim([0 14]) %O gráfico retrata o ângulo de ataque alpha. 
legend("Parte real do complexo conjugado", "Real positivo", "Real negativo", "Ângulo de toque")
title("Polos relevantes do sistema para vento " + v + "m/s e velocidade longitudinal " + u + "m/s")
xlabel('Ângulo de ataque (graus)')
