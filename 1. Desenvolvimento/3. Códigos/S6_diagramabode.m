plotoptions = bodeoptions;
plotoptions.Grid = 'on';
plotoptions.FreqUnits = 'Hz';
plotoptions.FreqScale = 'log';
plotoptions.Title.FontSize = 12;
plotoptions.XLabel.FontSize = 10;
plotoptions.YLabel.FontSize = 10;


%% Para yext
sysq2y = tf([0 0 0 4203.58 472482],[1 29.2103 6413.66 4500.48 472460]);
% figure(1)
% bodeplot(sysq2y, plotoptions)
% 
systy = tf([0 0 0 -74.8037 -74.8037*112.4 +74.8037*2.35569*10^(-14) -74.8037*5.3497*10^(-13)], [1 29.2103 6413.61 4538.93 472119 -241.186 -25097.3]);
% figure(2)
% bodeplot(systy, plotoptions)
% 
sysq2pontoy = tf([0 0 4203.58 472482 0],[1 29.2103 6413.66 4500.48 472460]);
% figure(4)
% bodeplot(sysq2pontoy, plotoptions)
% 
systpontoy = tf([0 0 -74.8037 -8407.92 0 0 0], [1 29.2103 6413.61 4538.93 472119 -241.186 -25097.3]);
% figure(5)
% bodeplot(systpontoy, plotoptions)

%% Para yextponto
sysq2yponto = tf([0 0 0 2.99814 336.991],[1 29.2103 6413.66 4500.48 472460]);
% figure(5)
% bodeplot(sysq2yponto, plotoptions)
% 
systyponto = tf([0 0 0 -0.0533526 -0.0533526*112.4 +0.0533526*1.06543*10^(-12) 0], [1 29.2103 6413.61 4538.93 472119 -241.186 -25097.3]);
% figure(6)
% bodeplot(systyponto, plotoptions)
% 
sysq2pontoyponto = tf([0 0 2.99814 336.991 0],[1 29.2103 6413.66 4500.48 472460]);
% figure(7)
% bodeplot(sysq2pontoyponto, plotoptions)
% 
systpontoyponto = tf([0 0 -0.0533526 -5.99682 0 0 0], [1 29.2103 6413.61 4538.93 472119 -241.186 -25097.3]);
% figure(8)
% bodeplot(systpontoyponto, plotoptions)

%% Plots conjuntos
figure(10)
margin(sysq2y, plotoptions)
hold on
bodeplot(sysq2yponto, plotoptions)
legend("y_ext", "yponto_ext", 'FontSize', 10)
title("Diagrama de Bode para q2")


figure(11)
bodeplot(systy, plotoptions)
hold on
bodeplot(systyponto, plotoptions)
legend("y_ext", "yponto_ext", 'FontSize', 10)
title("Diagrama de Bode para theta")

figure(12)
bodeplot(sysq2pontoy, plotoptions)
hold on
bodeplot(sysq2pontoyponto, plotoptions)
legend("y_ext", "yponto_ext", 'FontSize', 10)
title("Diagrama de Bode para q2 ponto")

figure(13)
bodeplot(systpontoy, plotoptions)
hold on
bodeplot(systpontoyponto, plotoptions)
legend("y_ext", "yponto_ext", 'FontSize', 10)
title("Diagrama de Bode para theta ponto")