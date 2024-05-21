%%F ACIMA TOMADA COMO A DECOMPOSIÇÃO EM FRAÇÃO PARCIAL DO TERMO (2,1) DA FUNÇÃO DE TRANSFERÊNCIA, OU SEJA, A RESPOSTA À IMPULSO DE POSIÇÃO NA COORDENADA THETA
syms s
Ft = -(0.00110718/(-0.198101+s))+0.00110749/(0.198101 + s)+(-0.813971-0.00353851*s)/(71.879 +0.363522*s+s^2)+(0.915025 +0.0035382*s)/(6325.97 +28.8003*s+ s^2);
fq=ilaplace(Ft);

t_t = 0:0.001:30;

FUNt = matlabFunction(fq);
transt = feval(FUNt, t_t);

% T1 = (4995101593999441.*exp(-(1745045776414265..*t_t)/9007199254740992))/4611686018427387904;
% T2 = (-4993718088193913.*exp((1745045776414265..*t_t)/9007199254740992))/4611686018427387904;
% T3 = (4097621377953287.*exp(-(36599..*t_t)/200000).*(cos((2^(1/2).*24746987492607220882933^(1/2)..*t_t)/26214400000) + (722735512897644203980292096.*2^(1/2).*24746987492607220882933^(1/2).*sin((2^(1/2).*24746987492607220882933^(1/2)..*t_t)/26214400000))/5964928528802938777666596178386444163))/1152921504606846976; 
% T4 = (4097275501501905.*exp(-(15249..*t_t)/800).*(cos((1383066871628038169^(1/2)..*t_t)/13107200) + (2661334583892191449759744.*1383066871628038169^(1/2).*sin((1383066871628038169^(1/2)..*t_t)/13107200))/1133361202012088190548213633242389))/1152921504606846976;

T1 = (1276849037137037*exp(-(7137340718253781.*t_t)/36028797018963968))/1152921504606846976; %-0.1981
T2 = -(5105966525882435*exp((7137340718253781.*t_t)/36028797018963968))/4611686018427387904 ; %0.1981
%-14.4002
T3 = (2039633433799973.*exp(-(4053281885879953.*t_t)/281474976710656).*(cos((484765885173958243527880426844639^(1/2).*t_t)/281474976710656) + (87627553455996499377510610139140827*484765885173958243527880426844639^(1/2)*sin((484765885173958243527880426844639^(1/2).*t_t)/281474976710656))/617965441854027421191054802605228431199920871716875))/576460752303423488 ;
%0.1818
T4 = -(2039812136633187.*exp(-(3274315087481955.*t_t)/18014398509481984).*(cos((5^(1/2)*2914418497390242841477716144047335003^(1/2).*t_t)/450359962737049600) + (6032913674873069652066666895771925*5^(1/2)*2914418497390242841477716144047335003^(1/2)*sin((5^(1/2).*2914418497390242841477716144047335003^(1/2).*t_t)/450359962737049600))/849266603172124797305772179751089821382556616649223))/576460752303423488;

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
legend( "Termo com exp(-0,1981) - T1", "Termo com exp(+0,1981) - T2", "Termo com exp(-14,4) - T3", "Termo com exp(-0,182) - T4")
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
p = patch([0 0 5 5],[-0.4 0.1 0.1 -0.4],'');
ylim([-0.4 0.1])
set(p,'FaceAlpha',0.1)
set(p,'EdgeColor','none')
legend("Termo com exponencial positiva", "Resposta do sistema", "Região passível de análise free-roll")

%%F ACIMA TOMADA COMO A DECOMPOSIÇÃO EM FRAÇÃO PARCIAL DO TERMO (2,1) DA FUNÇÃO DE TRANSFERÊNCIA, OU SEJA, A RESPOSTA À IMPULSO DE POSIÇÃO NA COORDENADA THETA
syms s
Fq=((72.7837+0.317715*s)/(72.0567+0.36599*s+1*s^2)+(-84.7795-0.317715*s)/(8413.84+38.1225*s+1*s^2));
fq=ilaplace(Fq);

t_q = 0:0.1:100;

FUNq = matlabFunction(fq);
transq = feval(FUNq, t_q);
% 
% figure(4)
% plot(t_q, transq, "b")
% grid on
% grid minor
% title("Transformada inversa de Laplace para q2- Impulso")
% xlabel('Tempo (s)')
% ylabel('Amplitude de q2 (metros)')

% figure(5)
% plot(t_t, -0.0011.*exp(0.1981.*t_t), "r", "LineWidth", 1)
% grid on
% grid minor
% title("T2")
% xlabel('Tempo (s)')
% ylabel('Amplitude de Theta (radianos)')
