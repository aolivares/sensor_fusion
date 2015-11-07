% -------------------------------------------------------------------------
%  'Accurate human limb angle measurement: sensor fusion through Kalman, 
%    least mean squares and recursive least-squares adaptive filtering'
%--------------------------------------------------------------------------
%
% *************************************************************************
% - Author: Alberto Olivares-Vicente.
% - Entity: University of Granada, Spain.
% - Last revision: 11/07/2015 (mm/dd/yy).
% *************************************************************************
%
% - DESCRIPTION: 
% The following file is the main routine of the experiments reflected in 
% the paper. 
%
% - PAPER ABSTRACT: 
% Inertial sensors are widely used in human body motion monitoring systems
% since they permit us to determine the position of the subject’s limbs. 
% Limb angle measurement is carried out through the integration of the 
% angular velocity measured by a rate sensor and the decomposition of the 
% components of static gravity acceleration measured by an accelerometer.
% Different factors derived from the sensors’ nature, such as the angle 
% random walk and dynamic bias, lead to erroneous measurements. Dynamic 
% bias effects can be reduced through the use of adaptive filtering based 
% on sensor fusion concepts. Most existing published works use a Kalman 
% filtering sensor fusion approach. Our aim is to perform a comparative 
% study among different adaptive filters. Several least mean squares (LMS),
% recursive least squares (RLS) and Kalman filtering variations are tested 
% for the purpose of finding the best method leading to a more accurate and
% robust limb angle measurement. A new angle wander compensation sensor
% fusion approach based on LMS and RLS filters has been developed.

%% Calibración de la aceleración.

[axC,ayC,azC]=calibracionA(ax,ay,az,Ugpx,Ugax,Ugpy,Ugay,Ugpz,Ugaz);

%% Calibración de la velocidad angular

%1) Traslación a la escala de º/s de los datos del giróscopo 
%usando las rectas de calibración de cada eje.
%--------------------------------------------------------------------------
gxE=Ngx.*gx+bgx;
gyE=Ngy.*gy+bgy;

% %2) Corrección de la desviación dinámica.
% %--------------------------------------------------------------------------
% 
% 
% %Definimos los umbrales a partir de los cuales se considera que el wagyro
% %está estático:
% 
% umbral_x=1.58;
% umbral_y=1.435;
% 
% %Determinamos el tiempo a partir del cual recalcularemos la desviación, es
% %decir, el tiempo que lleva parado Wagyro para empezar a calcular la
% %desviación, este tiempo (2 segundos) se traduce a muestras, puesto que la
% %frecuencia de muestreo es 20 muestras por segundo necesitaremos 40
% %muestras.
% 
% N_x=40;
% N_y=200;
% 
% %A continuación llamamos a la función que realiza la corrección de la
% %desviación y nos devuelve el vector de muestras ya corregido.
% 
% velCorr_x=corrDesvMod(gxE,N_x,umbral_x);
% velCorr_y=corrDesvMod(gyE,N_y,umbral_y);


%% Cálculo de los ángulos.

%--------------------------------------------------------------------------
% Usando la señal del acelerómetro
%--------------------------------------------------------------------------
[cabeceo_Ac,alabeo_Ac]=cabyAla_Ac(axC,ayC,azC);
%--------------------------------------------------------------------------
% Usando la señal del giróscopo.
%--------------------------------------------------------------------------
frec=20;
[cabeceo_gi,alabeo_gi]=cabyAlaGiro(gxE,gyE,frec,cabeceo_Ac,alabeo_Ac);

%--------------------------------------------------------------------------
% CORRECCIÓN DE BIAS
%--------------------------------------------------------------------------

    %Obtención de bias mediante fusión de sensores.
    biasY=alabeo_Ac-alabeo_gi;
    biasY=-biasY;
    biasY=biasY+alabeo_gi(1);
    
    biasX=cabeceo_Ac-cabeceo_gi;
    biasX=-biasX;
    biasX=biasX+cabeceo_gi(1);
%% Filtrado de Kalman-----------------------------------------------------

frec=20;
szini=0.3;        incsz=0.5;        szfin=2;
var_aini=0.1;       var_ainc=0.1;       var_afin=0.4;
var_dini=0.005;     var_dinc=0.005;     var_dfin=0.02;

anguloKalmanShow(tiempo,alabeo_gi,alabeo_Ac,frec,szini,incsz,szfin,var_aini,var_ainc,var_afin,var_dini,var_dinc,var_dfin);
anguloKalmanShow(tiempo,cabeceo_gi,cabeceo_Ac,frec,szini,incsz,szfin,var_aini,var_ainc,var_afin,var_dini,var_dinc,var_dfin);

Sz=0.03;    var_angulo= 0.3;    var_desv= 0.05;

alabeo_Kal_Re=anguloKalman(tiempo,alabeo_gi,alabeo_Ac,frec,Sz,var_angulo,var_desv);
cabeceo_Kal_Re=anguloKalman(tiempo,cabeceo_gi,cabeceo_Ac,frec,Sz,var_angulo,var_desv);

Sz=0.3;    var_angulo= 0.1;    var_desv= 0.05; 

cabeceo_Kal_Re2=anguloKalman(tiempo,cabeceo_gi,cabeceo_Ac,frec,Sz,var_angulo,var_desv);

% %% NLMS-------------------------------------------------------------------
% %//////////////////////////////////////////////////////////////////////////
% % En primer lugar se muestra gráficamente un bucle para distintos valores 
% % de los parámetros del filtro adaptativo (L y mu).
% 
% Lini=250;   Lfin=500;   incL=50;    %Longitud del filtro.
% muini=0.5;  mufin=0.7;  incmu=0.1;  %Longitud de paso.
% 
% %Aplicando recta de regresión al bias estimado mediante fusión
% anguloNLMSshow(alabeo_gi,alabeo_Ac,biasY,Lini,Lfin,incL,muini,mufin,incmu,1);
% anguloNLMSshow(cabeceo_gi,cabeceo_Ac,biasX,Lini,Lfin,incL,muini,mufin,incmu,1);
% 
% %% MNLMS------------------------------------------------------------
% %//////////////////////////////////////////////////////////////////////////
% %En primer lugar se muestra gráficamente un bucle para distintos valores de
% %los parámetros del filtro adaptativo (L y mu).
% 
% Lini=300;       Lfin=500;       incL=50;          %Longitud del filtro.
% muini=1*10^-11; mufin=1*10^-11; incmu=1*10^-11;   %Longitud de paso.
% 
% %Aplicando recta de regresión al bias estimado mediante fusión
% anguloMNLMSshow(alabeo_gi+300,alabeo_Ac+300,biasY+300,Lini,Lfin,incL,muini,mufin,incmu,1);
% anguloMNLMSshow(alabeo_gi+8000,alabeo_Ac+8000,biasY+8000,Lini,Lfin,incL,muini,mufin,incmu,1);
% anguloMNLMSshow(alabeo_gi,alabeo_Ac,biasY,Lini,Lfin,incL,muini,mufin,incmu,1);
% %Sin aplicar recta de regresión al bias estimado mediante fusión
% anguloMNLMSshow(alabeo_gi,alabeo_Ac,biasY,Lini,Lfin,incL,muini,mufin,incmu,0);
% 
% %% ENLMS
% 
% Lini=5;        Lfin=20;        incL=5;     %Longitud del filtro.
% muini=0.01;     mufin=0.1;      incmu=0.01;  %Longitud de paso.
% epsi=0.001;
% 
% %Aplicando recta de regresión al bias estimado mediante fusión
% anguloENLMSshow(alabeo_gi+300,alabeo_Ac+300,biasY+300,Lini,Lfin,incL,muini,mufin,incmu,epsi,1);
% 
% %% RLS
% %En primer lugar se muestra gráficamente un bucle para distintos valores de
% %los parámetros del filtro adaptativo (p)
% 
% pini=5;    pfin=20;   incp=5;    %Longitud del filtro.
% delta=100;
% 
% %Aplicando recta de regresión al bias estimado mediante fusión
% anguloRLSshow(alabeo_gi+300,alabeo_Ac+300,biasY+300,pini,pfin,incp,delta,1);

%%
%--------------------------------------------------------------------------
% ESQUEMA B
%--------------------------------------------------------------------------

 %% NLMS

Lini=400;   Lfin=600;   incL=100;    %Longitud del filtro.
muini=0.1;    mufin=1;  incmu=0.1;  %Longitud de paso.

anguloNLMSshowB(alabeo_gi,alabeo_Ac,Lini,Lfin,incL,muini,mufin,incmu);
anguloNLMSshowB(cabeceo_gi,cabeceo_Ac,Lini,Lfin,incL,muini,mufin,incmu);

L=400;
%/////////////////
mu=0.1;
[alabeoNLMS1,w_NLMS1y]=anguloNLMSB(alabeo_gi,alabeo_Ac,mu,L);
[cabeceoNLMS1,w_NLMS1x]=anguloNLMSB(cabeceo_gi,cabeceo_Ac,mu,L);
mu=0.2;
[alabeoNLMS2,w_NLMS2y]=anguloNLMSB(alabeo_gi,alabeo_Ac,mu,L);
[cabeceoNLMS2,w_NLMS2x]=anguloNLMSB(cabeceo_gi,cabeceo_Ac,mu,L);
mu=0.3;
[alabeoNLMS3,w_NLMS3y]=anguloNLMSB(alabeo_gi,alabeo_Ac,mu,L);
[cabeceoNLMS3,w_NLMS3x]=anguloNLMSB(cabeceo_gi,cabeceo_Ac,mu,L);
mu=0.4;
[alabeoNLMS4,w_NLMS4y]=anguloNLMSB(alabeo_gi,alabeo_Ac,mu,L);
[cabeceoNLMS4,w_NLMS4x]=anguloNLMSB(cabeceo_gi,cabeceo_Ac,mu,L);
mu=0.5;
[alabeoNLMS5,w_NLMS5y]=anguloNLMSB(alabeo_gi,alabeo_Ac,mu,L);
[cabeceoNLMS5,w_NLMS5x]=anguloNLMSB(cabeceo_gi,cabeceo_Ac,mu,L);
mu=0.6;
[alabeoNLMS6,w_NLMS6y]=anguloNLMSB(alabeo_gi,alabeo_Ac,mu,L);
[cabeceoNLMS6,w_NLMS6x]=anguloNLMSB(cabeceo_gi,cabeceo_Ac,mu,L);


%% MNLMS

Lini=100;       Lfin=200;       incL=50;        %Longitud del filtro.
muini=1*10^-7;  mufin=9*10^-7;  incmu=1*10^-7;  %Longitud de paso.

anguloMNLMSshowB(alabeo_gi,alabeo_Ac,Lini,Lfin,incL,muini,mufin,incmu);
anguloMNLMSshowB(cabeceo_gi,cabeceo_Ac,Lini,Lfin,incL,muini,mufin,incmu);

L=100;

%/////////////////
mu=3*10^-7;
[alabeoMNLMS1,w_MNLMS1y]=anguloMNLMSB(alabeo_gi,alabeo_Ac,mu,L);
[cabeceoMNLMS1,w_MNLMS1x]=anguloMNLMSB(cabeceo_gi,cabeceo_Ac,mu,L);
mu=4*10^-7;
[alabeoMNLMS2,w_MNLMS2y]=anguloMNLMSB(alabeo_gi,alabeo_Ac,mu,L);
[cabeceoMNLMS2,w_MNLMS2x]=anguloMNLMSB(cabeceo_gi,cabeceo_Ac,mu,L);
mu=5*10^-7;
[alabeoMNLMS3,w_MNLMS3y]=anguloMNLMSB(alabeo_gi,alabeo_Ac,mu,L);
[cabeceoMNLMS3,w_MNLMS3x]=anguloMNLMSB(cabeceo_gi,cabeceo_Ac,mu,L);
mu=6*10^-7;
[alabeoMNLMS4,w_MNLMS4y]=anguloMNLMSB(alabeo_gi,alabeo_Ac,mu,L);
[cabeceoMNLMS4,w_MNLMS4x]=anguloMNLMSB(cabeceo_gi,cabeceo_Ac,mu,L);
mu=7*10^-7;
[alabeoMNLMS5,w_MNLMS5y]=anguloMNLMSB(alabeo_gi,alabeo_Ac,mu,L);
[cabeceoMNLMS5,w_MNLMS5x]=anguloMNLMSB(cabeceo_gi,cabeceo_Ac,mu,L);
mu=8*10^-7;
[alabeoMNLMS6,w_MNLMS6y]=anguloMNLMSB(alabeo_gi,alabeo_Ac,mu,L);
[cabeceoMNLMS6,w_MNLMS6x]=anguloMNLMSB(cabeceo_gi,cabeceo_Ac,mu,L);

%% RLS 

pini=100;    pfin=150;   incp=5;    %Longitud del filtro.

delta=100;

%Aplicando recta de regresión al bias estimado mediante fusión
anguloRLSshowB(alabeo_gi,alabeo_Ac,pini,pfin,incp,delta);
anguloRLSshowB(cabeceo_gi,cabeceo_Ac,pini,pfin,incp,delta);

delta=100;
%///////////////
p=5;
[alabeoRLS1,w_RLS1y]=anguloRLSb(alabeo_gi,alabeo_Ac,p,delta);
[cabeceoRLS1,w_RLS1x]=anguloRLSb(cabeceo_gi,cabeceo_Ac,p,delta);
p=20;
[alabeoRLS2,w_RLS2y]=anguloRLSb(alabeo_gi,alabeo_Ac,p,delta);
[cabeceoRLS2,w_RLS2x]=anguloRLSb(cabeceo_gi,cabeceo_Ac,p,delta);
p=40;
[alabeoRLS3,w_RLS3y]=anguloRLSb(alabeo_gi,alabeo_Ac,p,delta);
[cabeceoRLS3,w_RLS3x]=anguloRLSb(cabeceo_gi,cabeceo_Ac,p,delta);
p=70;
[alabeoRLS4,w_RLS4y]=anguloRLSb(alabeo_gi,alabeo_Ac,p,delta);
[cabeceoRLS4,w_RLS4x]=anguloRLSb(cabeceo_gi,cabeceo_Ac,p,delta);
p=100;
[alabeoRLS5,w_RLS5y]=anguloRLSb(alabeo_gi,alabeo_Ac,p,delta);
[cabeceoRLS5,w_RLS5x]=anguloRLSb(cabeceo_gi,cabeceo_Ac,p,delta);
p=130;
[alabeoRLS6,w_RLS6y]=anguloRLSb(alabeo_gi,alabeo_Ac,p,delta);
[cabeceoRLS6,w_RLS6x]=anguloRLSb(cabeceo_gi,cabeceo_Ac,p,delta);

%% HRLS

pini=5;     pfin=40;    incp=5;         %Longitud del filtro.
lam=0.99;
delta=10;

anguloHRLSshowB(alabeo_gi,alabeo_Ac,pini,pfin,incp,lam,delta);
anguloHRLSshowB(cabeceo_gi,cabeceo_Ac,pini,pfin,incp,lam,delta);

p=5;
[alabeoHRLS1,w_HRLS1y]=anguloHRLSB(alabeo_gi,alabeo_Ac,p,lam,delta);
[cabeceoHRLS1,w_HRLS1x]=anguloHRLSB(cabeceo_gi,cabeceo_Ac,p,lam,delta);
p=20;
[alabeoHRLS2,w_HRLS2y]=anguloHRLSB(alabeo_gi,alabeo_Ac,p,lam,delta);
[cabeceoHRLS2,w_HRLS2x]=anguloHRLSB(cabeceo_gi,cabeceo_Ac,p,lam,delta);
p=40;
[alabeoHRLS3,w_HRLS3y]=anguloHRLSB(alabeo_gi,alabeo_Ac,p,lam,delta);
[cabeceoHRLS3,w_HRLS3x]=anguloHRLSB(cabeceo_gi,cabeceo_Ac,p,lam,delta);
p=70;
[alabeoHRLS4,w_HRLS4y]=anguloHRLSB(alabeo_gi,alabeo_Ac,p,lam,delta);
[cabeceoHRLS4,w_HRLS4x]=anguloHRLSB(cabeceo_gi,cabeceo_Ac,p,lam,delta);
p=90;
[alabeoHRLS5,w_HRLS5y]=anguloHRLSB(alabeo_gi,alabeo_Ac,p,lam,delta);
[cabeceoHRLS5,w_HRLS5x]=anguloHRLSB(cabeceo_gi,cabeceo_Ac,p,lam,delta);
p=100;
[alabeoHRLS6,w_HRLS6y]=anguloHRLSB(alabeo_gi,alabeo_Ac,p,lam,delta);
[cabeceoHRLS6,w_HRLS6x]=anguloHRLSB(cabeceo_gi,cabeceo_Ac,p,lam,delta);

%% QRDRLS

%En primer lugar se muestra gráficamente un bucle para distintos valores de
%los parámetros del filtro adaptativo (p)

pini=1;     pfin=10;    incp=1;         %Longitud del filtro.
lam=0.99;
delta=10;

anguloQRDRLSshowB(alabeo_gi,alabeo_Ac,pini,pfin,incp,lam,delta); 
anguloQRDRLSshowB(cabeceo_gi,cabeceo_Ac,pini,pfin,incp,lam,delta); 

p=5;
[alabeoQRDRLS1,w_QRDRLS1y]=anguloQRDRLSB(alabeo_gi,alabeo_Ac,p,lam,delta);
[cabeceoQRDRLS1,w_QRDRLS1x]=anguloQRDRLSB(cabeceo_gi,cabeceo_Ac,p,lam,delta);
p=10;
[alabeoQRDRLS2,w_QRDRLS2y]=anguloQRDRLSB(alabeo_gi,alabeo_Ac,p,lam,delta);
[cabeceoQRDRLS2,w_QRDRLS2x]=anguloQRDRLSB(cabeceo_gi,cabeceo_Ac,p,lam,delta);
p=15;
[alabeoQRDRLS3,w_QRDRLS3y]=anguloQRDRLSB(alabeo_gi,alabeo_Ac,p,lam,delta);
[cabeceoQRDRLS3,w_QRDRLS3x]=anguloQRDRLSB(cabeceo_gi,cabeceo_Ac,p,lam,delta);
p=20;
[alabeoQRDRLS4,w_QRDRLS4y]=anguloQRDRLSB(alabeo_gi,alabeo_Ac,p,lam,delta);
[cabeceoQRDRLS4,w_QRDRLS4x]=anguloQRDRLSB(cabeceo_gi,cabeceo_Ac,p,lam,delta);
p=25;
[alabeoQRDRLS5,w_QRDRLS5]=anguloQRDRLSB(alabeo_gi,alabeo_Ac,p,lam,delta);
[cabeceoQRDRLS5,w_QRDRLS5x]=anguloQRDRLSB(cabeceo_gi,cabeceo_Ac,p,lam,delta);
p=30;
[alabeoQRDRLS6,w_QRDRLS6]=anguloQRDRLSB(alabeo_gi,alabeo_Ac,p,lam,delta);
[cabeceoQRDRLS6,w_QRDRLS6x]=anguloQRDRLSB(cabeceo_gi,cabeceo_Ac,p,lam,delta);

%Representaciones gráficas
graficasRe
