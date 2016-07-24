%%%%%%%%%%%%%% Modell der Realität 
m_real=3;%Masse
g=9.81;%Erdbeschleunigung
c_w=0.1;%Luftwiderstandsbeiwert

%%%%%%%%%%%% Regler Bestimmen
% Masse in Kg für die konstante Kraft des Reglers zur Überwindung der
% Gewichtskrafts
m_regler=1.1*m_real;
% Lineare Zustandsrückführung zum Modellstabilisierung
r=acker([0 1;0 0],[0; 1/m_real],[-3 -2]);
r1=r(1);
r2=r(2);
% Lineare Vorsteuerung für Stationäre Genauigkeit
mx_u=[0 1 0;0 0 1/m_real;1 0 0]\[0; 0; 1];
mx1=mx_u(1);
% Integrator Anteil für Stationäre Genauigkeit
rI=0.5;
%Stellgrößebeschränkung in N
schub_max=m_regler*g*1.5;
schub_min=m_regler*g*0.5;


%%%%%%%% Beobachter bestimmen 
k=acker([0 1;0 0]',[1;0],[-5 -5]);
k1=k(1);
k2=k(2);