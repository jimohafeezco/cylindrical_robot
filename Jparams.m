% Author: Popov Dmitry, Berezhnoy Vladislav
% Innopolis University
% Industrial Robotics
% Homework 2

function out = Jparams(q,params,Tbase,Ttool)

% L1 = params(1); 
L2 = params(2); L3=params(3); L4=params(4);
 
drx = dRx(0); dry = dRy(0); drz = dRz(0);
dtx = dTx(0); dty = dTy(0); dtz = dTz(0);

T_rot = Tx(-q(3))*Tz(-q(2))*Rz(-q(1));

Td = Tbase * Rz(q(1)) * drz * Tz(L2) * Tz(q(2)) * Tx(L3) * Tx(q(3)) * Tx(L4) * Ttool * T_rot;

J1= [Td(1,4), Td(2,4), Td(3,4)]' ;

Td = Tbase * Rz(q(1)) * Tz(L2) *dtx* Tz(q(2)) * Tx(L3) * Tx(q(3)) * Tx(L4) * Ttool * T_rot;

J2= [Td(1,4), Td(2,4), Td(3,4)]' ;

Td = Tbase * Rz(q(1)) * Tz(L2) *dty* Tz(q(2)) * Tx(L3) * Tx(q(3)) * Tx(L4) * Ttool * T_rot;

J3= [Td(1,4), Td(2,4), Td(3,4)]' ;

Td = Tbase * Rz(q(1)) * Tz(L2) *dtz* Tz(q(2)) * Tx(L3) * Tx(q(3)) * Tx(L4) * Ttool * T_rot;

J4= [Td(1,4), Td(2,4), Td(3,4)]' ;

Td = Tbase * Rz(q(1)) * Tz(L2) *drx* Tz(q(2)) * Tx(L3) * Tx(q(3)) * Tx(L4) * Ttool * T_rot;

J5= [Td(1,4), Td(2,4), Td(3,4)]' ;

Td = Tbase * Rz(q(1)) * Tz(L2) *dry* Tz(q(2)) * Tx(L3) * Tx(q(3)) * Tx(L4) * Ttool * T_rot;

J6= [Td(1,4), Td(2,4), Td(3,4)]' ;

Td = Tbase * Rz(q(1)) * Tz(L2) *drz* Tz(q(2)) * Tx(L3) * Tx(q(3)) * Tx(L4) * Ttool * T_rot;

J7= [Td(1,4), Td(2,4), Td(3,4)]' ;

Td = Tbase * Rz(q(1)) * Tz(L2) * Tz(q(2)) *dtz* Tx(L3) * Tx(q(3)) * Tx(L4) * Ttool * T_rot;

J8= [Td(1,4), Td(2,4), Td(3,4)]' ;

Td = Tbase * Rz(q(1)) * Tz(L2) * Tz(q(2)) * Tx(L3) * dtx* Tx(q(3)) * Tx(L4) * Ttool * T_rot;

J9= [Td(1,4), Td(2,4), Td(3,4)]' ;

Td = Tbase * Rz(q(1)) * Tz(L2) * Tz(q(2)) * Tx(L3) * dty* Tx(q(3)) * Tx(L4) * Ttool * T_rot;

J10= [Td(1,4), Td(2,4), Td(3,4)]' ;

Td = Tbase * Rz(q(1)) * Tz(L2) * Tz(q(2)) * Tx(L3) * dtz* Tx(q(3)) * Tx(L4) * Ttool * T_rot;

J11= [Td(1,4), Td(2,4), Td(3,4)]' ;

Td = Tbase * Rz(q(1)) * Tz(L2) * Tz(q(2)) * Tx(L3) * drx* Tx(q(3)) * Tx(L4) * Ttool * T_rot;

J12= [Td(1,4), Td(2,4), Td(3,4)]' ;

Td = Tbase * Rz(q(1)) * Tz(L2) * Tz(q(2)) * Tx(L3) * dry* Tx(q(3)) * Tx(L4) * Ttool * T_rot;

J13= [Td(1,4), Td(2,4), Td(3,4)]' ;

Td = Tbase * Rz(q(1)) * Tz(L2) * Tz(q(2)) * Tx(L3) * drz* Tx(q(3)) * Tx(L4) * Ttool * T_rot;

J14= [Td(1,4), Td(2,4), Td(3,4)]' ;

Td = Tbase * Rz(q(1)) * Tz(L2) * Tz(q(2)) * Tx(L3) * Tx(q(3)) * dtx* Tx(L4) * Ttool * T_rot;

J15= [Td(1,4), Td(2,4), Td(3,4)]' ;

Td = Tbase * Rz(q(1)) * Tz(L2) * Tz(q(2)) * Tx(L3) * Tx(q(3)) * Tx(L4)* dtx * Ttool * T_rot;

J16= [Td(1,4), Td(2,4), Td(3,4)]' ;

Td = Tbase * Rz(q(1)) * Tz(L2) * Tz(q(2)) * Tx(L3) * Tx(q(3)) *Tx(L4)* dty * Ttool * T_rot;

J17= [Td(1,4), Td(2,4), Td(3,4)]' ;

Td = Tbase * Rz(q(1)) * Tz(L2) * Tz(q(2)) * Tx(L3) * Tx(q(3)) *Tx(L4)* dtz * Ttool * T_rot;

J18= [Td(1,4), Td(2,4), Td(3,4)]' ;

Td = Tbase * Rz(q(1)) * Tz(L2) * Tz(q(2)) * Tx(L3) * Tx(q(3)) *Tx(L4)* drx * Ttool * T_rot;

J19= [Td(1,4), Td(2,4), Td(3,4)]' ;

Td = Tbase * Rz(q(1)) * Tz(L2) * Tz(q(2)) * Tx(L3) * Tx(q(3)) *Tx(L4)* dry * Ttool * T_rot;

J20= [Td(1,4), Td(2,4), Td(3,4)]' ;

Td = Tbase * Rz(q(1)) * Tz(L2) * Tz(q(2)) * Tx(L3) * Tx(q(3)) *Tx(L4)* drz * Ttool * T_rot;

J21= [Td(1,4), Td(2,4), Td(3,4)]' ;

out = [J1 J2 J3 J4 J5 J6 J7 J8 J9 J10 J11 J12 J13 J14 J15 J16 J17 J18 J19 J20 J21];

end