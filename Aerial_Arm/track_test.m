% track
pdes=[0 0 0;0 0 4;4 0 4;4 4 4];
pointnum=length(pdes)-2;
final = pdes(length(pdes),:);
xdes = final(1); ydes = final(2); zdes = final(3);
vdes = 2;
psides = 0;

mb=1.8;
mL1=0.08;
mL2=0.08;
L=0.3;
                        
mq = mb;
mg = mL1+mL2;
mobj = 0; 
ms = mq+mg+mobj;
            
Ixx=0.342;
Iyy=0.342;
Izz=0.474;
            
Kf = 6.11e-8*3600/4/pi^2;
Km = 1.5e-9*3600/4/pi^2; 
g = 9.8; 
km=20;
            
kctp=5;
kcti=3e-4;
kctd=1;
katp=5;
kati=3e-4;

kpph=2000;
kdph=4000;
kpt=2000;
kdt=4000;
kpp=800;
kdp=4000;

kpx=1;
kix=0;
kdx=0.8;
kpy=1;
kiy=0;
kdy=0.8;
kpz=1.8;
kiz=0;
kdz=1.5;
            
wh=sqrt(ms*g/4/Kf);