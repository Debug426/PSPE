clc,clear
syms theta Tx Ty Tz
Person = pcread('3.pcd');
pos = Person.Location;

bowl=importdata('joint7.txt');
px=bowl(:,1);
py=bowl(:,2);
pz=bowl(:,3);
r = bowl(:,4);
g = bowl(:,5);
b = bowl(:,6);
D = [1;1;1;1;1;1;1;1;1;1;1;1;1;1];
p = [px,py,pz,D];

% Tx=-1.4;    %pos1
% Ty=-0.39;
% Tz=0.5;
Tx=-1.8;    %pos
Ty=0.45;
Tz=0.93;
alpha = -0.025*pi;
beta = 0;
theta=2*pi;
ptCloud_pos = pointCloud(pos);
RZ = [cos(theta) -sin(theta) 0;
    sin(theta) cos(theta) 0;
    0 0 1];
RY = [cos(beta) 0 sin(beta);
    0 1 0;
    -sin(beta) 0 cos(beta)];
RX = [1 0 0;
    0 cos(alpha) -sin(alpha);
    0 sin(alpha) cos(alpha)];
R = (RZ*RY*RX).';
ZERO = [0;0;0];
R_T = [R,ZERO];
t = [Tx Ty Tz 1];
T=[R_T;
    Tx Ty Tz 1];
tform_z = affine3d(T);
Pp = p*T;
% Pos_Trans_z = pctransform(ptCloud_pos, tform_z);


data=csvread('0001.csv',5,2)/100;
MarkerLabel=cell(39,2);
MarkerLabel{1,2}='LFHD';
MarkerLabel{2,2}='RFHD';
MarkerLabel{3,2}='LBHD';
MarkerLabel{4,2}='RBHD';
MarkerLabel{5,2}='C7';
MarkerLabel{6,2}='T10';
MarkerLabel{7,2}='CLAV';
MarkerLabel{8,2}='STRN';
MarkerLabel{9,2}='RBAK';
MarkerLabel{10,2}='LSHO';
MarkerLabel{11,2}='LUPA';
MarkerLabel{12,2}='LELB';
MarkerLabel{13,2}='LFRM';
MarkerLabel{14,2}='LWRA';
MarkerLabel{15,2}='LWRB';
MarkerLabel{16,2}='LFIN';
MarkerLabel{17,2}='RSHO';
MarkerLabel{18,2}='RUPA';
MarkerLabel{19,2}='RELB';
MarkerLabel{20,2}='RFRM';
MarkerLabel{21,2}='RWRA';
MarkerLabel{22,2}='RWRB';
MarkerLabel{23,2}='RFIN';
MarkerLabel{24,2}='LASI';
MarkerLabel{25,2}='RASI';
MarkerLabel{26,2}='LPSI';
MarkerLabel{27,2}='RPSI';
MarkerLabel{28,2}='LTHI';
MarkerLabel{29,2}='LKNE';
MarkerLabel{30,2}='LTIB';
MarkerLabel{31,2}='LANK';
MarkerLabel{32,2}='LHEE';
MarkerLabel{33,2}='LTOE';
MarkerLabel{34,2}='RTHI';
MarkerLabel{35,2}='RKNE';
MarkerLabel{36,2}='RTIB';
MarkerLabel{37,2}='RANK';
MarkerLabel{38,2}='RHEE';
MarkerLabel{39,2}='RTOE';
for i=1:39
    MarkerLabel{i,1}=data(:,(i-1)*3+1:i*3)/10;
end
new_point=cell(16,2);
new_point{1,2}='PELVIS';
new_point{2,2}='CHEST';
new_point{3,2}='MIDDLE';
new_point{4,2}='HEAD';
new_point{5,2}='LSHO';
new_point{6,2}='LELB';
new_point{7,2}='LWRB';
new_point{8,2}='RSHO';
new_point{9,2}='RELB';
new_point{10,2}='RWRB';
new_point{11,2}='LASI2LPSI';
new_point{12,2}='LKNE';
new_point{13,2}='LANK';
new_point{14,2}='RASI2RPSI';
new_point{15,2}='RKNE';
new_point{16,2}='RANK';
new_point{1,1}=(MarkerLabel{24,1}+MarkerLabel{27,1}+MarkerLabel{25,1}+MarkerLabel{26,1})/4;
new_point{2,1}=(MarkerLabel{5,1}+MarkerLabel{6,1}+MarkerLabel{7,1})/3;
new_point{3,1}=(new_point{1,1}+new_point{2,1})/2;
new_point{4,1}=(MarkerLabel{1,1}+MarkerLabel{2,1}+MarkerLabel{3,1}+MarkerLabel{4,1})/4;
new_point{5,1}=MarkerLabel{10,1};
new_point{6,1}=MarkerLabel{12,1};
new_point{7,1}=MarkerLabel{15,1};
new_point{8,1}=MarkerLabel{17,1};
new_point{9,1}=MarkerLabel{19,1};
new_point{10,1}=MarkerLabel{22,1};
new_point{11,1}=(MarkerLabel{24,1}+MarkerLabel{26,1})/2;
new_point{12,1}=MarkerLabel{29,1};
new_point{13,1}=MarkerLabel{31,1};
new_point{14,1}=(MarkerLabel{25,1}+MarkerLabel{27,1})/2;
new_point{15,1}=MarkerLabel{35,1};
new_point{16,1}=MarkerLabel{37,1};


%axis tight manual
close all
figure;
hold on; 
grid on;
% figure;
% pcshow(Pos_Trans_z.Location,[0,0,0],'MarkerSize', 50);

[x,y,z]=sphere;
r_origi=2.5/100;
r_others=2/100;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
new_point{2,1}=(new_point{5,1}+new_point{8,1})/2;

%transfer to original point and NED coordination
offset_x=mean(new_point{1,1}(1:1,1));
offset_y=mean(new_point{1,1}(1:1,2));
offset_z=mean(new_point{1,1}(1:1,3));
for i=1:16
    new_point{i,1}(:,1)=new_point{i,1}(:,1)-offset_x*ones(1,1);
    new_point{i,1}(:,2)=new_point{i,1}(:,2)-offset_y*ones(1,1);
    new_point{i,1}(:,2)=new_point{i,1}(:,2);
    new_point{i,1}(:,3)=new_point{i,1}(:,3);
end
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% other1=surf(x*r_origi+new_point{1,1}(1,1),y*r_origi+new_point{1,1}(1,2),z*r_origi+new_point{1,1}(1,3));
% set(other1,'FaceColor','red','EdgeColor','none');
% other2=surf(x*r_others+new_point{2,1}(1,1),y*r_others+new_point{2,1}(1,2),z*r_others+new_point{2,1}(1,3));
% set(other2,'FaceColor','blue','EdgeColor','none');
% other3=surf(x*r_others+new_point{3,1}(1,1),y*r_others+new_point{3,1}(1,2),z*r_others+new_point{3,1}(1,3));
% set(other3,'FaceColor','red','EdgeColor','none');
% other4=surf(x*r_others+new_point{4,1}(1,1),y*r_others+new_point{4,1}(1,2),z*r_others+new_point{4,1}(1,3));
% set(other4,'FaceColor','blue','EdgeColor','none');
% other5=surf(x*r_others+new_point{5,1}(1,1),y*r_others+new_point{5,1}(1,2),z*r_others+new_point{5,1}(1,3));
% set(other5,'FaceColor','red','EdgeColor','none');
% other6=surf(x*r_others+new_point{6,1}(1,1),y*r_others+new_point{6,1}(1,2),z*r_others+new_point{6,1}(1,3));
% set(other6,'FaceColor','red','EdgeColor','none');
% other7=surf(x*r_others+new_point{7,1}(1,1),y*r_others+new_point{7,1}(1,2),z*r_others+new_point{7,1}(1,3));
% set(other7,'FaceColor','red','EdgeColor','none');
% other8=surf(x*r_others+new_point{8,1}(1,1),y*r_others+new_point{8,1}(1,2),z*r_others+new_point{8,1}(1,3));
% set(other8,'FaceColor','green','EdgeColor','none');
% other9=surf(x*r_others+new_point{9,1}(1,1),y*r_others+new_point{9,1}(1,2),z*r_others+new_point{9,1}(1,3));
% set(other9,'FaceColor','green','EdgeColor','none');
% other10=surf(x*r_others+new_point{10,1}(1,1),y*r_others+new_point{10,1}(1,2),z*r_others+new_point{10,1}(1,3));
% set(other10,'FaceColor','green','EdgeColor','none');
% other11=surf(x*r_others+new_point{11,1}(1,1),y*r_others+new_point{11,1}(1,2),z*r_others+new_point{11,1}(1,3));
% set(other11,'FaceColor','red','EdgeColor','none');
% other12=surf(x*r_others+new_point{12,1}(1,1),y*r_others+new_point{12,1}(1,2),z*r_others+new_point{12,1}(1,3));
% set(other12,'FaceColor','red','EdgeColor','none');
% other13=surf(x*r_others+new_point{13,1}(1,1),y*r_others+new_point{13,1}(1,2),z*r_others+new_point{13,1}(1,3));
% set(other13,'FaceColor','red','EdgeColor','none');
% other14=surf(x*r_others+new_point{14,1}(1,1),y*r_others+new_point{14,1}(1,2),z*r_others+new_point{14,1}(1,3));
% set(other14,'FaceColor','green','EdgeColor','none');
% other15=surf(x*r_others+new_point{15,1}(1,1),y*r_others+new_point{15,1}(1,2),z*r_others+new_point{15,1}(1,3));
% set(other15,'FaceColor','green','EdgeColor','none');
% other16=surf(x*r_others+new_point{16,1}(1,1),y*r_others+new_point{16,1}(1,2),z*r_others+new_point{16,1}(1,3));
% set(other16,'FaceColor','green','EdgeColor','none');
% shading flat
% 
% head1(:,1)=new_point{2,1}(1,:)';
% head1(:,2)=new_point{4,1}(1,:)';
% h1=plot3(head1(1,:),head1(2,:),head1(3,:),'b','LineWidth',2);
% 
% chest1(:,1)=new_point{2,1}(1,:)';
% chest1(:,2)=new_point{5,1}(1,:)';
% chest2(:,1)=new_point{2,1}(1,:)';
% chest2(:,2)=new_point{8,1}(1,:)';
% chest3(:,1)=new_point{2,1}(1,:)';
% chest3(:,2)=new_point{3,1}(1,:)';
% c1=plot3(chest1(1,:),chest1(2,:),chest1(3,:),'b','LineWidth',2);
% c2=plot3(chest2(1,:),chest2(2,:),chest2(3,:),'b','LineWidth',2);
% c3=plot3(chest3(1,:),chest3(2,:),chest3(3,:),'b','LineWidth',2);
% 
% waist1(:,1)=new_point{1,1}(1,:)';
% waist1(:,2)=new_point{3,1}(1,:)';
% waist2(:,1)=new_point{1,1}(1,:)';
% waist2(:,2)=new_point{11,1}(1,:)';
% waist3(:,1)=new_point{1,1}(1,:)';
% waist3(:,2)=new_point{14,1}(1,:)';
% w1=plot3(waist1(1,:),waist1(2,:),waist1(3,:),'k','LineWidth',2);
% w2=plot3(waist2(1,:),waist2(2,:),waist2(3,:),'k','LineWidth',2);
% w3=plot3(waist3(1,:),waist3(2,:),waist3(3,:),'k','LineWidth',2);
% 
% leftarm1(:,1)=new_point{5,1}(1,:)';
% leftarm1(:,2)=new_point{6,1}(1,:)';
% leftarm2(:,1)=new_point{6,1}(1,:)';
% leftarm2(:,2)=new_point{7,1}(1,:)';
% la1=plot3(leftarm1(1,:),leftarm1(2,:),leftarm1(3,:),'r','LineWidth',2);
% la2=plot3(leftarm2(1,:),leftarm2(2,:),leftarm2(3,:),'r','LineWidth',2);
% 
% rightarm1(:,1)=new_point{8,1}(1,:)';
% rightarm1(:,2)=new_point{9,1}(1,:)';
% rightarm2(:,1)=new_point{9,1}(1,:)';
% rightarm2(:,2)=new_point{10,1}(1,:)';
% ra1=plot3(rightarm1(1,:),rightarm1(2,:),rightarm1(3,:),'g','LineWidth',2);
% ra2=plot3(rightarm2(1,:),rightarm2(2,:),rightarm2(3,:),'g','LineWidth',2);
% 
% leftleg1(:,1)=new_point{11,1}(1,:)';
% leftleg1(:,2)=new_point{12,1}(1,:)';
% leftleg2(:,1)=new_point{12,1}(1,:)';
% leftleg2(:,2)=new_point{13,1}(1,:)';
% ll1=plot3(leftleg1(1,:),leftleg1(2,:),leftleg1(3,:),'r','LineWidth',2);
% ll2=plot3(leftleg2(1,:),leftleg2(2,:),leftleg2(3,:),'r','LineWidth',2);
% 
% rightleg1(:,1)=new_point{14,1}(1,:)';
% rightleg1(:,2)=new_point{15,1}(1,:)';
% rightleg2(:,1)=new_point{15,1}(1,:)';
% rightleg2(:,2)=new_point{16,1}(1,:)';
% rl1=plot3(rightleg1(1,:),rightleg1(2,:),rightleg1(3,:),'g','LineWidth',2);
% rl2=plot3(rightleg2(1,:),rightleg2(2,:),rightleg2(3,:),'g','LineWidth',2);
% 
% axis equal;
% grid on
%  j = 1;
%     delete(other1);
%     other1=surf(x*r_others+new_point{1,1}(j,1),y*r_others+new_point{1,1}(j,2),z*r_others+new_point{1,1}(j,3));
%     set(other1,'FaceColor','red','EdgeColor','none');
%     delete(other2);
%     other2=surf(x*r_others+new_point{2,1}(j,1),y*r_others+new_point{2,1}(j,2),z*r_others+new_point{2,1}(j,3));
%     set(other2,'FaceColor','blue','EdgeColor','none');
%     delete(other3);
%     other3=surf(x*r_others+new_point{3,1}(j,1),y*r_others+new_point{3,1}(j,2),z*r_others+new_point{3,1}(j,3));
%     set(other3,'FaceColor','red','EdgeColor','none');
%     delete(other4);
%     other4=surf(x*r_others+new_point{4,1}(j,1),y*r_others+new_point{4,1}(j,2),z*r_others+new_point{4,1}(j,3));
%     set(other4,'FaceColor','blue','EdgeColor','none');
%     delete(other5);
%     other5=surf(x*r_others+new_point{5,1}(j,1),y*r_others+new_point{5,1}(j,2),z*r_others+new_point{5,1}(j,3));
%     set(other5,'FaceColor','red','EdgeColor','none');
%     delete(other6);
%     other6=surf(x*r_others+new_point{6,1}(j,1),y*r_others+new_point{6,1}(j,2),z*r_others+new_point{6,1}(j,3));
%     set(other6,'FaceColor','red','EdgeColor','none');
%     delete(other7);
%     other7=surf(x*r_others+new_point{7,1}(j,1),y*r_others+new_point{7,1}(j,2),z*r_others+new_point{7,1}(j,3));
%     set(other7,'FaceColor','red','EdgeColor','none');
%     delete(other8);
%     other8=surf(x*r_others+new_point{8,1}(j,1),y*r_others+new_point{8,1}(j,2),z*r_others+new_point{8,1}(j,3));
%     set(other8,'FaceColor','green','EdgeColor','none');
%     delete(other9);
%     other9=surf(x*r_others+new_point{9,1}(j,1),y*r_others+new_point{9,1}(j,2),z*r_others+new_point{9,1}(j,3));
%     set(other9,'FaceColor','green','EdgeColor','none');
%     delete(other10);
%     other10=surf(x*r_others+new_point{10,1}(j,1),y*r_others+new_point{10,1}(j,2),z*r_others+new_point{10,1}(j,3));
%     set(other10,'FaceColor','green','EdgeColor','none');
%     delete(other11);
%     other11=surf(x*r_others+new_point{11,1}(j,1),y*r_others+new_point{11,1}(j,2),z*r_others+new_point{11,1}(j,3));
%     set(other11,'FaceColor','red','EdgeColor','none');
%     delete(other12);
%     other12=surf(x*r_others+new_point{12,1}(j,1),y*r_others+new_point{12,1}(j,2),z*r_others+new_point{12,1}(j,3));
%     set(other12,'FaceColor','red','EdgeColor','none');
%     delete(other13);
%     other13=surf(x*r_others+new_point{13,1}(j,1),y*r_others+new_point{13,1}(j,2),z*r_others+new_point{13,1}(j,3));
%     set(other13,'FaceColor','red','EdgeColor','none');
%     delete(other14);
%     other14=surf(x*r_others+new_point{14,1}(j,1),y*r_others+new_point{14,1}(j,2),z*r_others+new_point{14,1}(j,3));
%     set(other14,'FaceColor','green','EdgeColor','none');
%     delete(other15);
%     other15=surf(x*r_others+new_point{15,1}(j,1),y*r_others+new_point{15,1}(j,2),z*r_others+new_point{15,1}(j,3));
%     set(other15,'FaceColor','green','EdgeColor','none');
%     delete(other16);
%     other16=surf(x*r_others+new_point{16,1}(j,1),y*r_others+new_point{16,1}(j,2),z*r_others+new_point{16,1}(j,3));
%     set(other16,'FaceColor','green','EdgeColor','none');
%         shading flat
%     
%     head1(:,1)=new_point{2,1}(j,:)';
%     head1(:,2)=new_point{4,1}(j,:)';
%     delete(h1);
%     h1=plot3(head1(1,:),head1(2,:),head1(3,:),'b','LineWidth',2);
%     
%     chest1(:,1)=new_point{2,1}(j,:)';
%     chest1(:,2)=new_point{5,1}(j,:)';
%     chest2(:,1)=new_point{2,1}(j,:)';
%     chest2(:,2)=new_point{8,1}(j,:)';
%     chest3(:,1)=new_point{2,1}(j,:)';
%     chest3(:,2)=new_point{3,1}(j,:)';
%     delete(c1);
%     c1=plot3(chest1(1,:),chest1(2,:),chest1(3,:),'b','LineWidth',2);
%     delete(c2);
%     c2=plot3(chest2(1,:),chest2(2,:),chest2(3,:),'b','LineWidth',2);
%     delete(c3);
%     c3=plot3(chest3(1,:),chest3(2,:),chest3(3,:),'b','LineWidth',2);
%     
%     waist1(:,1)=new_point{1,1}(j,:)';
%     waist1(:,2)=new_point{3,1}(j,:)';
%     waist2(:,1)=new_point{1,1}(j,:)';
%     waist2(:,2)=new_point{11,1}(j,:)';
%     waist3(:,1)=new_point{1,1}(j,:)';
%     waist3(:,2)=new_point{14,1}(j,:)';
%     delete(w1);
%     w1=plot3(waist1(1,:),waist1(2,:),waist1(3,:),'k','LineWidth',2);
%     delete(w2);
%     w2=plot3(waist2(1,:),waist2(2,:),waist2(3,:),'k','LineWidth',2);
%     delete(w3);
%     w3=plot3(waist3(1,:),waist3(2,:),waist3(3,:),'k','LineWidth',2);
%     
%     leftarm1(:,1)=new_point{5,1}(j,:)';
%     leftarm1(:,2)=new_point{6,1}(j,:)';
%     leftarm2(:,1)=new_point{6,1}(j,:)';
%     leftarm2(:,2)=new_point{7,1}(j,:)';
%     delete(la1);
%     la1=plot3(leftarm1(1,:),leftarm1(2,:),leftarm1(3,:),'r','LineWidth',2);
%     delete(la2);
%     la2=plot3(leftarm2(1,:),leftarm2(2,:),leftarm2(3,:),'r','LineWidth',2);
%     
%     rightarm1(:,1)=new_point{8,1}(j,:)';
%     rightarm1(:,2)=new_point{9,1}(j,:)';
%     rightarm2(:,1)=new_point{9,1}(j,:)';
%     rightarm2(:,2)=new_point{10,1}(j,:)';
%     delete(ra1);
%     ra1=plot3(rightarm1(1,:),rightarm1(2,:),rightarm1(3,:),'g','LineWidth',2);
%     delete(ra2);
%     ra2=plot3(rightarm2(1,:),rightarm2(2,:),rightarm2(3,:),'g','LineWidth',2);
%     
%     leftleg1(:,1)=new_point{11,1}(j,:)';
%     leftleg1(:,2)=new_point{12,1}(j,:)';
%     leftleg2(:,1)=new_point{12,1}(j,:)';
%     leftleg2(:,2)=new_point{13,1}(j,:)';
%     delete(ll1);
%     ll1=plot3(leftleg1(1,:),leftleg1(2,:),leftleg1(3,:),'r','LineWidth',2);
%     delete(ll2);
%     ll2=plot3(leftleg2(1,:),leftleg2(2,:),leftleg2(3,:),'r','LineWidth',2);
%     
%     rightleg1(:,1)=new_point{14,1}(j,:)';
%     rightleg1(:,2)=new_point{15,1}(j,:)';
%     rightleg2(:,1)=new_point{15,1}(j,:)';
%     rightleg2(:,2)=new_point{16,1}(j,:)';
%     delete(rl1);
%     rl1=plot3(rightleg1(1,:),rightleg1(2,:),rightleg1(3,:),'g','LineWidth',2);
%     delete(rl2);
%     rl2=plot3(rightleg2(1,:),rightleg2(2,:),rightleg2(3,:),'g','LineWidth',2);

data = readmatrix("denghuanyu Cal 02999.csv");
% Head = data(39,18:20);
% LSHO = data(51,30:32);
% RSHO = data(51,51:53);
% Clav = (LSHO + RSHO)/2;
% % LEJC = data(39,39:41);
% % REJC = data(39,)
% LELB = data(51,36:38);
% RELB = data(51,57:59);
% LFIN = data(51,48:50);
% RFIN = data(51,69:71);
% ThoraxCOM = data(39,264:266);
% LHJC = data(39,66:68);
% RHJC = data(39,210:212);
% PEL = (LHJC + RHJC)/2;
% LKJC = data(39,93:95);
% RKJC = data(39,219:221);
% LAJC = data(39,21:23);
% RAJC = data(39,180:182);

data_shape = size(data);
data_shape = size(data);
data_row = data_shape(1,2);
data_line = data_shape(1,1);
switch(data_line)
    case 42
        line1 = 30;
        line2 = 42;
        case 51
        line1 = 39;
        line2 = 51;    
end
switch(data_row)
    case 1029
Head = data(line1,18:20);
LSHO = data(line2,30:32);
RSHO = data(line2,51:53);
Clav = (LSHO + RSHO)/2;
% LEJC = data(39,39:41);
% REJC = data(39,)
LELB = data(line2,36:38);
RELB = data(line2,57:59);
LFIN = data(line2,48:50);
RFIN = data(line2,69:71);
ThoraxCOM = data(line1,273:275);
LHJC = data(line1,66:68);
RHJC = data(line1,213:215);
PEL = (LHJC + RHJC)/2;
LKJC = data(line1,93:95);
RKJC = data(line1,222:224);
LAJC = data(line1,21:23);
RAJC = data(line1,180:182);
    case 1272
Head = data(line1,18:20);
LSHO = data(line2,30:32);
RSHO = data(line2,51:53);
Clav = (LSHO + RSHO)/2;
% LEJC = data(39,39:41);
% REJC = data(39,)
LELB = data(line2,36:38);
RELB = data(line2,57:59);
LFIN = data(line2,48:50);
RFIN = data(line2,69:71);
ThoraxCOM = data(line1,336:338);
LHJC = data(line1,66:68);
RHJC = data(line1,225:227);
PEL = (LHJC + RHJC)/2;
LKJC = data(line1,93:95);
RKJC = data(line1,252:254);
LAJC = data(line1,21:23);
RAJC = data(line1,180:182);
    case 1002
Head = data(line1,18:20);
LSHO = data(line2,30:32);
RSHO = data(line2,51:53);
Clav = (LSHO + RSHO)/2;
% LEJC = data(39,39:41);
% REJC = data(39,)
LELB = data(line2,36:38);
RELB = data(line2,57:59);
LFIN = data(line2,48:50);
RFIN = data(line2,69:71);
ThoraxCOM = data(line1,264:266);
LHJC = data(line1,66:68);
RHJC = data(line1,210:212);
PEL = (LHJC + RHJC)/2;
LKJC = data(line1,93:95);
RKJC = data(line1,219:221);
LAJC = data(line1,21:23);
RAJC = data(line1,180:182);
end

X = [Head(1,1),LSHO(1,1),RSHO(1,1),Clav(1,1),LELB(1,1),RELB(1,1),LFIN(1,1),RFIN(1,1),ThoraxCOM(1,1),LHJC(1,1),RHJC(1,1),PEL(1,1),LKJC(1,1),RKJC(1,1),LAJC(1,1),RAJC(1,1)];
Y = [Head(1,2),LSHO(1,2),RSHO(1,2),Clav(1,2),LELB(1,2),RELB(1,2),LFIN(1,2),RFIN(1,2),ThoraxCOM(1,2),LHJC(1,2),RHJC(1,2),PEL(1,2),LKJC(1,2),RKJC(1,2),LAJC(1,2),RAJC(1,2)];
Z = [Head(1,3),LSHO(1,3),RSHO(1,3),Clav(1,3),LELB(1,3),RELB(1,3),LFIN(1,3),RFIN(1,3),ThoraxCOM(1,3),LHJC(1,3),RHJC(1,3),PEL(1,3),LKJC(1,3),RKJC(1,3),LAJC(1,3),RAJC(1,3)];

% h2 = scatter3(X/1000,Y/1000,Z/1000,20,"red","filled");
% hold on 
% plot3([Head(1,1)/1000,Clav(1,1)/1000],[Head(1,2)/1000,Clav(1,2)/1000],[Head(1,3)/1000,Clav(1,3)/1000],'g','LineWidth',4);
% plot3([LSHO(1,1)/1000,Clav(1,1)/1000],[LSHO(1,2)/1000,Clav(1,2)/1000],[LSHO(1,3)/1000,Clav(1,3)/1000],'g','LineWidth',4);
% plot3([RSHO(1,1)/1000,Clav(1,1)/1000],[RSHO(1,2)/1000,Clav(1,2)/1000],[RSHO(1,3)/1000,Clav(1,3)/1000],'g','LineWidth',4);
% plot3([LSHO(1,1)/1000,LELB(1,1)/1000],[LSHO(1,2)/1000,LELB(1,2)/1000],[LSHO(1,3)/1000,LELB(1,3)/1000],'g','LineWidth',4);
% plot3([RSHO(1,1)/1000,RELB(1,1)/1000],[RSHO(1,2)/1000,RELB(1,2)/1000],[RSHO(1,3)/1000,RELB(1,3)/1000],'g','LineWidth',4);
% plot3([LFIN(1,1)/1000,LELB(1,1)/1000],[LFIN(1,2)/1000,LELB(1,2)/1000],[LFIN(1,3)/1000,LELB(1,3)/1000],'g','LineWidth',4);
% plot3([RFIN(1,1)/1000,RELB(1,1)/1000],[RFIN(1,2)/1000,RELB(1,2)/1000],[RFIN(1,3)/1000,RELB(1,3)/1000],'g','LineWidth',4);
% plot3([RHJC(1,1)/1000,LHJC(1,1)/1000],[RHJC(1,2)/1000,LHJC(1,2)/1000],[RHJC(1,3)/1000,LHJC(1,3)/1000],'g','LineWidth',4);
% plot3([RHJC(1,1)/1000,RKJC(1,1)/1000],[RHJC(1,2)/1000,RKJC(1,2)/1000],[RHJC(1,3)/1000,RKJC(1,3)/1000],'g','LineWidth',4);
% plot3([LKJC(1,1)/1000,LHJC(1,1)/1000],[LKJC(1,2)/1000,LHJC(1,2)/1000],[LKJC(1,3)/1000,LHJC(1,3)/1000],'g','LineWidth',4);
% plot3([RAJC(1,1)/1000,RKJC(1,1)/1000],[RAJC(1,2)/1000,RKJC(1,2)/1000],[RAJC(1,3)/1000,RKJC(1,3)/1000],'g','LineWidth',4);
% plot3([LKJC(1,1)/1000,LAJC(1,1)/1000],[LKJC(1,2)/1000,LAJC(1,2)/1000],[LKJC(1,3)/1000,LAJC(1,3)/1000],'g','LineWidth',4);
% plot3([ThoraxCOM(1,1)/1000,Clav(1,1)/1000],[ThoraxCOM(1,2)/1000,Clav(1,2)/1000],[ThoraxCOM(1,3)/1000,Clav(1,3)/1000],'g','LineWidth',4);
% 

pp = scatter3(Pp(:,1),Pp(:,2),Pp(:,3),50,"blue","filled");
plot3([Pp(1,1),Pp(2,1)],[Pp(1,2),Pp(2,2)],[Pp(1,3),Pp(2,3)],'r','LineWidth',4);
plot3([Pp(2,1),Pp(3,1)],[Pp(2,2),Pp(3,2)],[Pp(2,3),Pp(3,3)],'r','LineWidth',4);
plot3([Pp(2,1),Pp(4,1)],[Pp(2,2),Pp(4,2)],[Pp(2,3),Pp(4,3)],'r','LineWidth',4);
plot3([Pp(4,1),Pp(5,1)],[Pp(4,2),Pp(5,2)],[Pp(4,3),Pp(5,3)],'r','LineWidth',4);
plot3([Pp(3,1),Pp(6,1)],[Pp(3,2),Pp(6,2)],[Pp(3,3),Pp(6,3)],'r','LineWidth',4);
plot3([Pp(7,1),Pp(8,1)],[Pp(7,2),Pp(8,2)],[Pp(7,3),Pp(8,3)],'r','LineWidth',4);
plot3([Pp(7,1),Pp(9,1)],[Pp(7,2),Pp(9,2)],[Pp(7,3),Pp(9,3)],'r','LineWidth',4);
plot3([Pp(8,1),Pp(10,1)],[Pp(8,2),Pp(10,2)],[Pp(8,3),Pp(10,3)],'r','LineWidth',4);
plot3([Pp(6,1),Pp(11,1)],[Pp(6,2),Pp(11,2)],[Pp(6,3),Pp(11,3)],'r','LineWidth',4);
plot3([Pp(5,1),Pp(12,1)],[Pp(5,2),Pp(12,2)],[Pp(5,3),Pp(12,3)],'r','LineWidth',4);
plot3([Pp(13,1),Pp(9,1)],[Pp(13,2),Pp(9,2)],[Pp(13,3),Pp(9,3)],'r','LineWidth',4);
plot3([Pp(14,1),Pp(10,1)],[Pp(14,2),Pp(10,2)],[Pp(14,3),Pp(10,3)],'r','LineWidth',4);




xlabel('x(m)');
ylabel('y(m)');
zlabel('z(m)');
axis equal;
axis auto;
axis off;
grid on
% title('旋转后');
% xlabel('x(m)');
% ylabel('y(m)');
% zlabel('z(m)');
% axis equal;
% grid on



