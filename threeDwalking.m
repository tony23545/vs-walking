function threeDwalking()
%UNTITLED �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
T=0.8;
Dtime=0.01;
L1=1;    %����һ�ؽ�
L2=1;
D=0.2; %�����Ŀ��(һ��)

%���ùؼ�����
%sagittal
phi0=35;
alpha0=15;
beta0=30;
%Lateral
bigphi0=0;
gama0=5;

sphi1=1-cosd(alpha0)*cosd(phi0);
cphi1=cosd(alpha0)*sind(phi0);
phi1=atan2(sphi1,cphi1)/pi*180;
t1=0:Dtime:T/2;
t2=T/2+Dtime:Dtime:T;

step=1;%ָʾ�ڼ���
note=0;%ָʾ���ҽ��ĸ��ŵ�,0��ʾ�ҽţ���ɫ����
i=1;
stance=[0;D;0];%֧�ŵ�����

%����ؽڽǣ������ظ����ã���һ������
for n=t1
    joint(i).Shippitch1=(phi0-phi1)/2*(1+cos(2*pi*n/T))+alpha0;
    joint(i).Shippitch2=-phi1/2*(1+cos(2*pi*n/T))+beta0/2*(1-cos(2*pi*n/T));
    joint(i).Sknee1=2*alpha0;
    joint(i).Sknee2=beta0*(1-cos(2*pi*n/T));
    joint(i).Shiproll1=bigphi0-gama0*sin(pi*n/T);
    joint(i).Sankleroll1=joint(i).Shiproll1;
    joint(i).Shiproll2=bigphi0+gama0*sin(pi*n/T);
    joint(i).Sankleroll2=joint(i).Shiproll2;
    i=i+1;
end
for n=t2
    joint(i).Shippitch1=-phi1/2*(1+cos(2*pi*n/T))+alpha0/2*(1-cos(2*pi*n/T));
    joint(i).Shippitch2=(phi0-phi1)/2*(1+cos(2*pi*n/T))+(alpha0+beta0)/2+(alpha0-beta0)/2*cos(2*pi*n/T);
    joint(i).Sknee1=alpha0*(1-cos(2*pi*n/T));
    joint(i).Sknee2=2*alpha0+(beta0-alpha0)*(1-cos(2*pi*n/T));
    joint(i).Shiproll1=bigphi0-gama0*sin(pi*n/T);
    joint(i).Sankleroll1=joint(i).Shiproll1;
    joint(i).Shiproll2=bigphi0+gama0*sin(pi*n/T);
    joint(i).Sankleroll2=joint(i).Shiproll2;
    i=i+1;
end

%����ؽ�λ��
%�ٶ���һ�����ҽ��ŵأ���ص��ڣ�0��0��0������������x������
%���ڲ�̬�ĶԳƺ��ظ�����Щ�ؽ�λ��Ҳ�ǿ����ظ����õ�
%���ҽ�Ϊ֧����Ϊ��
for i=1:length(t1)+length(t2)
    gait(i).foot1=stance;
    %�����ҽ�ϥ��λ��
    knee1pitch=joint(i).Sknee1-joint(i).Shippitch1;
    RY=[cosd(knee1pitch) 0 sind(knee1pitch);0 1 0;-sind(knee1pitch) 0 cosd(knee1pitch)];
    RX=[1 0 0;0 cosd(joint(i).Sankleroll1) -sind(joint(i).Sankleroll1);0 sind(joint(i).Sankleroll1) cosd(joint(i).Sankleroll1)];
    gait(i).knee1=RY*RX*[0;0;L1]+gait(i).foot1;
    %�����ұ��β���λ��
    RY=[cosd(-joint(i).Shippitch1) 0 sind(-joint(i).Shippitch1);0 1 0;-sind(-joint(i).Shippitch1) 0 cosd(-joint(i).Shippitch1)];
    gait(i).hip1=RY*RX*[0;0;L2]+gait(i).knee1;
    %����β�λ��
    gait(i).hip2=gait(i).hip1-[0;2*D;0];
    %���ϥ��
    RY=[cosd(-joint(i).Shippitch2) 0 sind(-joint(i).Shippitch2);0 1 0;-sind(-joint(i).Shippitch2) 0 cosd(-joint(i).Shippitch2)];
    RX=[1 0 0;0 cosd(joint(i).Sankleroll2) -sind(joint(i).Sankleroll2);0 sind(joint(i).Sankleroll2) cosd(joint(i).Sankleroll2)];
    gait(i).knee2=RY*RX*[0;0;-L2]+gait(i).hip2;
    %���
    knee2pitch=joint(i).Sknee2-joint(i).Shippitch2;
    RY=[cosd(knee2pitch) 0 sind(knee2pitch);0 1 0;-sind(knee2pitch) 0 cosd(knee2pitch)];
    gait(i).foot2=RY*RX*[0;0;-L1]+gait(i).knee2;
end
steplength=gait(i).foot2(1);

%������ĸ߶�
for i=1:length(t1)+length(t2)
    heigh(i)=gait(i).hip1(3);
end
count=1:length(t1)+length(t2);
plot(count,heigh);


%ȷ�����귶Χ
AX=[-2 10];
AY=[-1 1];
AZ=[0 2];

figure;
view(3);
axis equal;
xlim(AX);
ylim(AY);
zlim(AZ);

for step=0:6
    add=[steplength*step;0;0];
    if(note==0)%�ҽ�
        for i=1:length(t1)+length(t2)
            rfoot=gait(i).foot1+add;
            rknee=gait(i).knee1+add;
            rhip=gait(i).hip1+add;
            lhip=gait(i).hip2+add;
            lknee=gait(i).knee2+add;
            lfoot=gait(i).foot2+add;
            %��ͼ
            cla;
            line([rfoot(1) rknee(1)],[rfoot(2) rknee(2)],[rfoot(3) rknee(3)]);
            line([rknee(1) rhip(1)],[rknee(2) rhip(2)],[rknee(3) rhip(3)]);
            line([rhip(1) lhip(1)],[rhip(2) lhip(2)],[rhip(3) lhip(3)],'color','y');
            line([lknee(1) lhip(1)],[lknee(2) lhip(2)],[lknee(3) lhip(3)],'color','r');
            line([lfoot(1) lknee(1)],[lfoot(2) lknee(2)],[lfoot(3) lknee(3)],'color','r');
            drawnow
        end
        note=1;
    else
        for i=1:length(t1)+length(t2)
            transmatrix=[1 0 0;0 -1 0;0 0 1];   %����任
            lfoot=transmatrix*gait(i).foot1+add;
            lknee=transmatrix*gait(i).knee1+add;
            lhip=transmatrix*gait(i).hip1+add;
            rhip=transmatrix*gait(i).hip2+add;
            rknee=transmatrix*gait(i).knee2+add;
            rfoot=transmatrix*gait(i).foot2+add;
            cla;
            line([rfoot(1) rknee(1)],[rfoot(2) rknee(2)],[rfoot(3) rknee(3)]);
            line([rknee(1) rhip(1)],[rknee(2) rhip(2)],[rknee(3) rhip(3)]);
            line([rhip(1) lhip(1)],[rhip(2) lhip(2)],[rhip(3) lhip(3)],'color','y');
            line([lknee(1) lhip(1)],[lknee(2) lhip(2)],[lknee(3) lhip(3)],'color','r');
            line([lfoot(1) lknee(1)],[lfoot(2) lknee(2)],[lfoot(3) lknee(3)],'color','r');
            drawnow
        end
        note=0;
    end
end
