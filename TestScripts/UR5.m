%% DH and Inertia Parameters of UR5
alpha = [0,pi/2,0,0,-pi/2,pi/2];
a = [0,0,.425,.39225,0,0];
d = [.08916,0,0,.10915,.09456,.0823];
m = [3.7,8.393,2.275,1.219,1.219,.1879];
p_c1 = [0,-1.93,-26.51]*10^(-3);
p_c2 = [212.5,0,113.36]*10^(-3);
p_c3 = [272.32,0,26.5]*10^(-3);
p_c4 = [0,16,34,107.35]*10^(-3);
p_c5 = [0,-16.34,-1.8]*10^(-3);
p_c6 = [0,0,-1.159]*10^(-3);
In_1 = [84,0,0;0,64,0;0,0,84]*10^(-4);
In_2 = [78,0,0;0,21,0;0,0,21]*10^(-4);
In_3 = [16,0,0;0,462,0;0,0,462]*10^(-4);
In_4 = [16,0,0;0,16,0;0,0,9]*10^(-4);
In_5 = [16,0,0;0,16,0;0,0,9]*10^(-4);
In_6 = eye(3)*10^(-4);
%% PARAMETERs AND SYMBOLs
g=sym('g');
alpha_0=alpha(1);alpha_1=alpha(2);alpha_2=alpha(3);alpha_3=alpha(4);alpha_4=alpha(5);alpha_5=alpha(6);
a_0=a(1);a_1=a(2);a_2=a(3);a_3=a(4);a_4=a(5);a_5=a(6);
d_1=d(1);d_2=d(2);d_3=d(3);d_4=d(4);d_5=d(5);d_6=d(6);
p_cx1=p_c1(1);p_cy1=p_c1(2);p_cz1=p_c1(3);
p_cx2=p_c2(1);p_cy2=p_c2(2);p_cz2=p_c2(3);
p_cx3=p_c3(1);p_cy3=p_c3(2);p_cz3=p_c3(3);
p_cx4=p_c4(1);p_cy4=p_c4(2);p_cz4=p_c4(3);
p_cx5=p_c5(1);p_cy5=p_c5(2);p_cz5=p_c5(3);
p_cx6=p_c6(1);p_cy6=p_c6(2);p_cz6=p_c6(3);
m_1=m(1);m_2=m(2);m_3=m(3);m_4=m(4);m_5=m(5);m_6=m(6);
q_1=sym('q_1');q_2=sym('q_2');q_3=sym('q_3');q_4=sym('q_4');q_5=sym('q_5');q_6=sym('q_6');
dq_1=sym('dq_1');dq_2=sym('dq_2');dq_3=sym('dq_3');dq_4=sym('dq_4');dq_5=sym('dq_5');dq_6=sym('dq_6');
%% ROTATION MATRICEs
R_1=[cos(q_1) -sin(q_1) 0;
     sin(q_1)*cos(alpha_0) cos(q_1)*cos(alpha_0) -sin(alpha_0);
     sin(q_1)*sin(alpha_0) cos(q_1)*sin(alpha_0)  cos(alpha_0)];
R_2=[cos(q_2) -sin(q_2) 0;
     sin(q_2)*cos(alpha_1) cos(q_1)*cos(alpha_1) -sin(alpha_1);
     sin(q_2)*sin(alpha_1) cos(q_2)*sin(alpha_1)  cos(alpha_1)];
R_3=[cos(q_3) -sin(q_3) 0;
     sin(q_3)*cos(alpha_2) cos(q_3)*cos(alpha_2) -sin(alpha_2);
     sin(q_3)*sin(alpha_2) cos(q_3)*sin(alpha_2)  cos(alpha_2)];
R_4=[cos(q_4) -sin(q_4) 0;
     sin(q_4)*cos(alpha_3) cos(q_4)*cos(alpha_3) -sin(alpha_3);
     sin(q_4)*sin(alpha_3) cos(q_4)*sin(alpha_3)  cos(alpha_3)];
R_5=[cos(q_5) -sin(q_5) 0;
     sin(q_5)*cos(alpha_4) cos(q_5)*cos(alpha_4) -sin(alpha_4);
     sin(q_5)*sin(alpha_4) cos(q_5)*sin(alpha_4)  cos(alpha_4)];
R_6=[cos(q_6) -sin(q_6) 0;
     sin(q_6)*cos(alpha_5) cos(q_6)*cos(alpha_5) -sin(alpha_5);
     sin(q_6)*sin(alpha_5) cos(q_6)*sin(alpha_5)  cos(alpha_5)];
%% POSITION VECTORs
p_1=[a_0;-sin(alpha_0)*d_1;cos(alpha_0)*d_1];
p_2=[a_1;-sin(alpha_1)*d_2;cos(alpha_1)*d_2];
p_3=[a_2;-sin(alpha_2)*d_3;cos(alpha_2)*d_3];
p_4=[a_3;-sin(alpha_3)*d_4;cos(alpha_3)*d_4];
p_5=[a_4;-sin(alpha_4)*d_5;cos(alpha_4)*d_5];
p_6=[a_5;-sin(alpha_5)*d_6;cos(alpha_5)*d_6];
%% TRANSLATION MATRICES AND FORWARD KINEMATICS
T_1 = [R_1,p_1;zeros(1,3),1];
T_2 = [R_2,p_2;zeros(1,3),1];
T_3 = [R_3,p_3;zeros(1,3),1];
T_4 = [R_4,p_4;zeros(1,3),1];
T_5 = [R_5,p_5;zeros(1,3),1];
T_6 = [R_6,p_6;zeros(1,3),1];
T = T_1*T_2*T_3*T_4*T_5*T_6;
%% COMs' POSITION VECTORs
p_c1=p_1+R_1*[p_cx1;p_cy1;p_cz1];
p_c2=p_1+R_1*(p_2+R_2*[p_cx2;p_cy2;p_cz2]);
p_c3=p_1+R_1*(p_2+R_2*(p_3+R_3*[p_cx3;p_cy3;p_cz3]));
p_c4=p_1+R_1*(p_2+R_2*(p_3+R_3*(p_4+R_4*[p_cx4;p_cy4;p_cz4])));
p_c5=p_1+R_1*(p_2+R_2*(p_3+R_3*(p_4+R_4*(p_5+R_5*([p_cx5;p_cy5;p_cz5])))));
p_c6=p_1+R_1*(p_2+R_2*(p_3+R_3*(p_4+R_4*(p_5+R_5*(p_6+R_6*[p_cx6;p_cy6;p_cz6])))));
%% SYSTEM's STATEs
q=[q_1;q_2;q_3;q_4;q_5;q_6];
% dq=[dq_1;dq_2;dq_3;dq_4;dq_5;dq_6];
%% LINEAR PART of JACOBIANs
J_v1=jacobian(p_c1,q);
J_v2=jacobian(p_c2,q);
J_v3=jacobian(p_c3,q);
J_v4=jacobian(p_c4,q);
J_v5=jacobian(p_c5,q);
J_v6=jacobian(p_c6,q);
%% ROTATION MATRICEs FROM BASE
R_20=R_1*R_2;
R_30=R_20*R_3;
R_40=R_30*R_4;
R_50=R_40*R_5;
R_60=R_50*R_6;
%% ANGULAR PART of JACOBIANs
%o=zeros(3,7);
J_o1=[R_1(:,3),zeros(3,5)];
J_o2=[R_1(:,3),R_20(:,3),zeros(3,4)];
J_o3=[R_1(:,3),R_20(:,3),R_30(:,3),zeros(3,3)];
J_o4=[R_1(:,3),R_20(:,3),R_30(:,3),R_40(:,3),zeros(3,2)];
J_o5=[R_1(:,3),R_20(:,3),R_30(:,3),R_40(:,3),R_50(:,3),zeros(3,1)];
J_o6=[R_1(:,3),R_20(:,3),R_30(:,3),R_40(:,3),R_50(:,3),R_60(:,3)];
%% JACOBIAN MATRIX OF THE END-EFFECTOR
Jacobi = [J_v6;J_o6];
%% ROBOT's INERTIA (MASS) MATRIX
M=J_v1.'*m_1*eye(3)*J_v1+J_o1.'*R_1*In_1*R_1.'*J_o1...
 +J_v2.'*m_2*eye(3)*J_v2+J_o2.'*R_20*In_2*R_20.'*J_o2...
 +J_v3.'*m_3*eye(3)*J_v3+J_o3.'*R_30*In_3*R_30.'*J_o3...
 +J_v4.'*m_4*eye(3)*J_v4+J_o4.'*R_40*In_4*R_40.'*J_o4...
 +J_v5.'*m_5*eye(3)*J_v5+J_o5.'*R_50*In_5*R_50.'*J_o5...
 +J_v6.'*m_6*eye(3)*J_v6+J_o6.'*R_60*In_6*R_60.'*J_o6;
%% CORIOLIS and CENTRIFUGAL MATRIX
for k=1:6
   for s=1:6
      C(k,s)=.5*((diff(M(k,s),q_1)+diff(M(k,1),q(s,1))-diff(M(1,s),q(k,1)))*dq_1...
                +(diff(M(k,s),q_2)+diff(M(k,2),q(s,1))-diff(M(2,s),q(k,1)))*dq_2...
                +(diff(M(k,s),q_3)+diff(M(k,3),q(s,1))-diff(M(3,s),q(k,1)))*dq_3...
                +(diff(M(k,s),q_4)+diff(M(k,4),q(s,1))-diff(M(4,s),q(k,1)))*dq_4...
                +(diff(M(k,s),q_5)+diff(M(k,5),q(s,1))-diff(M(5,s),q(k,1)))*dq_5...
                +(diff(M(k,s),q_6)+diff(M(k,6),q(s,1))-diff(M(6,s),q(k,1)))*dq_6);
   end
end
%% POTENTIAL ENERGIES and GRAVITY VECTOR
P1=m_1*[0,0,g]*p_c1;
P2=m_2*[0,0,g]*p_c2;
P3=m_3*[0,0,g]*p_c3;
P4=m_4*[0,0,g]*p_c4;
P5=m_5*[0,0,g]*p_c5;
P6=m_6*[0,0,g]*p_c6;
P=P1+P2+P3+P4+P5+P6;
g_1=diff(P,q_1);
g_2=diff(P,q_2);
g_3=diff(P,q_3);
g_4=diff(P,q_4);
g_5=diff(P,q_5);
g_6=diff(P,q_6);
G=[g_1;g_2;g_3;g_4;g_5;g_6];
%% DYNAMICAL EQUATIONs of MOTION
% % % % %           M(q)*ddq + C(q,dq)dq + G(q) = u
save ('UR5.mat','T','Jacobi','M','C','G');

fid = fopen('UR5T.txt', 'w');
fwrite(fid, char(T), 'char');
fclose(fid);

fid = fopen('UR5M.txt', 'w');
fwrite(fid, char(M), 'char');
fclose(fid);

fid = fopen('UR5C.txt', 'w');
fwrite(fid, char(C), 'char');
fclose(fid);

fid = fopen('UR5G.txt', 'w');
fwrite(fid, char(G), 'char');
fclose(fid);

fid = fopen('UR5J.txt', 'w');
fwrite(fid, char(Jacobi), 'char');
fclose(fid);

clear 