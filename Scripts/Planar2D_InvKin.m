%% clear
clear; close all; clc

%% input
ang2 = -125;
ang3 = -50;
ang4 = -360-ang2-ang3;

%% data
l1 = 612;
l2 = 572;
l3 = 116;
work = l1+l2+l3;
theta1s = 180+ang2;
theta2s = ang3;
theta3s = 90+ang4;
dist = -300;

%% start position
x1s = l1*cosd(theta1s);
y1s = l1*sind(theta1s);
x2s = x1s+(l2*cosd(theta1s+theta2s));
y2s = y1s+(l2*sind(theta1s+theta2s));
x3s = x2s+(l3*cosd(theta1s+theta2s+theta3s));
y3s = y2s+(l3*sind(theta1s+theta2s+theta3s));

%% solution vertical
x2v = x2s;
y2v = y2s + dist;

theta2v = atan2d(-sqrt(1-((x2v^2+y2v^2-l1^2-l2^2)/(2*l1*l2))^2),(x2v^2+y2v^2-l1^2-l2^2)/(2*l1*l2));
theta1v = atan2d(sqrt(1-(((x2v*(l1+(l2*cosd(theta2v))))+(y2v*l2*sind(theta2v)))/(x2v^2+y2v^2))^2),((x2v*(l1+(l2*cosd(theta2v))))+(y2v*l2*sind(theta2v)))/(x2v^2+y2v^2));
% ref: http://cdn.intechweb.org/pdfs/379.pdf
theta3v = -90-theta1v-theta2v;

x1v = l1*cosd(theta1v);
y1v = l1*sind(theta1v);
x2v = x1v+(l2*cosd(theta1v+theta2v));
y2v = y1v+(l2*sind(theta1v+theta2v));
x3v = x2v+(l3*cosd(theta1v+theta2v+theta3v));
y3v = y2v+(l3*sind(theta1v+theta2v+theta3v));

%% solution horizontal
x2h = x2s + dist;
y2h = y2s;

theta2h = atan2d(-sqrt(1-((x2h^2+y2h^2-l1^2-l2^2)/(2*l1*l2))^2),(x2h^2+y2h^2-l1^2-l2^2)/(2*l1*l2));
theta1h = atan2d(sqrt(1-(((x2h*(l1+(l2*cosd(theta2h))))+(y2h*l2*sind(theta2h)))/(x2h^2+y2h^2))^2),((x2h*(l1+(l2*cosd(theta2h))))+(y2h*l2*sind(theta2h)))/(x2h^2+y2h^2));
% ref: http://cdn.intechweb.org/pdfs/379.pdf
theta3h = -90-theta1h-theta2h;

x1h = l1*cosd(theta1h);
y1h = l1*sind(theta1h);
x2h = x1h+(l2*cosd(theta1h+theta2h));
y2h = y1h+(l2*sind(theta1h+theta2h));
x3h = x2h+(l3*cosd(theta1h+theta2h+theta3h));
y3h = y2h+(l3*sind(theta1h+theta2h+theta3h));

%% New positions



%% plot
plot([0 x1s x2s x3s],[0 y1s y2s y3s],'-o','LineWidth',2);
axis equal
axis([-work work -work work]);
hold on
plot([0 x1v x2v x3v],[0 y1v y2v y3v],'-o','LineWidth',2);
plot([0 x1h x2h x3h],[0 y1h y2h y3h],'-o','LineWidth',2);
plot([x3s x3v],[y3s y3v],':','LineWidth',1);
plot([x3s x3h],[y3s y3h],':','LineWidth',1);

viscircles([0 0],work,'LineStyle','--','LineWidth',1);

