 clear all;
 clc;
 
box_center = [0 0];
rho=25;
for t=0:0.05:3.14
    
    angular_rho =-rho/(0.4+sin(t));
    scatter(angular_rho*sin(t),angular_rho*cos(t));
    hold on;
    angular_rho =rho/(0.2+sin(t));
    scatter(angular_rho*sin(t),angular_rho*cos(t));
    hold on;
end
xlim([-100, 100]);
ylim([-100, 100]);
