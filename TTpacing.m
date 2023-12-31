% Conventions
% Positive y-axis -> North
% Positive x-axis -> East

% Parameters
windspeed = 3.5; %m/s
wind_heading = pi/2; %x,y norm
mass = 70 + 10; %rider + bike + gear
ftp = 330; %watts
cda = 0.28; %m^2
Cr = 0.005;
slope = 0; %percent
power_array = [293,380,380,350]; %watts - each entry sets the power per section. Array is used for running one lap manually.
max_avgpower = 350; %watts


% Treinweg segmenten
sections_heading = [-120,170,60,-10]*pi/180; %rad, ccw from postive x-axis
sections_length = [2100,1000,2200,1160]; %m

% Run one lap manually
[totaltime,avgpower,vsections,tsections] = simlap(sections_heading,sections_length,power_array,windspeed,wind_heading,cda,mass,Cr,slope);

speed_power_time = [vsections*3.6;power_array;tsections]
totaltime
avgpower

%% Optimization
func = @(x) simlap(sections_heading,sections_length,x,windspeed,wind_heading,cda,mass,Cr,slope);
nonlinconx = @(x) nonlincon(x,sections_heading,sections_length,windspeed,wind_heading,cda,mass,Cr,slope,max_avgpower);

options = optimoptions("fmincon","Display","iter");

x0 = [ftp,ftp,ftp,ftp]; %initial values (set to ftp)
A = [];
b = [];
Aeq = [];
beq = [];
lb = [200,200,200,200]; %lower bounds per section in watts
ub = [370,370,370,370]; %upper bounds per section in watts

%Fmincon optimizer (optimization toolbox)
x = fmincon(func,x0,A,b,Aeq,beq,lb,ub,nonlinconx,options)

%% Functions
function [c,ceq] = nonlincon(x,sections_heading,sections_length,windspeed,wind_heading,cda,mass,Cr,slope,max_avgpower)
    [~,avgpower,~,~] = simlap(sections_heading,sections_length,x,windspeed,wind_heading,cda,mass,Cr,slope);
    ceq = [];

    c(1) = avgpower-max_avgpower;
end

function [totaltime,avgpower,vsections,tsections] = simlap(sections_heading,sections_length,power_array,windspeed,wind_heading,cda,mass,Cr,slope)
    [t1,p1,v1] = simsection(sections_heading(1),sections_length(1),power_array(1),windspeed,wind_heading,cda,mass,Cr,slope);
    [t2,p2,v2] = simsection(sections_heading(2),sections_length(2),power_array(2),windspeed,wind_heading,cda,mass,Cr,slope);
    [t3,p3,v3] = simsection(sections_heading(3),sections_length(3),power_array(3),windspeed,wind_heading,cda,mass,Cr,slope);
    [t4,p4,v4] = simsection(sections_heading(4),sections_length(4),power_array(4),windspeed,wind_heading,cda,mass,Cr,slope);

    vsections = [v1,v2,v3,v4];
    tsections = [t1,t2,t3,t4];

    totaltime = t1+t2+t3+t4;
    avgpower = (power_array(1)*t1 + power_array(2)*t2+ power_array(3)*t3 + power_array(4)*t4)  /  totaltime;

end

function [sectiontime,sectionpower,sectionspeed] = simsection(section_heading,section_length,power,windspeed,wind_heading,cda,mass,Cr,slope)
    sectionspeed = power2speed(windspeed,wind_heading,power,cda,section_heading,mass,Cr,slope);
    sectiontime = section_length/sectionspeed;
    sectionpower = power;
end

function bikespeed = power2speed(windspeed,wind_angle,power,cda,track_angle,mass,Cr,slope)
    w = windspeed*cos(wind_angle-track_angle);

    syms v positive
    Froll = 9.81*cos(atan(slope))*mass*Cr;
    Faero = 0.5*cda*1.225*(v+w)^2;
    Fg = 9.81*sin(atan(slope))*mass;

    Ftot = Froll + Faero + Fg;
    eq = power == Ftot*v;
    
    bikespeed = double(solve(eq,v));    
end


