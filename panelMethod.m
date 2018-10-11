%% PANEL METHOD LABORATORY

clear; clc; %We clear the console and all data

fprintf("\n-------------------------------");
fprintf("\n--- PANEL METHOD LABORATORY ---");
fprintf("\n-------------------------------\n\n");

%% DEFINING THE VARIABLES

U_inf = 1; %Uniform Speed [m/s]
c = 1; %Chord of the airfoil
ro = 1; %Air density [Kg/m^3]

beta = deg2rad(30); %Angle at leading edge
alfa = deg2rad(15); %Angle at trailing edge

gamma = pi - beta - alfa; %Angle at upper point
a = sin(alfa)/sin(gamma); %Lenght of a
b = sin(beta)/sin(gamma); %Lenght of b

N = 3; %Number of panels

teta = deg2rad([180, 30, -15]); %Vector of angles

%% POINTS OF THE AIRFOIL

P1 = [c, 0]; %Trailing Edge
P2 = [0 ,0]; %Leading Edge
P3 = [a*cos(beta), a*sin(beta)]; %Upper point
P4 = P1; %Trailing Edge Again, so we close the surface

panel_points = [P1; P2; P3; P4]; %Matrix of the Nodes
list_panel_points = [P1, P2, P3, P4]; %List of all points
   
x_coord_panel = panel_points(:,1); %We get all x components
y_coord_panel = panel_points(:,2); %We get all y components
        
figure(1)
plot(x_coord_panel, y_coord_panel, '-*r'); %Plot the wing body
hold on 

%% MIDDLE POINTS OF AIRFOIL

P1_mid = [c/2, 0]; %Compute point one
P2_mid = [a*cos(beta)/2, a*sin(beta)/2]; %Computing point 2
P3_mid = [c - b*cos(alfa)/2, b*sin(alfa)/2]; %Computing point 3

middle_points = [P1_mid; P2_mid; P3_mid]; %We create the matrix of Control Points
list_middle_points = [P1_mid, P2_mid, P3_mid]; %List of all middle points

x_coord_middle = middle_points(:,1); %We get all x components
y_coord_middle = middle_points(:,2); %We get all y components
        
plot(x_coord_middle, y_coord_middle, 'ob'); %Plot the triangle
title('WING PROFILE');

%% COMPUTING B_pj

[v0, r_pj] = get_v0(panel_points, middle_points); %Compute v0
[v1, r_pj1] = get_v1(panel_points, middle_points); %Compute v1

r_pj = r_pj'; %We need to get the transpose
r_pj = [r_pj r_pj(:,1)]; %We add the first colum

beta_mat = real((v1-v0))'; %We need its transpose
mat_correction = [1, -1, -1; -1, 1, -1; -1, -1, 1]; %Corrects the signs
beta_mat = beta_mat.*mat_correction; %Apply the correction to beta_mat
fprintf("[+] Beta Matrix:\n"); %Display beta matrix
disp(beta_mat);

%% COMPUTING Velocities*

u_star =(beta_mat)./(2*pi); %Computing u_Pj*

for i=1:3
    for j=1:3
        
        w_star(i,j) = (1/(2*pi))*log(r_pj(i,j+1)/r_pj(i,j));
        
    end
end



%% Computing R [matrix of coefficients]

%AoA = 0; %Angle of attack EX1
AoA = deg2rad(15); %Angle of attack EX2

for i=1:3
    
    R(i,1) = -U_inf*sin(teta(i)-AoA); %Matrix of coefficients
    
    for j=1:3
        
        A(i,j) = u_star(i,j)*sin(teta(i)-teta(j))-w_star(i,j)*cos(teta(i)-teta(j)); %Matrix
    
    end
end

%% CHECK MATRIX DETERMINANT

%{
    Debido a que A es linealmente independiente, imponemos la
    KuttaCondition

    Ultima fila de A = [1, 0, 1...]
    Ultima fila de R = [0]
%}

fprintf("\n[+] Matrix A: \n");
disp(A);

fprintf("\n[+] Matrix R: \n");
disp(R);

fprintf("\n det(A) = %f", det(A));

fprintf("\n\n-------------------------------------------------")
fprintf("\n[+] Since det(A) ~= 0 we impose Kutta Condition");
fprintf("\n-------------------------------------------------")

A(3,:) = [1, 0, 1]; %Substitute last row gamma_1 = -gamma_3
R(3,1) = 0; %Correction in R


fprintf("\n\n[+] Matrix A: \n");
disp(A);

fprintf("\n[+] Matrix R: \n");
disp(R);


x = A\R;
fprintf("Gamma Solution: \n");
disp(x);

for i=1:3    
    L(i) = pdist([panel_points(i); panel_points(i+1)], 'euclidean');
end

circulation = x(1)*L(1) + x(2)*L(2) + x(3)*L(3);

c_l = (2*circulation)/(U_inf*c)

    















