%% PANEL METHOD LABORATORY

clear; clc; %We clear the console and all data

%% DEFINING THE VARIABLES

U_inf = 1; %Uniform Speed [m/s]
c = 1; %Chord of the airfoil
ro = 1; %Air density [Kg/m^3]
AoA = 0; %Angle of attack

beta = deg2rad(30); %Angle at leading edge
alfa = deg2rad(15); %Angle at trailing edge

gamma = pi - beta - alfa; %Angle at upper point
a = sin(alfa)/sin(gamma); %Lenght of a
b = sin(beta)/sin(gamma); %Lenght of b

N = 3; %Number of panels

%% POINTS OF THE AIRFOIL

P1 = [c, 0]; %Trailing Edge
P2 = [0 ,0]; %Leading Edge
P3 = [a*cos(beta), a*sin(beta)]; %Upper point
P4 = P1; %Trailing Edge Again, so we close the surface

panel_points = [P1; P2; P3; P4]; %Matrix of the Nodes
list_panel_points = [P1, P2, P3, P4]; %List of all points
   
x_coord_panel = panel_points(:,1); %We get all x components
y_coord_panel = panel_points(:,2); %We get all y components
        
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

%% COMPUTING B_pj

[v0, r_pj] = get_v0(panel_points, middle_points); %Compute v0
[v1, r_pj1] = get_v1(panel_points, middle_points); %Compute v1

beta_mat = real((v1-v0))'; %We need its transpose
mat_correction = [1, -1, -1; -1, 1, -1; -1, -1, 1]; %Corrects the signs
beta_mat = beta_mat.*mat_correction; %Apply the correction to beta_mat
fprintf("Beta Matrix:\n"); %Display beta matrix
disp(beta_mat);

%% COMPUTING















