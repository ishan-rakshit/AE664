% Methodology for arriving at the baseline specifications of a non-rigid airship of
% conventional configuration, given the performance and operational requirements

% Methodology in the design mode - calculations are initiated with an
% assumed value of envelope volume

%% Free parameters

V_e = 1;
% V_e = 'Starting value of envelope volume: ';
% V_e = input(V_e); % Envelope volume as an input

%% Input parameters

% Operation related parameters

Hmax = 1; % maximum pressure altitude to be attained by airship; in meters
isa_condition = 1; % ISA (1) or ISA+15 (2) condition
min_operating_alt = 1;
He_purity_level = 1; % Helium purity level, in percentage
power_offtake = 0.5; % Power offtake, <1
Re = 1; % Reynolds number
Hmin = 1; % Min height of operation, in meters

% Performance requirements

range = 1; % Range, in meters
cruising_alt = 1; % Cruising altitude, in meters
cruising_speed = 1; % in m/s
prop_eff = 1; % Propulsion efficiency
num_passengers = 1; % Number of passengers
w_payload = 1; % Payload desired, in kg

% Configuration related parameters

num_fins = 1; % Number of fins
fin_layout = 1;
num_engines = 1;
engine_type = 1; % Type of engine: (1) Petrol (2) Diesel
engine_asp = 1; % (1) Normally aspirated engine (2) Not normally aspirated engine
engine_trans = 1; % (1) Simple transmission system (2) Otherwise
engine_thrustvec = 1; % (1) Thrust vectoring (2) Otherwise
engine_prop_unducted = 1; % (1) For unducted propeller (2) Otherwise

env_l_by_d = 1; % Envelope L/D ratio
V_b = 1;
delta_p = 1; % Internal_overpressure
k_se = 1; % Envelope surface area factor
k_ve = 1; % Envelope volume factor


%% Calculations - Conditions imposed by input parameters

if engine_type == 1
    sfc = 0.46*0.45/(746*3600);
    power_weight = 0.85;
else
    sfc=0.37*0.45/(746*3600);
    power_weight = 1.025;
end
power_weight = unit_conversion(power_weight,'power'); % unit conversion

if engine_thrustvec == 1
    engine_thrust_vectoring = 1.14;
else
    engine_thrust_vectoring = 1;
end


[Temp_Hmin, a_min, P_Hmin, rho_air_Hmin] = atmosisa(Hmin);
ballonet_percentage_ISA = [0, 4.7, 9.3, 13.6, 17.9, 20.2, 21.9, 21.9, 23.5, 23.8, 25.8, 26.2, 29.6, 33.2];
bellonet_percentage_ISA_15 = [0, 4.5, 8.8, 13, 17, 19.3, 20.9, 20.9, 22.4, 22.8, 24.7, 25, 28.3, 31.8];
P_alt = [0, 500, 1000, 1500, 2000, 2285, 2500, 2501, 2700, 2743, 3000, 3050, 3500, 4000];

if isa_condition ==1
    v_btr= interp1(P_alt, ballonet_percentage_ISA, Hmax, 'spline')/100;
else
    v_btr= interp1(P_alt, bellonet_percentage_ISA_15, Hmax, 'spline')/100;
end

%% Weather/ISA data

rho_He_std = 0.1785;
rho_He = (He_purity_level * rho_He_std/400 + (100 - He_purity_level) * 1.225/2880)/(He_purity_level/400 + (100 - He_purity_level)/2880);

%% Lift Calculations at Hmax

[Temp_Hmax, a_Hmax, P_Hmax, rho_air_Hmax] = atmosisa(Hmax);
rho_air_std = 1.225;
sigma_air_Hmax = rho_air_Hmax/rho_air_std;
sigma_air_Hmin = rho_air_Hmin/rho_air_std; % will be used later
i = 0;
tolerance = 1;
flag = 1;

%% Calculations - Geometry Submodule

% Envelope

l_e = ((V_e * (env_l_by_d)^2 )/k_ve)^(1/3);
S_e = k_se * (l_e^2)/env_l_by_d;

% Ballonet

v_bpc = 1 - (Hmax/(sigma_air_Hmin * (rho_air_std - rho_He_std * (1 + (delta_p/P_Hmin))) * V_e));
V_b = (v_bpc + v_btr) * V_e;

% Assuming a twin spherical ballonet layout, radius and surface area of
% each ballonet can be estimated

r_b = (3*V_b/(8*pi))^(1/3);
S_b = 2*pi*(r_b^2);

%% Calculations - Fin Geometry

% tail_area_ratio = N_f * (S_f + S_ctr)/S_e;
% fin_location_ratio = l_fte/l_e;
% fin_taper_ratio = C_Tf/C_Rf;
% fin_aspect_ratio = (b^2)/(S_f + S_ctr);
% control_area_ratio = S_ctr/(S_f + S_ctr);
% control_taper_ratio = C_Tctr/C_Rctr;

S_f = 0.061 * (S_e/num_fins);
b_f = sqrt(0.602 * S_f);

while flag == 1 && i < 10000
    k_alt = 1;
    %% Calculations - Aerostatics Submodule
    % Net lift available at pressure altitude Hmax
    L = V_e*(1-v_btr)*sigma_air_Hmax*(rho_air_std - rho_He_std * (1 + (delta_p/P_Hmax)));

    %% Calculating envelope properties
    L_e = (V_e * (env_l_by_d^2)/k_ve)^(1/3);
    S_e = k_se*(L_e)^2/env_l_by_d;
    
    %% Calculating tail properties
    tail_area = 0.061 * (S_e/num_fins);
    b_f = (0.602 * tail_area)^0.5;
    s_ctr = 0.258 * tail_area;
    S_f = tail_area - s_ctr;
    
    %% Calculating ballonet properties
    sigma_air_Hmin = rho_air_Hmin/rho_air_std;
    v_bpc = 1 - Hmax/(sigma_air_Hmin*(rho_air_std - rho_He*(1 + delta_p/P_Hmin)) * V_e);
    V_b = (v_bpc+v_btr)*V_e;
    r_b = (3 * V_b/(8 * pi))^(1/3);
    S_b = 2 * pi * (r_b^2);

    %% Drag-based Calculations
    C_dv = ((0.172 * (env_l_by_d)^0.33 + 0.252/((env_l_by_d)^1.2) + 1.032/(env_l_by_d)^2.7)/((cruising_speed*Re)^(1/6)))/0.5243;
    [Temp_cr, a_cr, P_cruise, rho_air_cruise] = atmosisa(cruising_alt);
    D = C_dv * 0.5 * rho_air_cruise * (cruising_speed^2) * ((V_e)^0.67);
    P_cr = D * (cruising_speed/prop_eff);

    sigma_air_cruise = rho_air_cruise/rho_air_std;
    if engine_type == 1
        k_alt = sigma_air_cruise-((1 - sigma_air_cruise)/7.55);
    end
    
    P_inst = P_cr * (1 + power_offtake)/k_alt;
    w_fuel = range * sfc * P_cr * (1 + power_offtake)/cruising_speed;

    %% Weight-based Calculations
    w_engines = engine_thrust_vectoring * power_weight * P_inst;
    w_empty = weighing(engine_prop_unducted, engine_trans, V_e, S_b, S_f, num_passengers, w_engines, w_fuel, P_inst);
    max_payload = L - w_empty;
    if max_payload < w_payload
        V_e = V_e * (1 + (w_payload - max_payload)/w_payload);
    end
    if abs(max_payload - w_payload) < tolerance
        flag = 0;
    elseif max_payload - w_payload > tolerance
        V_e = V_e * (1 - (max_payload - w_payload)/w_payload);
    end
    i = i + 1;
    disp(i)
end
