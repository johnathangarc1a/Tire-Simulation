%% Given Data

clear all;
clc

g = 32.2;
% acceleration due to gravity (ft/s^2 )

W_car = 505;  %%
W_driver = 150; %%
W_total = total(W_car, W_driver);
Mass_car = W_total / 9.8;

front_bias = .75; %%
rear_bias = 1-front_bias;

Weight_f = front_bias * W_total;
% weight on front tires (lbs)
Weight_r = rear_bias * W_total; 
% weight on rear tires (lbs)


len_car = 62; %% 
% (in)

X_cg = rear_bias*len_car;
% horizontal distance from front tire to cg
Y_cg = 10.9; %%
% height of cg (inches)

V = 0:5:75;
% range of velocities (mph)
V_fps = mphTOfps(V);
% velocity in fps

%% Given Data (brake components)

tire_radius = 5; %%
% radius of tire (inches)

mu_t = 1.4; %%
% coefficient of friction between tire & road

mu_front_brake = 0.60; %%
% coefficient of friction between front pads and rotors
mu_rear_brake = 0.60; %%
% coefficient of friction between rear pads and rotots

pedal_ratio = 6.2; %%
% pedal ratio 

%% Downforce 

air_density = .0023363397;
% air density in slug/ft^3

wing_SA = 16.89934; %%
% surface area of wing (ft^2)

c_df = 2.3; %%
% coefficient of downforce at max downforce setting

df = downforce(air_density, V_fps, wing_SA, c_df);
% lift equation (lbs)

df_fbias = .48; %%
% amount of front downforce bias (%)

df_rbias = 1-df_fbias;
% amount of rear downforce bias (%)

df_front = df*df_fbias;
% amount of downforce in front at different velocities (lbs)

df_rear = df*df_rbias;
% amount of downforce in rear at different velocities (lbs)

%% Braking Distance

braking_dist = braking_distance(V_fps, mu_t);

%% Kinetic Energy at Specific Speed Changes

V_f = 0;
% final speed in mph
KE_init = kinetic_energy(Mass_car, mphTOms(V));
% initial kinetic energy (ft*lb)
KE_final = kinetic_energy(Mass_car, mphTOms(V_f));
% final kinetic energy (ft*lb) , (0 bc at rest after braking)
change_KE = KE_init - KE_final;
% change in kinetic energy

%% MAX Permissable Deceleration of Car at Different Speeds

N_front = ((mu_t*W_total*Y_cg)/len_car) + ((W_total*(len_car-X_cg))/len_car) + df_front;
% Normal force on front tire during max braking (lbs)
N_rear = ((W_total*X_cg)/len_car) - ((W_total*mu_t*Y_cg)/len_car) + df_rear;
% Normal force on front tire during max braking (lbs)
N_total = total(N_front, N_rear);

front_stopping_force = -N_front * mu_t;
rear_stopping_force = -N_rear * mu_t;
total_stopping_force = total(front_stopping_force, rear_stopping_force);
% max potential stopping force of car (lbs)
% aka max available tractive force 

acceleration = total_stopping_force/W_total;
% F=ma : deceleration in g's
% max deceleration = max tractive force / mass


save brakeInput.mat

%% Functions 

% downforce
function df = downforce(air_density, V_fps, wing_SA, c_df)
    df = ((1/2)*air_density*V_fps.^2*wing_SA*c_df);
end

% total function
function total = total(x, y)
    total = x + y;
end

% mph to ft/s
function q = mphTOfps(mph)
    q = (mph * 5280) / 3600;
end

% braking distance
function b = braking_distance(velocity, coeff_friction)
    b = (velocity.^2)/(g*coeff_friction);
end

% kinetic energy
function ke = kinetic_energy(m, v)
    ke = 1/2 * m * v.^2;
end

% mph to m/s
function m = mphTOms(mph)
    m = mph * .44704;
end