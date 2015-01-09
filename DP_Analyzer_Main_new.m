
%DP Simplified - Coarse Partitions Method
%this implements the simplified benchmark including a coarse partitions method to successively
%improve the resolution of the optimal trajectories. 
%clear all
close all
clc
format short

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%put in the name of the save file
% prompt='Input filename of the save file for final results:';
% dlg_title='Input';
% answer=inputdlg(prompt,dlg_title);
global selection
global regime
global limited_regen

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%select the drive cycle you will be using, by uncommenting the corresponding number:

%selection=1; %Validation cycle with Preq derived from Simulink model
selection=2; %validation cycle
%selection=3; %US06  full cycle
%selection=4; %US06 city cycle
%selection=5; %US06 highway cycle
%selection=6; %UDDS cycle
%selection=7; %HWFET cycle

%sample time
Ts=1;
%Ts=0.5;

[DP_SIM_DATA.CYCLE.v_vehicle,DP_SIM_DATA.CYCLE.P_req,DP_SIM_DATA.CYCLE.T,DP_SIM_DATA.CYCLE.distance_m]=DP_drive_cycle_select(Ts,selection);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%select operating regime

%regime=1; %series
regime=2; %parallel

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%define the vehicle parameters for the simulation:

%ESS Setup: 
% coarse partition of the state variables is defined as follows:

%Electrical power state values are based on the requirement to change the ESS charge by 1 step value. 
%minimum step: E(1) to E(2)=(2800-2801)=-1Wh=-3600Ws=-3600W=-3.6kW in 1s (since simulation is at 1Hz)
%maximum step: E(1) to E(400)=2800-3200=-400Wh=-1.44e6Ws=-1.4MW in 1s
%upper and lower bounds constrain the maximum change of E at between stages

SOC_max=0.22; %max SOC (%) %0.312=5054Wh 
SOC_min=0.14; %min SOC (%) %0.288=4665Wh

SOC_target=0.20; %target SOC

DP_SIM_DATA.ESS.E_capacity_max=16200; %ESS energy capacity (Wh)
DP_SIM_DATA.ESS.V_nominal=292; %ESS nominal voltage (V)
DP_SIM_DATA.ESS.Discharge_Limit=[188.700000000000,198.940000000000,209.180000000000,219.420000000000,229.660000000000,239.900000000000,250.140000000000,260.380000000000,270.620000000000,280.860000000000,291.100000000000,301.340000000000,311.580000000000,321.820000000000,332.060000000000,342.300000000000,352.540000000000,362.780000000000,373.020000000000,383.260000000000,393.500000000000;];

DP_SIM_DATA.ESS.E_max=SOC_max*DP_SIM_DATA.ESS.E_capacity_max; %maximum value of state variable E (Wh)
DP_SIM_DATA.ESS.E_target=SOC_target*DP_SIM_DATA.ESS.E_capacity_max; %target value of state variable E (Wh)
DP_SIM_DATA.ESS.E_min=SOC_min*DP_SIM_DATA.ESS.E_capacity_max; %minimum value of state variable E (Wh)

E_step_min_coarse=1; % minimum step size of the state variable

E_grid_size=int16((DP_SIM_DATA.ESS.E_max-DP_SIM_DATA.ESS.E_min)/E_step_min_coarse); %grid size of the state variable at each stage
E_grid=DP_SIM_DATA.ESS.E_min:E_step_min_coarse:DP_SIM_DATA.ESS.E_max; %generate the SOC grid

%E_state_target=int16(length(E_grid)/2); %this is the target final value of the state variable.
E_state_target=find(abs(E_grid-DP_SIM_DATA.ESS.E_target)<=(E_step_min_coarse/2),1,'last'); %this is the target final value of the state variable.

E_grid_size_fine=200;
E_state_target_fine=101;

%Electric motor setup:
%regen levels: -90kW=100%, -18kW=20%, -9kW=10%
DP_SIM_DATA.RTM.P_min=-18000; % RTM power limits in [W] %%%%%%%%%%%%%%%%%%%(-40kW works, -33kW just barely not CS in US06City, -40 works for validation, -40kW US06City) 
%motoring limit
DP_SIM_DATA.RTM.P_max=110000; %110k for series, 90k for parallel, 50k for parallel limited 
DP_SIM_DATA.RTM.RTM_gear=7.82;
DP_SIM_DATA.RTM.RTMSpeedIndex_rpm=[0,250.8,501.3,802.3,1003.3,1494.3,2004.9,2497,2997.6,3499.4,4001.6,4502.5,5005,5504.4,6006.5,6507.1,7009.4,7510.3,8006,8502.4,8813.5];
DP_SIM_DATA.RTM.RTM_Avg_Eff_Table=[0.100000000000000,0.637980392156863,0.758117647058824,0.811294117647059,0.831196078431373,0.855450980392157,0.866000000000000,0.877019607843138,0.882176470588236,0.889215686274510,0.891411764705883,0.897960784313726,0.899745098039215,0.900607843137254,0.901647058823530,0.899294117647059,0.899313725490196,0.898431372549020,0.898764705882353,0.891764705882354,0.885980392156862;];

DP_SIM_DATA.BAS.P_BAS_max=80000;
DP_SIM_DATA.BAS.BAS_belt_ratio=1.23;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%select full or limited regen
limited_regen=0; %full regen
%limited_regen=1; %limited regen (modifies penalties)

P_req_cont=DP_SIM_DATA.CYCLE.P_req;

DP_SIM_DATA.Trans.gear_ratios=[4.584 2.964 1.912 1.446 1 0.746]; %array of transmission gear ratios;
DP_SIM_DATA.Trans.upshift_threshold=3000; %engine speed (in RPM) threshold, moving above this triggers an upshift
DP_SIM_DATA.Trans.downshift_threshold=1000; %engine speed (in RPM) threshold, moving below this triggers a downshift 
DP_SIM_DATA.Trans.front_diff_ratio=3.71; %front differential ratio
DP_SIM_DATA.Vehicle.wheel_radius=18; %wheel radius in inches

DP_SIM_DATA.CONST.wheel_conversion_const=(1/((DP_SIM_DATA.Vehicle.wheel_radius*2*pi)*0.0254)*(2*pi)); %conversion from longitudinal vehicle speed [km/hr] to angular wheel speed [rpm], multiply it with v_vehicle in m/s to get wheel rad/s 
DP_SIM_DATA.CONST.rads_to_rpm=(30/pi); %convert rotational speed in rads/s to rpm
DP_SIM_DATA.CONST.rpm_to_rads=1/DP_SIM_DATA.CONST.rads_to_rpm; %convert rotational speed in rpm to rads
DP_SIM_DATA.CONST.kmhr_to_ms=0.277778; %conversion factor to go from km/hr to m/s


DP_SIM_DATA.ICE.idle_speed=1000; %engine idle speed, in RPM
DP_SIM_DATA.ICE.P_ICE_max=130000; %130kW peak @5800 rpm, 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%start the benchmark program:
%you can comment or uncomment coarse or fine search to avoid repeating long computations, if you
%already have the data you need.
%the nominal optimal trajectory is found with an initial DP passthrough:

%Generate Vehicle Operation Profile - speed/power over the drive cycle,
%gears, wheel/RTM speeds, efficiencies, etc. 
disp('Generating vehicle operation profile...');
[DP_SIM_DATA,process_time_veh_operation]=DP_Vehicle_Drive_Profile(Ts,DP_SIM_DATA);
disp('Done.');
% 
disp('Preparing DP coarse search...');
%[E_grid,process_times_coarse]=DP_Coarse_Search(Ts,regime,limited_regen,E_grid,E_grid_size,E_state_target,DP_SIM_DATA);
disp('DP coarse search complete. Running initial analysis...');
[E_cost_rolledforward,E_elec_opt_coarse,P_ICE_opt_coarse,P_brake_opt_coarse,P_RTM_opt_coarse,P_gen_opt_coarse,P_BAS_opt_coarse,P_elec_opt_coarse,Fuel_Used_coarse,process_time_coarse_analysis]=DP_Coarse_Analysis(Ts,regime,E_grid,E_state_target,E_grid_size,DP_SIM_DATA);
disp('Done.');

%save important data from the coarse analysis, clear out other variables to free memory
E_cost_rolledforward_coarse=E_cost_rolledforward;
save('DP_General_Data.mat','DP_SIM_DATA','E_state_target','E_state_target_fine','Ts','process_time_coarse_analysis','process_time_veh_operation','limited_regen','regime','selection','E_grid_size_fine','E_step_min_coarse');
save('DP_Coarse_Data.mat','E_elec_opt_coarse', 'P_RTM_opt_coarse','P_gen_opt_coarse','P_BAS_opt_coarse','P_ICE_opt_coarse','P_brake_opt_coarse','P_elec_opt_coarse','E_cost_rolledforward_coarse','process_times_coarse');
clear all %...to here. 
load('DP_General_Data');
load('DP_Coarse_Data');

disp('Starting DP fine search...');
[E_cost,P_RTM,P_gen,process_times_fine]=DP_Fine_Search(E_grid_size_fine,Ts,regime,E_elec_opt_coarse,E_step_min_coarse,DP_SIM_DATA); %run the DP fine search 
disp('DP fine search complete. Running analysis...');

%save important data from the fine analysis, clear out other variables to free memory
E_cost_fine=E_cost;
P_RTM_fine=P_RTM;
P_gen_fine=P_gen;
save('DP_Fine_Data.mat','E_cost_fine','P_RTM_fine','P_gen_fine','process_times_fine','E_grid_size_fine');
clear all

%now reload all important data, and perform a final analysis
load('DP_General_Data.mat');
load('DP_Coarse_Data.mat');
load('DP_Fine_Data.mat');

[E_elec_opt_fine,P_elec_opt_fine,P_ICE_opt_fine,P_brake_opt_fine,fuel_consumed_L_opt,P_RTM_opt_fine,P_gen_opt_fine,P_BAS_opt_fine,process_time_fine_analysis,N_ICE_opt]=DP_Fine_Analysis(E_grid_size_fine,Ts,regime,E_cost_fine,E_state_target,E_step_min_coarse,E_elec_opt_coarse,P_RTM_fine,P_gen_fine,DP_SIM_DATA);
disp('Done.');

disp('Processing fine results...');
[DP_SIM_DATA,process_times]=DP_Postprocessing(DP_SIM_DATA,fuel_consumed_L_opt,process_times_coarse,process_times_fine,process_time_coarse_analysis,process_time_fine_analysis,process_time_veh_operation);
disp('Done.');

clear s process_times_coarse process_times_fine process_time_veh_operation process_time_coarse_analysis process_time_postprocessing_fine process_time_fine_analysis

%evaluate the memory usage
mem_struct=whos();
mem_struct(1).bytes;
process_memory_used=cell(1,10);
process_memory_used{1,1}='Total Memory Used (Mb)';
process_memory_used{1,2}=sum([mem_struct(:).bytes])/1048576;
process_memory_used{1,3}='Size of E_cost_coarse (Mb)';
for i=1:length(mem_struct) 
        if strcmp(mem_struct(i).name,'E_cost_coarse') 
            process_memory_used{1,4}=mem_struct(i).bytes/1048576;
        else
        end
end
process_memory_used{1,5}='Size of E_cost_fine (Mb)';
for i=1:length(mem_struct) 
        if strcmp(mem_struct(i).name,'E_cost_fine') 
            process_memory_used{1,6}=mem_struct(i).bytes/1048576;
        else
        end
end
process_memory_used{1,7}='Size of P_RTM_coarse (Mb)';
process_memory_used{1,9}='Size of P_RTM_fine (Mb)';
for i=1:length(mem_struct) 
        if strcmp(mem_struct(i).name,'P_RTM_fine') 
            process_memory_used{1,10}=mem_struct(i).bytes/1048576;
        else
        end
end

%generate drive cycle stuff for simulink
% sch_cycle=zeros(T,2);
% sch_cycle(:,1)=v_vehicle;
% sch_cycle(:,2)=1:1:T;

% %clear coarse arrays and cost arrays, to reduce memory. can comment this to
% %retain these variables, if troubleshooting.
% clear E_cost_coarse E_cost_fine E_cost_rolledforward_coarse E_cost_rolledforward_fine P_BAS_opt_coarse P_ICE_opt_coarse P_RTM_opt_coarse P_brake_opt_coarse P_elec_opt_coarse P_gen_opt_coarse 
% 
% switch selection
%     case 1
%         if regime==1
%             delete('Validation_4Wh_Series_simulink.mat');
%             save('Validation_4Wh_Series_simulink.mat');
%             delete('Validation_reqs_series_simulink.mat');
%             save('Validation_reqs_series_simulink.mat','Treq_RTM_opt','Treq_ICE_opt','Treq_brake_opt');
%         else
%             delete('Validation_4Wh_Parallel_simulink.mat');
%             save('Validation_4Wh_Parallel_simulink.mat');
%             delete('Validation_reqs_parallel_simulink.mat');
%             save('Validation_reqs_parallel_simulink.mat','Treq_RTM_opt','Treq_ICE_opt','Treq_brake_opt','Trans_gear_opt','PS_opt','ESS_CS_target_opt');
%         end
%     case 2
%         if regime==1
%             delete('Validation_4Wh_Series.mat');
%             save('Validation_4Wh_Series.mat');
%             %save('Validation_2Wh_0p5s_Series.mat');
%             
%             delete('Validation_reqs_series.mat');
%             save('Validation_reqs_series.mat','Treq_RTM_opt','Treq_ICE_opt','Treq_brake_opt');
%         else
%             %delete('Validation_2Wh_Parallel.mat');
%             save('Validation_2Wh_0p5s_Parallel.mat');
%             
%             %             delete('Validation_4Wh_Parallel.mat');
%             %             save('Validation_4Wh_Parallel.mat');
%             %             delete('Validation_reqs_parallel.mat');
%             %             save('Validation_reqs_parallel.mat','Treq_RTM_opt','Treq_ICE_opt','Treq_brake_opt','Trans_gear_opt','PS_opt','ESS_CS_target_opt');
%         end
%     case 4
%         switch limited_regen
%             case 0
%                 if regime==1
%                     %delete('US06City_2Wh_0p5s_Series.mat');
%                     %save('US06City_4Wh_Series.mat');
%                     save('US06City_2Wh_0p5s_Series.mat');
%                     delete('US06_city_reqs_series.mat');
%                     save('US06_city_reqs_series.mat','Treq_RTM_opt','Treq_ICE_opt','Treq_brake_opt','Trans_gear_opt','PS_opt','ESS_CS_target_opt');
%                 else
%                     delete('US06City_2Wh_0p5s_Parallel.mat');
%                     %save('US06City_4Wh_Parallel.mat');
%                     save('US06City_2Wh_0p5s_Parallel.mat');
%                     delete('US06_city_reqs_parallel.mat');
%                     save('US06_city_reqs_parallel.mat','Treq_RTM_opt','Treq_ICE_opt','Treq_brake_opt','Trans_gear_opt','PS_opt','ESS_CS_target_opt');
%                 end
%             case 1
%                 if regime==1
%                     %delete('US06City_2Wh_0p5s_Series.mat');
%                     save('US06City_4Wh_limited_Series.mat');
%                     %save('US06City_2Wh_0p5s_Series.mat');
%                     delete('US06_city_reqs_series.mat');
%                     save('US06_city_reqs_series.mat','Treq_RTM_opt','Treq_ICE_opt','Treq_brake_opt','Trans_gear_opt','PS_opt','ESS_CS_target_opt');
%                 else
%                     delete('US06City_2Wh_0p5s_Parallel_limited.mat');
%                     %save('US06City_4Wh_Parallel.mat');
%                     save('US06City_2Wh_0p5s_Parallel_limited.mat');
%                     delete('US06_city_reqs_parallel_limited.mat');
%                     save('US06_city_reqs_parallel_limited.mat','Treq_RTM_opt','Treq_ICE_opt','Treq_brake_opt','Trans_gear_opt','PS_opt','ESS_CS_target_opt');
%                 end
%                    
%         end
%     case 5
%         if regime==1
%             delete('US06Highway_2Wh_0p5s_Series.mat');
%             save('US06Highway_2Wh_0p5s_Series.mat');
%             
%             delete('US06_highway_reqs_series.mat');
%             save('US06_highway_reqs_series.mat','Treq_RTM_opt','Treq_ICE_opt','Treq_brake_opt','Trans_gear_opt','PS_opt','ESS_CS_target_opt');
%         else
%             delete('US06Highway_2Wh_0p5s_Parallel.mat');
%             save('US06Highway_2Wh_0p5s_Parallel.mat');
%             
%             delete('US06_highway_reqs_parallel.mat');
%             save('US06_highway_reqs_parallel.mat','Treq_RTM_opt','Treq_ICE_opt','Treq_brake_opt','Trans_gear_opt','PS_opt','ESS_CS_target_opt');
%         end
%     case 6
%         if regime==1
%             delete('UDDS_4Wh_Series.mat');
%             save('UDDS_4Wh_Series.mat');
%             delete('UDDS_reqs_series.mat');
%             save('UDDS_reqs_series.mat','Treq_RTM_opt','Treq_ICE_opt','Treq_brake_opt','Trans_gear_opt','PS_opt','ESS_CS_target_opt');
%         else
%             delete('UDDS_4Wh_Parallel.mat');
%             save('UDDS_4Wh_Parallel.mat');
%             delete('UDDS_reqs_parallel.mat');
%             save('UDDS_reqs_parallel.mat','Treq_RTM_opt','Treq_ICE_opt','Treq_brake_opt','Trans_gear_opt','PS_opt','ESS_CS_target_opt');
%         end
%     case 7
%         if regime==1
%             delete('HWFET_4Wh_Series.mat');
%             save('HWFET_4Wh_Series.mat');
%             delete('HWFET_reqs_series.mat');
%             save('HWFET_reqs_series.mat','Treq_RTM_opt','Treq_ICE_opt','Treq_brake_opt','Trans_gear_opt','PS_opt','ESS_CS_target_opt');
%         else
%             delete('HWFET_4Wh_Parallel.mat');
%             save('HWFET_4Wh_Parallel.mat');
%             delete('HWFET_reqs_parallel.mat');
%             save('HWFET_reqs_parallel.mat','Treq_RTM_opt','Treq_ICE_opt','Treq_brake_opt','Trans_gear_opt','PS_opt','ESS_CS_target_opt');
%         end
%     otherwise
% end
% 
% %clear all
% %close all
