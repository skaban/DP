%Vehicle Drive Profile
function [DP_SIM_DATA_updated,process_time_veh_operation]=DP_Vehicle_Drive_Profile(Ts,DP_SIM_DATA)

tStart=tic;

DP_SIM_DATA_updated=DP_SIM_DATA;

Trans_gear=0; %from 0 to 6, start in 0 (neutral)

v_k=DP_SIM_DATA.CYCLE.v_vehicle(DP_SIM_DATA.CYCLE.T); %current vehicle speed
v_kp1=0; %vehicle speed at previous time step (previous in time, not in the simulation)
Trans_gear_kp1=0;
[N_ICE_rpm_k,Trans_gear_k,wheelspeed_rads_k]=DP_transmission_operation(DP_SIM_DATA,v_k,v_kp1,Trans_gear_kp1);
DP_SIM_DATA_updated.CYCLE.wheelspeed_rads(DP_SIM_DATA.CYCLE.T)=wheelspeed_rads_k;
DP_SIM_DATA_updated.CYCLE.Trans_gear(DP_SIM_DATA.CYCLE.T)=Trans_gear_k;
Trans_gear_kp1=Trans_gear_k;
DP_SIM_DATA_updated.CYCLE.ICE_rpm(DP_SIM_DATA.CYCLE.T)=N_ICE_rpm_k;
DP_SIM_DATA_updated.RTM.w_RTM_rads(DP_SIM_DATA.CYCLE.T)=(DP_SIM_DATA_updated.CYCLE.wheelspeed_rads(DP_SIM_DATA.CYCLE.T)*DP_SIM_DATA.RTM.RTM_gear)+0.00001; %Find RTM rotational speed
DP_SIM_DATA_updated.RTM.RTM_Eff(DP_SIM_DATA.CYCLE.T)=interp1(DP_SIM_DATA.RTM.RTMSpeedIndex_rpm,DP_SIM_DATA.RTM.RTM_Avg_Eff_Table,abs(DP_SIM_DATA_updated.RTM.w_RTM_rads(DP_SIM_DATA.CYCLE.T)*DP_SIM_DATA.CONST.rads_to_rpm)); %Find RTM efficiency using avg efficiency table

for k=(DP_SIM_DATA.CYCLE.T-1):-1:1
    %determine vehicle speed, and therefore transmission/ICE conditions
    v_k=DP_SIM_DATA.CYCLE.v_vehicle(k); %current vehicle speed
    v_kp1=DP_SIM_DATA.CYCLE.v_vehicle(k+1); %vehicle speed at previous time step (previous in time, not in the simulation)
    [N_ICE_rpm_k,Trans_gear_k,wheelspeed_rads_k]=DP_transmission_operation(DP_SIM_DATA,v_k,v_kp1,Trans_gear_kp1);
    DP_SIM_DATA_updated.CYCLE.wheelspeed_rads(k)=wheelspeed_rads_k;
    DP_SIM_DATA_updated.CYCLE.Trans_gear(k)=Trans_gear_k;
    Trans_gear_kp1=Trans_gear_k;
    DP_SIM_DATA_updated.CYCLE.ICE_rpm(k)=N_ICE_rpm_k;
    DP_SIM_DATA_updated.RTM.w_RTM_rads(k)=(DP_SIM_DATA_updated.CYCLE.wheelspeed_rads(k)*DP_SIM_DATA.RTM.RTM_gear)+0.00001; %Find RTM rotational speed
    DP_SIM_DATA_updated.RTM.RTM_Eff(k)=interp1(DP_SIM_DATA.RTM.RTMSpeedIndex_rpm,DP_SIM_DATA.RTM.RTM_Avg_Eff_Table,abs(DP_SIM_DATA_updated.RTM.w_RTM_rads(k)*DP_SIM_DATA.CONST.rads_to_rpm)); %Find RTM efficiency using avg efficiency table
end

process_time_veh_operation=toc(tStart);

end