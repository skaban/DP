%this function handles gear shift stuff
function [ICE_speed,newgear,wheelspeed_rads] = transmission_operation(DP_SIM_DATA,v_k,v_km1,Trans_gear)

N_wheel_k=double(v_k)*(1/3.6)*DP_SIM_DATA.CONST.wheel_conversion_const*DP_SIM_DATA.CONST.rads_to_rpm; %convert current vehicle speed to wheel speed in rpm
N_wheel_km1=double(v_km1)*(1/3.6)*DP_SIM_DATA.CONST.wheel_conversion_const*DP_SIM_DATA.CONST.rads_to_rpm; %convert previous vehicle speed to wheel speed in rpm
wheelspeed_rads=double(v_k)*(1/3.6)*DP_SIM_DATA.CONST.wheel_conversion_const;

if Trans_gear<1
    ratio=0;
else
    ratio=DP_SIM_DATA.Trans.gear_ratios(Trans_gear)*DP_SIM_DATA.Trans.front_diff_ratio; %determine current gear ratio between ICE and wheels
end

ICE_speed_k=N_wheel_k*ratio; %Find the current and previous engine speed, in RPM
ICE_speed_km1=N_wheel_km1*ratio;

if(ICE_speed_km1>DP_SIM_DATA.Trans.upshift_threshold)&&(Trans_gear<6) %shift up if above threshold and < gear 6
    newgear=Trans_gear+1;
    ICE_speed=N_wheel_km1*DP_SIM_DATA.Trans.gear_ratios(newgear)*DP_SIM_DATA.Trans.front_diff_ratio; %new engine speed
elseif(ICE_speed_km1>DP_SIM_DATA.Trans.upshift_threshold)&&(Trans_gear==6) %can't shift up if in gear 6, so just stay there
    newgear=6;
    ICE_speed=N_wheel_km1*DP_SIM_DATA.Trans.gear_ratios(newgear)*DP_SIM_DATA.Trans.front_diff_ratio; %new engine speed
elseif(ICE_speed_km1<DP_SIM_DATA.Trans.downshift_threshold)&&(Trans_gear>1) %shift down if below threshold and > gear 1
    newgear=Trans_gear-1;
    ICE_speed=N_wheel_km1*DP_SIM_DATA.Trans.gear_ratios(newgear)*DP_SIM_DATA.Trans.front_diff_ratio; %new engine speed
elseif(ICE_speed_km1<1)&&(ratio>0) %shift into neutral if speed is low
    newgear=0;
    ICE_speed=DP_SIM_DATA.ICE.idle_speed; %engine idle speed
elseif(v_km1>1)&&(ratio<1) %shift out of neutral to 1st when at a slow speed
    newgear=1;
    ICE_speed=N_wheel_km1*DP_SIM_DATA.Trans.gear_ratios(newgear)*DP_SIM_DATA.Trans.front_diff_ratio; %new engine speed
elseif(v_km1<1)&&(ratio<1) %if the car is just sitting, stay in neutral
    newgear=0;
    ICE_speed=DP_SIM_DATA.ICE.idle_speed;
else %otherwise don't change gears. 
    newgear=Trans_gear;
    ICE_speed=ICE_speed_km1;
end



end