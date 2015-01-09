function [E_cost,P_RTM,P_gen,process_times_fine]=DP_Fine_Search(E_grid_size,Ts,regime,E_elec_opt_coarse,E_step_min_coarse,DP_SIM_DATA)

if license('test','parallel_toolbox') %if parallel computing toolbox is available, use it. 
    if matlabpool('size') == 0 % checking to see if the pool is already open
        matlabpool open 4
    else
    end
else
    disp('Parallel Computing Toolbox not available.');
end
% % Initialization 
%for fine search, the grid size is now defined at each stage of the problem, since the exact dimensions of the search
%space change based on the coarse ESS state at each consecutive stage. 

E_step_min_fine=(2*E_step_min_coarse)/E_grid_size; %determine maximum step size of the state variable for fine analysis.%E_grid is generated at each stage for the fine simulation.
E_step_boundary=E_step_min_coarse; %define once to speed up computation. 

E_cost=cell(1,DP_SIM_DATA.CYCLE.T);
P_RTM=cell(1,DP_SIM_DATA.CYCLE.T);
P_gen=cell(1,DP_SIM_DATA.CYCLE.T);
process_times_fine=cell(1,DP_SIM_DATA.CYCLE.T);

%% Main DP Function

%k=T
k=DP_SIM_DATA.CYCLE.T;
k
tStart=tic;
E_cost{1,k}=0;
P_RTM{1,k}=0;
%E_cost_rolledforward(1,:,k)=0;
%E_cost_rolledforward(2,:,k)=E_state_target_fine; %at stage k, want to end up at target grid point.
P_gen{1,k}=0;
process_times_fine{1,k}=toc(tStart);

disp('Starting Parallel Processing of Fine Cost-To-Go Array...');
%k=T-2..1 - do the rest of the steps in parallel!

load P_Gen_opt_map
P_Gen_opt_map=cell2mat(P_Gen_opt_map);
P_req=DP_SIM_DATA.CYCLE.P_req;
ICE_rpm=DP_SIM_DATA.CYCLE.ICE_rpm;
RTM_Eff=DP_SIM_DATA.RTM.RTM_Eff;
Trans_gear=DP_SIM_DATA.CYCLE.Trans_gear;
wheelspeed_rads=DP_SIM_DATA.CYCLE.wheelspeed_rads;
P_max=DP_SIM_DATA.RTM.P_max;
P_min=DP_SIM_DATA.RTM.P_min;
RTM_speed_rads=DP_SIM_DATA.RTM.w_RTM_rads;

for k=1:(DP_SIM_DATA.CYCLE.T-1) %loop from the 2nd-to-last sample of the drive back to the start...
    k
    tStart=tic;
    %generate new SOC state search grid
    P_req_temp=P_req;
    P_req_par=P_req_temp(k);
    P_req_kp1_par=P_req_temp(k+1);
    wheelspeed_par=wheelspeed_rads(k);
    Trans_gear_par=Trans_gear(k);
    ICE_rpm_par=ICE_rpm(k);
    E_elec_opt_coarse_par=E_elec_opt_coarse;
    
    Ek=E_elec_opt_coarse_par(k); %current coarse E state
    Ekp1=E_elec_opt_coarse_par(k+1); %previous coarse E state
    
    E_grid_k=Ek-(E_step_boundary):E_step_min_fine:Ek+(E_step_boundary);   
    E_grid_kp1=Ekp1-(E_step_boundary):E_step_min_fine:Ekp1+(E_step_boundary); 
    
    Ek_grid_size=E_grid_size;
    Ekp1_grid_size=E_grid_size;
    
    E_cost_par=zeros(E_grid_size,E_grid_size,'double'); %temp array for this iteration, to hold raw data.
    P_RTM_par=zeros(E_grid_size,E_grid_size,'double');
    P_gen_par=zeros(E_grid_size,E_grid_size,'double');
    
    if Ek>Ekp1
        P_step=abs((min(E_grid_kp1)-max(E_grid_k))*3600/E_grid_size);
    else
        P_step=abs((min(E_grid_k)-max(E_grid_kp1))*3600/E_grid_size);
    end
    
    E_kp1=E_grid_kp1(1:Ekp1_grid_size); %select a value for E[k+1]
    for i=1:Ek_grid_size %from all steps of E[k]...
        E_k=E_grid_k(i); %select a value for E[k].
        P_k=(E_kp1-E_k)*-1*(3600/Ts); %new power in W; negative indicates charging
        [E_cost_par(i,:),P_RTM_par(i,:),P_gen_par(i,:)]=DP_costfunction_fine(P_req_par,P_k,P_min,P_max,wheelspeed_par,Trans_gear_par,ICE_rpm_par,regime,P_step,DP_SIM_DATA); %transition cost
    end %end i for
    
    %sort out the array
    if (E_cost_par(1)==2)
        E_cost{1,k}=2;
        P_RTM{1,k}=0;
        P_gen{1,k}=0;
    else
        %convert E_cost_par and P_RTM_par to sparse arrays, to save memory:
        %sparse arrays need to have mostly 0's, so swap 0's for 69 and
        %Infs for 0's, then convert.
        E_cost_par(E_cost_par==0)=69; %swap 0's
        E_cost_par(E_cost_par==Inf)=0;
        E_cost_par=sparse(E_cost_par);
        nzmax1=nnz(E_cost_par);
        
        P_RTM_par(P_RTM_par==0)=69; %swap 0's 
        P_RTM_par(P_RTM_par==Inf)=0;
        P_RTM_par=sparse(P_RTM_par);    
        nzmax2=nnz(P_RTM_par);
        
        P_gen_par(P_gen_par==0)=69; %swap 0's 
        P_gen_par(P_gen_par==Inf)=0;
        P_gen_par=sparse(P_gen_par);    
        nzmax3=nnz(P_gen_par);
        
        %preallocate the next cell of E_cost, and fill the cell with the appropriate
        %processed cost and RTM values
        sparse_size=double(E_grid_size);
        E_cost{1,k}=spalloc(sparse_size,sparse_size,nzmax1);
        E_cost{1,k}=E_cost_par;
        P_RTM{1,k}=spalloc(sparse_size,sparse_size,nzmax2);
        P_RTM{1,k}=P_RTM_par;
        P_gen{1,k}=spalloc(sparse_size,sparse_size,nzmax3);
        P_gen{1,k}=P_gen_par;
    end
    process_times_fine{1,k}=toc(tStart);
end %end k

disp('Done.');

if license('test','parallel_toolbox') %if using parallel computing toolbox...
    matlabpool close force local %close the pool after you're done
else
end


end %end function

















