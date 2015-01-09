function [E_grid,process_times_coarse]=DP_Coarse_Search(Ts,regime,limited_regen,E_grid,E_grid_size,E_state_target,DP_SIM_DATA)

load P_Gen_opt_map

if license('test','parallel_toolbox') %if parallel computing toolbox is available, use it. 
    if matlabpool('size') == 0 % checking to see if the pool is already open
        matlabpool open 4
    else
    end
else
    disp('Parallel Computing Toolbox not available.');
end
%E_cost cell array structure:
%E_cost{1,k}=[x1,x2,x3...,xm;]; %this holds the feasible costs
%E_cost{2,k}=[gx1,gx2,gx3,...,gxm;]; %this holds the grid point values of
%feasible costs
%costs aka the offset from E_grid(1) to reference them
%E_cost{1,k}(i) codes:
%1=infeasible point
%2=P_req=0 for that k, so skip E_cost\

E_cost=cell(1,DP_SIM_DATA.CYCLE.T);
P_RTM=cell(1,DP_SIM_DATA.CYCLE.T);
P_gen=cell(1,DP_SIM_DATA.CYCLE.T);

process_times_coarse=cell(1,DP_SIM_DATA.CYCLE.T);

disp('Solving Initial Simulation Step (k=T)...');
% k=T: execute the final step
k=DP_SIM_DATA.CYCLE.T;
k
tStart=tic;
E_cost{1,k}=0;
E_cost{2,k}=E_state_target;
process_times_coarse{1,k}=toc(tStart);
disp('Done.');

disp('Starting Parallel Processing of Cost-To-Go Array (k=T-1 to k=1)...');
%k=T-1..1 - do the rest of the steps using parallel processing

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
    E_grid_par=E_grid; %create variable copies to reduce parallel processing overhead due to non-sliced arrays
    P_req_parallel=P_req;
    P_req_par=P_req_parallel(k);
    P_req_kp1_par=P_req_parallel(k+1);
    wheelspeed_par=wheelspeed_rads(k);
    Trans_gear_par=Trans_gear(k);
    ICE_rpm_par=ICE_rpm(k);
    RTM_Eff_par=RTM_Eff(k);
    RTM_speed_rads_par=RTM_speed_rads(k);
    P_Gen_opt_map_par=P_Gen_opt_map;
    
    %preallocate
    E_cost_par=zeros(E_grid_size,E_grid_size,'double'); %temp array for this iteration, to hold raw data.
    P_RTM_par=zeros(E_grid_size,E_grid_size,'double'); %temp array for this iteration, to hold raw data.
    P_gen_par=zeros(E_grid_size,E_grid_size,'double'); %temp array for this iteration, to hold raw data.
    
    %original loop
    %     for i=1:int32(E_grid_size) %for all steps of E[k]...
    %         E_k=E_grid_par(i); %select a value for E[k]
    %         for j=1:int32(E_grid_size) %...to all steps of E[k+1]...
    %             E_kp1=E_grid_par(j); %select a value for E[k+1]
    %             P_k=-1*(E_kp1-E_k)*(3600/Ts); %new power in W; negative indicates charging
    %             if (P_k>P_max*1.2)||(P_k<P_min*1.2)  %if the power limit is beyond the Pelec bounds, the point is infeasible.
    %                 E_cost_par(i,j)=Inf;
    %             else %otherwise do the calculations
    %                 [E_cost_par(i,j),P_RTM_par(i,j),P_gen_par(i,j)]=DP_costfunction(P_req_par,P_k,k,E_state_target,E_kp1,ICE_rpm_par,RTM_Eff_par,RTM_speed_rads_par,regime,limited_regen,P_Gen_opt_map_par,DP_SIM_DATA); %find the cost of that transition
    %             end
    %         end %end j
    %     end %end i
    
    %new matrix-based loop
    E_kp1=E_grid_par(1:E_grid_size); %select a value for E[k+1]
    for i=1:int32(E_grid_size) %for all steps of E[k]...
        E_k=E_grid_par(i); %select a value for E[k]
        P_k=(E_kp1-E_k)*-1*(3600/Ts); %new power in W; negative indicates charging
        [E_cost_par(i,:),P_RTM_par(i,:),P_gen_par(i,:)]=DP_costfunction(P_req_par,P_k,k,E_state_target,E_kp1,ICE_rpm_par,RTM_Eff_par,RTM_speed_rads_par,regime,limited_regen,P_Gen_opt_map_par,E_grid_par,DP_SIM_DATA); %find the cost of that transition
    end %end i
    
    %sort out the array
    if (E_cost_par(1)==2)
        E_cost{1,k}=2;
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
    
    clear E_cost_par P_RTM_par P_gen_par
    process_times_coarse{1,k}=toc(tStart);
    if (mod(k,101)==0)
        disp('Saving Data Segment...');
        filename=strcat('DP_storage/DPtemp',num2str(k));
        save(filename,'E_cost','P_RTM','P_gen','-v7.3');
        clear E_cost P_RTM P_gen
        E_cost=cell(1,DP_SIM_DATA.CYCLE.T);
        P_RTM=cell(1,DP_SIM_DATA.CYCLE.T);
        P_gen=cell(1,DP_SIM_DATA.CYCLE.T);
    elseif (k==DP_SIM_DATA.CYCLE.T-1)
        disp('Saving Data Segment...');
        filename=strcat('DP_storage/DPtempfinal');
        save(filename,'E_cost','P_RTM','P_gen','-v7.3');
        clear E_cost P_RTM P_gen
    else
    end
end %end k

disp('Done.');

if license('test','parallel_toolbox') %if using parallel computing toolbox...
    matlabpool close force local %close the pool after you're done
else
end


end %end coarse search




