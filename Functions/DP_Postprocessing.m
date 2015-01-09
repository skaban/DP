function [DP_SIM_DATA_updated,process_times]=DP_Postprocessing(DP_SIM_DATA,fuel_consumed_L_opt,process_times_coarse,process_times_fine,process_time_coarse_analysis,process_time_fine_analysis,process_time_veh_operation)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%some post processing of values

DP_SIM_DATA_updated=DP_SIM_DATA;

%determine vehicle driven distance
DP_SIM_DATA_updated.RESULTS.total_driven_distance_km=trapz(DP_SIM_DATA.CYCLE.v_vehicle.*DP_SIM_DATA.CONST.kmhr_to_ms)/1000; %distance in km

%determine the CS fuel consumption 
DP_SIM_DATA_updated.RESULTS.fuel_consumed_L_opt=fuel_consumed_L_opt;
DP_SIM_DATA_updated.RESULTS.FC_opt=(fuel_consumed_L_opt/DP_SIM_DATA_updated.RESULTS.total_driven_distance_km)*100;

%DP simulation times

%evaluate processing time
process_times=[process_times_coarse; process_times_fine;];
process_times{3,1}='Time to Generate Vehicle Operating Profile (s)';
process_times{3,2}=process_time_veh_operation;
process_times{3,3}='Time to Analyze Coarse Results (s)';
process_times{3,4}=process_time_coarse_analysis;
process_times{3,5}='Time to Analyze Fine Results (s)';
process_times{3,6}=process_time_fine_analysis;
process_times{3,7}='Total Simulation Time (s)';
process_times{3,8}=process_times{3,2}+process_times{3,4}+process_times{3,6}+(sum(cell2mat(process_times_coarse))/4)+(sum(cell2mat(process_times_fine))/4);

process_times{3,9}='Average Coarse Iteration Time (s)';
s=0;
for i=1:DP_SIM_DATA.CYCLE.T
   s=s+process_times{1,i};
end
process_times{3,10}=s/DP_SIM_DATA.CYCLE.T;
process_times{3,11}='Average Fine Iteration Time (s)';
s=0;
for i=1:DP_SIM_DATA.CYCLE.T
   s=s+process_times{2,i};
end
process_times{3,12}=s/DP_SIM_DATA.CYCLE.T;










% %determine optimal torque requests 
% Treq_RTM_opt_simulink=zeros(T,2);
% Treq_brake_opt_simulink=zeros(T,2);
% Treq_ICE_opt_simulink=zeros(T,2);
% N_RTM_opt=zeros(1,T);
% Trans_gear_opt=zeros(T,2);
% PS_opt=zeros(T,2); 
% ESS_CS_target_opt=zeros(T,2); 
% 
% for i=1:DP_SIM_DATA.CYCLE.T
%     ESS_CS_target_opt(i,1)=i;
%     ESS_CS_target_opt(i,2)=E_elec_opt_fine(i)/16200;
%     PS_opt(i,1)=i;
%     PS_opt(i,2)=P_RTM_opt_fine(i)/P_req(i);
%     Trans_gear_opt(i,1)=i;
%     Trans_gear_opt(i,2)=Trans_gear(i);
%     Treq_brake_opt_simulink(i,2)=(P_brake_opt_fine(i)/wheelspeed_rads(i))/DP_SIM_DATA.RTM.RTM_Gear;
%     N_RTM_opt(i)=wheelspeed_rads(i)*DP_SIM_DATA.RTM.RTM_Gear*(30/pi);
%     Treq_RTM_opt_simulink(i,1)=i;
%     Treq_RTM_opt_simulink(i,2)=P_RTM_opt_fine(i)/((N_RTM_opt(i)*(pi/30))+0.00001);
%     if Treq_RTM_opt_simulink(i,2)>250
%         Treq_RTM_opt_simulink(i,2)=250;
%     elseif Treq_RTM_opt_simulink(i)<-250
%         Treq_RTM_opt_simulink(i)=-250;   
%     end
%     N_BAS_opt(i)=N_ICE_opt(i)*BAS_belt_ratio;
%     Treq_BAS_opt(i)=P_BAS_opt_fine(i)/(N_BAS_opt(i)*(pi/30));
% end
% 
% %%%%%%%%%%%%%make these into timeseries objects for simulink use
% %Treq_RTM_opt_simulink(:,2)=Treq_RTM_opt';
% Treq_RTM_opt_simulink(isnan(Treq_RTM_opt_simulink))=0;
% %Treq_RTM_opt_simulink(:,1)=1:T;
% 
% Treq_ICE_opt_simulink(:,2)=Treq_ICE_opt';
% Treq_ICE_opt_simulink(isnan(Treq_ICE_opt))=0;
% Treq_ICE_opt_simulink(:,1)=1:T;
% 
% Treq_brake_opt_simulink(isnan(Treq_brake_opt_simulink))=0;
% Treq_brake_opt_simulink(isinf(Treq_brake_opt_simulink))=0;
% Treq_brake_opt_simulink(:,1)=1:T;
% 
% 
% Trans_gear_opt(isnan(Trans_gear_opt))=0;
% 
% PS_opt(isnan(PS_opt))=0;
% PS_opt(PS_opt<0)=0;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%now plot a bunch of figures, for science!
close all

% figure
% subplot(2,1,1)
% plot(v_vehicle(1:T))
% title('Speed (km/hr)')
% subplot(2,1,2)
% plot(E_elec_opt_fine)
% title('ESS Charge (Wh)')
% 
% figure
% subplot(5,1,1)
% plot(P_req(1:T)/1000,'g')
% axis([0 T+1 -100 100])
% title('Driver Power Req (kW)')
% subplot(5,1,2)
% plot(P_RTM_opt_fine/1000,'b')
% hold on
% plot([1,T],[50 50], '--b')
% plot([1,T],[-50 -50], '--b')
% axis([0 T+1 -60 105])
% title('EM Power Req (kW)')
% subplot(5,1,3)
% plot(P_ICE_opt_fine/1000,'r')
% axis([0 T+1 -5 100])
% title('Engine Power Req (kW)')
% subplot(5,1,4)
% plot(P_brake_opt_fine/1000,'c')
% axis([0 T+1 -100 5])
% title('Brake Power Req (kW)')
% subplot(5,1,5)
% plot(E_elec_opt_fine,'k')
% axis([0 T+1 1600 3400])
% title('ESS Charge (Wh)')


% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%picture for thesis 
% figure
% subplot(5,1,1)
% axh = gca;
% plot(v_vehicle(1:T),'k','LineWidth',2.5);
% set(gca,'fontsize',15)
% grid on;
% grid minor
% set(axh,'GridLineStyle','-');
% legend('Vehicle Speed (km/hr)');
% axis([0 T+1 -5 105])
% %title('Speed (km/hr)')
% subplot(5,1,2)
% plot(P_RTM_opt_fine/1000,'b','LineWidth',2.5)
% hold on
% plot([1,T],[50 50], '--b','LineWidth',1.5)
% plot([1,T],[-50 -50], '--b','LineWidth',1.5)
% set(gca,'fontsize',15)
% grid on;
% grid minor
% set(axh,'GridLineStyle','-');
% legend('RTM Commanded Power (kW)');
% axis([0 T+1 -55 55])
% %title('EM Power Req (kW)')
% subplot(5,1,3)
% plot(P_ICE_opt_fine/1000,'r','LineWidth',2.5)
% set(gca,'fontsize',15)
% grid on;
% grid minor
% set(axh,'GridLineStyle','-');
% legend('ICE Commanded Power (kW)');
% axis([0 T+1 -5 150])
% %title('Engine Power Req (kW)')
% subplot(5,1,4)
% plot(P_brake_opt_fine/1000,'c','LineWidth',2.5)
% set(gca,'fontsize',15)
% grid on;
% grid minor
% set(axh,'GridLineStyle','-');
% h=legend('Brake Commanded Power (kW)');
% set(h,'Location','southeast')
% axis([0 T+1 -100 5])
% %title('Brake Power Req (kW)')
% subplot(5,1,5)
% plot((E_elec_opt_fine./16200)*100,'k','LineWidth',2.5)
% axis([0 T+1 18.3 20.3])
% set(gca,'fontsize',15)
% grid on;
% grid minor
% set(axh,'GridLineStyle','-');
% h=legend('ESS SOC (%)');
% set(h,'Location','southeast')
% %title('ESS Charge (Wh)')
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%picture for thesis 
% figure
% subplot(4,1,1)
% axh = gca;
% plot(P_req,'k','LineWidth',2.5);
% set(gca,'fontsize',15)
% grid on;
% grid minor
% set(axh,'GridLineStyle','-');
% legend('Preq (kW)');
% %axis([0 T+1 -5 105])
% %title('Speed (km/hr)')
% subplot(4,1,2)
% plot(N_ICE_opt,'b','LineWidth',2.5)
% hold on
% %plot([1,T],[50 50], '--b','LineWidth',1.5)
% %plot([1,T],[-50 -50], '--b','LineWidth',1.5)
% set(gca,'fontsize',15)
% grid on;
% grid minor
% set(axh,'GridLineStyle','-');
% legend('ICE Optimal Speed (rpm)');
% %axis([0 T+1 -55 55])
% %title('EM Power Req (kW)')
% subplot(4,1,3)
% plot(Treq_ICE_opt,'r','LineWidth',2.5)
% set(gca,'fontsize',15)
% grid on;
% grid minor
% set(axh,'GridLineStyle','-');
% legend('ICE Optimal Torque (Nm)');
% %axis([0 T+1 -5 150])
% %title('EM Power Req (kW)')
% subplot(4,1,4)
% plot(fuel_power_in,'r','LineWidth',2.5)
% set(gca,'fontsize',15)
% grid on;
% grid minor
% set(axh,'GridLineStyle','-');
% legend('Fuel Power In (kW)');
% %axis([0 T+1 -5 150])
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%generate efficiency plots of ICE, BAS, and RTM, and populate them with the
%operating points of the components over the drive cycle 

% figure
% axh = gca;
% [C,h]=contour(ICESpeedIndex_rpm,ICETorqueIndex_Nm,ICE_Eff_Table,[0.1 0.2 0.25 0.275 0.3 0.32]);
% axis([800 6400 1 220])
% set(gca,'fontsize',15)
% set(axh,'GridLineStyle','-');
% clabel(C,h);
% xlabel('ICE Speed (RPM)');
% ylabel('ICE Torque (Nm)');
% hold on
% h1=plot(N_ICE_opt,Treq_ICE_opt_simulink(:,2),'o');
% delete(h1(h1==0));
% set(h1,'linewidth',3);
% legend('ICE Efficiency','ICE Operating Points');
% title('ICE Operating Points');
% 
% figure
% axh = gca;
% [C,h]=contour(RTMSpeedIndex_rpm,RTMTorqueIndex_Nm_Q12,RTM_Eff_Table_Q12,[0.6 0.7 0.8 0.825 0.85 0.875 0.9 0.925]);
% axis([0 7000 -250 250])
% set(gca,'fontsize',15);
% set(axh,'GridLineStyle','-');
% clabel(C,h);
% xlabel('RTM Speed (RPM)');
% ylabel('RTM Torque (Nm)');
% hold on
% h1=plot(N_RTM_opt,Treq_RTM_opt_simulink(:,2),'o');
% %delete(h1(h1==0));
% set(h1,'linewidth',3);
% legend('RTM Efficiency','RTM Operating Points');
% title('RTM Operating Points');









end