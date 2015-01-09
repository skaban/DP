%this subroutine selects which driving cycle to use, based on the number
%passed.

function [drive_cycle_speed_data,drive_cycle_power_data,T,distance]=drive_cycle_select(Ts,selected_cycle)

kmhr_to_ms=0.277778; %conversion factor to go from km/hr to m/s

switch selected_cycle
    
    case 1 %Set up the simulation to use the Validation cycle for Simulink Model
        load Validation_cycle_data.mat;
        load P_req_validation_simulink.mat;
        
        F0=21.77; %malibu road load values
        F1=0.3673;	
        F2=0.01847;
        m=2500;                %Vehicle mass, kg   (1325kg curb weight)
        r=18*0.0254; %wheel radius in m, converted from inches
        T=length(v_vehicle);
        validation_cycle_ms=v_vehicle.*kmhr_to_ms;
        Power_demand = nan(1,T);
        Wreq_intp =  nan(1,T);
        Treq_intp =  nan(1,T);
        accel=nan(1,T);
        FF=nan(1,T);
        distance = 0;
        
        for i=1:length(P_wheels_W)
            if mod(i,1000)==0
                Power_demand(i/1000) = P_wheels_W(i); %W
            else
            end
        end
        
        for i=1:T-1
            accel(i)=(validation_cycle_ms(i+1)-validation_cycle_ms(i))/Ts;
            if validation_cycle_ms(i)==0 && accel(i)==0
                FF(i) = 0;
            else
                FF(i)=(F0+F1*validation_cycle_ms(i)+F2*validation_cycle_ms(i)^2)+(m*accel(i));
            end
            % Torque and speed demand under static conditions
            Wreq_intp(i) = validation_cycle_ms(i)/r;  % rad/s
            
            Treq_intp(i) = Power_demand(i)/Wreq_intp(i);  % Nm
            
            distance = distance + validation_cycle_ms(i)*Ts;
        end
        drive_cycle_speed_data=double(v_vehicle);
        drive_cycle_power_data=Power_demand;
        drive_cycle_power_data(isnan(drive_cycle_power_data))=0;
        drive_cycle_power_data(1:5)=0;
        drive_cycle_power_data(65)=0;
        drive_cycle_power_data(64)=0;
        clear P_wheels_W
        
        figure
        hold on;
        plot(v_vehicle);  % vehicle speed(km/hr)
        plot(drive_cycle_power_data/1000,'r');
        plot(Treq_intp,'k');
        grid on;
        legend('vehicle speed(km/hr)','Power Demand(kW)','Treq(Nm)');
        axis([0 T+1 -175 175])
        
        figure
        subplot(3,1,1)
        plot(v_vehicle);  % vehicle speed(km/hr)
        legend('vehicle speed(km/hr)');
        grid on
        axis([0 T+1 -5 105])
        subplot(3,1,2)
        plot(drive_cycle_power_data/1000,'r');
        legend('Power Demand(kW)');
        grid on
        axis([0 T+1 -100 100])
        subplot(3,1,3)
        plot(Treq_intp,'k');
        legend('Treq(Nm)');
        grid on
        axis([0 T+1 -1750 1750])
        
    case 2
        
        %% Plot the vehicle speed(m/s)
        %F0 = 88.6000; %jackie's original values
        %F1 = 1.3900;
        %F2 = 0.3600;
        
        %Validation_cycle_kmhr=[0 0 0 0 0 5 10 15 20 25 30 35 40 45 50 55 60 65 70 75 80 85 90 95 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 95 90 85 80 75 70 65 60 55 50 45 40 35 30 25 20 15 10 5 0 0 0 0];
        if Ts==1
            load Validation_cycle_data.mat;
        elseif Ts==0.5
            load Validation_cycle_Ts0p5s_data.mat;
        else
        end
        F0=21.77; %malibu road load values
        F1=0.3673;	
        F2=0.01847;
        m=2500;                %Vehicle mass, kg   (1325kg curb weight)
        r=18*0.0254; %wheel radius in m, converted from inches
        T=length(v_vehicle);
        validation_cycle_ms=v_vehicle.*kmhr_to_ms;
        Power_demand = nan(1,T);
        Wreq_intp =  nan(1,T);
        Treq_intp =  nan(1,T);
        accel=nan(1,T);
        FF=nan(1,T);
      
        distance = 0;
        for i=1:T-1
            accel(i)=(validation_cycle_ms(i+1)-validation_cycle_ms(i))/Ts;
            if validation_cycle_ms(i)==0 && accel(i)==0
                FF(i) = 0;
            else
                FF(i)=(F0+F1*validation_cycle_ms(i)+F2*validation_cycle_ms(i)^2)+(m*accel(i));
            end
            % Torque and speed demand under static conditions
            Wreq_intp(i) = validation_cycle_ms(i)/r;  % rad/s
            Treq_intp(i) = FF(i)*r;  % Nm
            Power_demand(i) = Wreq_intp(i)*Treq_intp(i); %W
            distance = distance + validation_cycle_ms(i)*Ts;
        end
        
        drive_cycle_speed_data=double(v_vehicle);
        drive_cycle_power_data=Power_demand;
        drive_cycle_power_data(isnan(drive_cycle_power_data))=0;
        
%         figure
%         hold on;
%         plot(Validation_cycle_kmhr);  % vehcile speed(m/s)
%         plot(Power_demand/1000,'r');
%         plot(Treq_intp,'k');
%         grid on;
%         legend('vehicle speed(km/hr)','Power Demand(kW)','interpolated Treq(N)/20');
%         axis([0 T+1 -175 175])
        
%         
%         figure
%         subplot(3,1,1)
%         axh = gca;
%         plot(Validation_cycle_kmhr,'LineWidth',2.5);  % vehicle speed(km/hr)
%         set(gca,'fontsize',15)
%         grid on;
%         grid minor
%         set(axh,'GridLineStyle','-');
%         legend('Vehicle Speed (km/hr)');
%         axis([0 T+1 -5 105])
%         subplot(3,1,2)
%         axh = gca;
%         plot(Power_demand/1000,'r','LineWidth',2.5);
%         set(gca,'fontsize',15)
%         grid on;
%         grid minor
%         set(axh,'GridLineStyle','-');
%         legend('Driver Power Demand (kW)');
%         axis([0 T+1 -100 100])
%         subplot(3,1,3)
%         axh = gca;
%         plot(Treq_intp,'k','LineWidth',2.5);
%         set(gca,'fontsize',15)
%         grid on;
%         grid minor
%         set(axh,'GridLineStyle','-');
%         legend('Driving Torque at Wheels (Nm)');
%         axis([0 T+1 -1750 1750])
%         xlabel('Time (s)')
        
    case 3 %Set up the simulation to use the US06 Ts=0.5s City cycle
        load US06_City_data_Ts_0p5s.mat;
        drive_cycle_speed_data=v_vehicle;
        T=length(drive_cycle_speed_data);        
        F0=21.77; %malibu road load values
        F1=0.3673;	
        F2=0.01847;
        m=2500;                %Vehicle mass, kg   (1325kg curb weight)
        r=18*0.0254; %wheel radius in m, converted from inches
        validation_cycle_ms=drive_cycle_speed_data.*kmhr_to_ms;
        Ts=1;
        Power_demand = nan(1,T);
        Wreq_intp =  nan(1,T);
        Treq_intp =  nan(1,T);
        accel=nan(1,T);
        FF=nan(1,T);
        
        distance = 0;
        for i=1:T-1
            accel(i)=(validation_cycle_ms(i+1)-validation_cycle_ms(i))/Ts;
            if validation_cycle_ms(i)==0 && accel(i)==0
                FF(i) = 0;
            else
                FF(i)=(F0+F1*validation_cycle_ms(i)+F2*validation_cycle_ms(i)^2)+(m*accel(i));
            end
            % Torque and speed demand under static conditions
            Wreq_intp(i) = validation_cycle_ms(i)/r;  % rad/s
            Treq_intp(i) = FF(i)*r;  % Nm
            Power_demand(i) = Wreq_intp(i)*Treq_intp(i); %W
            distance = distance + validation_cycle_ms(i)*Ts;
        end
        
        drive_cycle_power_data=Power_demand;
        drive_cycle_power_data(isnan(drive_cycle_power_data))=0;
      
        
    case 4 %Set up the simulation to use the US06 city cycle.
        if Ts==1
            load US06_City_data.mat;
        elseif Ts==0.5
            load US06_City_data_Ts_0p5s.mat;
        else
        end
        drive_cycle_speed_data=v_vehicle;
        T=length(drive_cycle_speed_data);
        F0=21.77; %malibu road load values
        F1=0.3673;
        F2=0.01847;
        
        m=2500;                %Vehicle mass, kg   (1325kg curb weight)
        r=18*0.0254; %wheel radius in m, converted from inches
        validation_cycle_ms=drive_cycle_speed_data.*kmhr_to_ms;
        Power_demand = nan(1,T);
        Wreq_intp =  nan(1,T);
        Treq_intp =  nan(1,T);
        accel=nan(1,T);
        FF=nan(1,T);
        
        distance = 0;
        for i=1:T-1
            accel(i)=(validation_cycle_ms(i+1)-validation_cycle_ms(i))/Ts;
            if validation_cycle_ms(i)==0 && accel(i)==0
                FF(i) = 0;
            else
                FF(i)=(F0+F1*validation_cycle_ms(i)+F2*validation_cycle_ms(i)^2)+(m*accel(i));
            end
            % Torque and speed demand under static conditions
            Wreq_intp(i) = validation_cycle_ms(i)/r;  % rad/s
            Treq_intp(i) = FF(i)*r;  % Nm
            Power_demand(i) = Wreq_intp(i)*Treq_intp(i); %W
            distance = distance + validation_cycle_ms(i)*Ts;
        end
        
        drive_cycle_power_data=Power_demand;
        drive_cycle_power_data(isnan(drive_cycle_power_data))=0;
        
%         figure
%         subplot(3,1,1)
%         axh = gca;
%         plot(drive_cycle_speed_data,'LineWidth',2.5);  % vehicle speed(km/hr)
%         set(gca,'fontsize',15)
%         grid on;
%         grid minor
%         set(axh,'GridLineStyle','-');
%         legend('Vehicle Speed (km/hr)');
%         axis([0 T+1 -5 130])
%         subplot(3,1,2)
%         axh = gca;
%         plot(Power_demand/1000,'r','LineWidth',2.5);
%         set(gca,'fontsize',15)
%         grid on;
%         grid minor
%         set(axh,'GridLineStyle','-');
%         legend('Driver Power Demand (kW)');
%         axis([0 T+1 -100 150])
%         subplot(3,1,3)
%         axh = gca;
%         plot(Treq_intp,'k','LineWidth',2.5);
%         set(gca,'fontsize',15)
%         grid on;
%         grid minor
%         set(axh,'GridLineStyle','-');
%         legend('Driving Torque at Wheels (Nm)');
%         axis([0 T+1 -3000 4250])
%         xlabel('Time (s)')
        
    case 5 %Set up the simulation to use the US06 highway cycle.
        if Ts==1
            load US06_Highway_data.mat;
        elseif Ts==0.5
            load US06_Highway_data_Ts_0p5s.mat;
        end
        drive_cycle_speed_data=v_vehicle;
        T=length(drive_cycle_speed_data);
        F0=21.77; %malibu road load values
        F1=0.3673;
        F2=0.01847;
        m=2500;                %Vehicle mass, kg   (1325kg curb weight)
        r=18*0.0254; %wheel radius in m, converted from inches
        validation_cycle_ms=drive_cycle_speed_data.*kmhr_to_ms;
        Power_demand = nan(1,T);
        Wreq_intp =  nan(1,T);
        Treq_intp =  nan(1,T);
        accel=nan(1,T);
        FF=nan(1,T);
        
        distance = 0;
        for i=1:T-1
            accel(i)=(validation_cycle_ms(i+1)-validation_cycle_ms(i))/Ts;
            if validation_cycle_ms(i)==0 && accel(i)==0
                FF(i) = 0;
            else
                FF(i)=(F0+F1*validation_cycle_ms(i)+F2*validation_cycle_ms(i)^2)+(m*accel(i));
            end
            % Torque and speed demand under static conditions
            Wreq_intp(i) = validation_cycle_ms(i)/r;  % rad/s
            Treq_intp(i) = FF(i)*r;  % Nm
            Power_demand(i) = Wreq_intp(i)*Treq_intp(i); %W
            distance = distance + validation_cycle_ms(i)*Ts;
        end
        
        drive_cycle_power_data=Power_demand;
        drive_cycle_power_data(isnan(drive_cycle_power_data))=0;
        
%         figure
%         subplot(3,1,1)
%         axh = gca;
%         plot(drive_cycle_speed_data,'LineWidth',2.5);  % vehicle speed(km/hr)
%         set(gca,'fontsize',15)
%         grid on;
%         grid minor
%         set(axh,'GridLineStyle','-');
%         legend('Vehicle Speed (km/hr)');
%         axis([0 T+1 -5 130])
%         subplot(3,1,2)
%         axh = gca;
%         plot(Power_demand/1000,'r','LineWidth',2.5);
%         set(gca,'fontsize',15)
%         grid on;
%         grid minor
%         set(axh,'GridLineStyle','-');
%         legend('Driver Power Demand (kW)');
%         axis([0 T+1 -100 150])
%         subplot(3,1,3)
%         axh = gca;
%         plot(Treq_intp,'k','LineWidth',2.5);
%         set(gca,'fontsize',15)
%         grid on;
%         grid minor
%         set(axh,'GridLineStyle','-');
%         legend('Driving Torque at Wheels (Nm)');
%         axis([0 T+1 -3000 4250])
%         xlabel('Time (s)')

    case 6 %Set up the simulation to use the US06 highway cycle.
        load UDDS_data.mat;
        drive_cycle_speed_data=v_vehicle;
        T=length(drive_cycle_speed_data);
        F0=21.77; %malibu road load values
        F1=0.3673;
        F2=0.01847;
        m=2500;                %Vehicle mass, kg   (1325kg curb weight)
        r=18*0.0254; %wheel radius in m, converted from inches
        validation_cycle_ms=drive_cycle_speed_data.*kmhr_to_ms;
        Power_demand = nan(1,T);
        Wreq_intp =  nan(1,T);
        Treq_intp =  nan(1,T);
        accel=nan(1,T);
        FF=nan(1,T);
        
        distance = 0;
        for i=1:T-1
            accel(i)=(validation_cycle_ms(i+1)-validation_cycle_ms(i))/Ts;
            if validation_cycle_ms(i)==0 && accel(i)==0
                FF(i) = 0;
            else
                FF(i)=(F0+F1*validation_cycle_ms(i)+F2*validation_cycle_ms(i)^2)+(m*accel(i));
            end
            % Torque and speed demand under static conditions
            Wreq_intp(i) = validation_cycle_ms(i)/r;  % rad/s
            Treq_intp(i) = FF(i)*r;  % Nm
            Power_demand(i) = Wreq_intp(i)*Treq_intp(i); %W
            distance = distance + validation_cycle_ms(i)*Ts;
        end
        
        drive_cycle_power_data=Power_demand;
        drive_cycle_power_data(isnan(drive_cycle_power_data))=0;
        
%         figure
%         subplot(3,1,1)
%         axh = gca;
%         plot(drive_cycle_speed_data,'LineWidth',2.5);  % vehicle speed(km/hr)
%         set(gca,'fontsize',15)
%         grid on;
%         grid minor
%         set(axh,'GridLineStyle','-');
%         legend('Vehicle Speed (km/hr)');
%         axis([0 T+1 -5 130])
%         subplot(3,1,2)
%         axh = gca;
%         plot(Power_demand/1000,'r','LineWidth',2.5);
%         set(gca,'fontsize',15)
%         grid on;
%         grid minor
%         set(axh,'GridLineStyle','-');
%         legend('Driver Power Demand (kW)');
%         axis([0 T+1 -100 150])
%         subplot(3,1,3)
%         axh = gca;
%         plot(Treq_intp,'k','LineWidth',2.5);
%         set(gca,'fontsize',15)
%         grid on;
%         grid minor
%         set(axh,'GridLineStyle','-');
%         legend('Driving Torque at Wheels (Nm)');
%         axis([0 T+1 -3000 4250])
%         xlabel('Time (s)')

case 7 %Set up the simulation to use the US06 highway cycle.
        load HWFET_data.mat;
        drive_cycle_speed_data=v_vehicle;
        
        T=length(drive_cycle_speed_data);
        F0=21.77; %malibu road load values
        F1=0.3673;
        F2=0.01847;
        
        m=2500;                %Vehicle mass, kg   (1325kg curb weight)
        r=18*0.0254; %wheel radius in m, converted from inches
        validation_cycle_ms=drive_cycle_speed_data.*kmhr_to_ms;
        Power_demand = nan(1,T);
        Wreq_intp =  nan(1,T);
        Treq_intp =  nan(1,T);
        accel=nan(1,T);
        FF=nan(1,T);
        
        distance = 0;
        for i=1:T-1
            accel(i)=(validation_cycle_ms(i+1)-validation_cycle_ms(i))/Ts;
            if validation_cycle_ms(i)==0 && accel(i)==0
                FF(i) = 0;
            else
                FF(i)=(F0+F1*validation_cycle_ms(i)+F2*validation_cycle_ms(i)^2)+(m*accel(i));
            end
            % Torque and speed demand under static conditions
            Wreq_intp(i) = validation_cycle_ms(i)/r;  % rad/s
            Treq_intp(i) = FF(i)*r;  % Nm
            Power_demand(i) = Wreq_intp(i)*Treq_intp(i); %W
            distance = distance + validation_cycle_ms(i)*Ts;
        end
        
        drive_cycle_power_data=Power_demand;
        drive_cycle_power_data(isnan(drive_cycle_power_data))=0;
        
%         figure
%         subplot(3,1,1)
%         axh = gca;
%         plot(drive_cycle_speed_data,'LineWidth',2.5);  % vehicle speed(km/hr)
%         set(gca,'fontsize',15)
%         grid on;
%         grid minor
%         set(axh,'GridLineStyle','-');
%         legend('Vehicle Speed (km/hr)');
%         axis([0 T+1 -5 130])
%         subplot(3,1,2)
%         axh = gca;
%         plot(Power_demand/1000,'r','LineWidth',2.5);
%         set(gca,'fontsize',15)
%         grid on;
%         grid minor
%         set(axh,'GridLineStyle','-');
%         legend('Driver Power Demand (kW)');
%         axis([0 T+1 -100 150])
%         subplot(3,1,3)
%         axh = gca;
%         plot(Treq_intp,'k','LineWidth',2.5);
%         set(gca,'fontsize',15)
%         grid on;
%         grid minor
%         set(axh,'GridLineStyle','-');
%         legend('Driving Torque at Wheels (Nm)');
%         axis([0 T+1 -3000 4250])
%         xlabel('Time (s)')

    otherwise
        disp('Invalid selection, try again.');
        drive_cycle_speed_data=0;
        drive_cycle_power_data=0;
        T=0;


end