%Script to run through all 4 driving cycles and store the data from each

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%select the drive cycle you will be using, by uncommenting the corresponding number:

%selection=1; %Validation cycle with no aero
%selection=2; %validation cycle with aero losses
%selection=3; %US06  full cycle
%selection=4; %US06 city cycle
%selection=5; %US06 highway cycle
%selection=6; %UDDS cycle
%selection=7; %HWFET cycle

%Power limits set to +/- 90kW

global selection
global regime

run US06 city cycle
clear all
selection=4;
regime=1; %series
DP_Analyzer_Main
clear all
selection=4;
regime=2; %parallel
DP_Analyzer_Main

% %run US06 highway cycle
% clear all
% selection=5;
% regime=1; %series
% DP_Analyzer_Main
% clear all
% selection=5;
% regime=2; %parallel
% DP_Analyzer_Main

%run UDDS (505) cycle
% clear all
% selection=6;
% regime=1; %series
% DP_Analyzer_Main
% clear all
% selection=6;
% regime=2; %parallel
% DP_Analyzer_Main

%run HWFET cycle
% clear all
% selection=7;
% regime=1; %series
% DP_Analyzer_Main;
% clear all
% selection=7;
% regime=2; %parallel
% DP_Analyzer_Main

%post-process the data
% FC_data{1,1}=[];
% FC_data{2,1}='Fuel Consumption';
% 
% load('US06City_4Wh_Series.mat');
% FC_data{1,2}='US06City_4Wh_Series';
% FC_data{2,2}=FC_opt;
% 
% load('US06City_4Wh_Parallel.mat');
% FC_data{1,3}='US06City_4Wh_Parallel';
% FC_data{2,3}=FC_opt;
% disp('US06 City Parallel:');
% process_memory_used{1,1}
% process_memory_used{1,2}
% 
% 
% load('US06Highway_4Wh_Series.mat');
% FC_data{1,4}='US06Highway_4Wh_Series';
% FC_data{2,4}=FC_opt;
% 
% load('US06Highway_4Wh_Parallel.mat');
% FC_data{1,5}='US06Highway_4Wh_Parallel';
% FC_data{2,5}=FC_opt;
% disp('US06 Highway Parallel:');
% process_memory_used{1,1}
% process_memory_used{1,2}
% 
% load('UDDS_4Wh_Series.mat');
% FC_data{1,6}='UDDS_4Wh_Series';
% FC_data{2,6}=FC_opt;
% 
% load('UDDS_4Wh_Parallel.mat');
% FC_data{1,7}='UDDS_4Wh_Parallel';
% FC_data{2,7}=FC_opt;
% disp('UDDS Parallel:');
% process_memory_used{1,1}
% process_memory_used{1,2}
% 
% load('HWFET_4Wh_Series.mat');
% FC_data{1,8}='HWFET_4Wh_Series';
% FC_data{2,8}=FC_opt;
% 
% load('HWFET_4Wh_Parallel.mat');
% FC_data{1,9}='HWFET_4Wh_Parallel';
% FC_data{2,9}=FC_opt;
% disp('HWFET Parallel:');
% process_memory_used{1,1}
% process_memory_used{1,2}
% 
% %%%%%%%%%%%%%%%%%%%%%%%%
% %CS_FE=0.29*FE_UDDS+0.12*FE_HWFET+0.14*FE_US06_City+0.45*FE_US06_Hwy;
% CS_FE_series=0.29*FC_data{2,6}+0.12*FC_data{2,8}+0.14*FC_data{2,2}+0.45*FC_data{2,4}; %series regime 4cycle FE
% CS_FE_parallel=0.29*FC_data{2,7}+0.12*FC_data{2,9}+0.14*FC_data{2,3}+0.45*FC_data{2,5}; %parallel 4cycle FE
% CS_FE_opt=0.29*min(FC_data{2,6},FC_data{2,7})+0.12*min(FC_data{2,8},FC_data{2,9})+0.14*min(FC_data{2,2},FC_data{2,3})+0.45*min(FC_data{2,4},FC_data{2,5}); %best possible 4cycle FE
% 
