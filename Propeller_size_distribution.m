clear all
clc
syms x ;


w = 1000; %weigth(g)
N_m = 4; %number of motor
%T_req = w/N_m; % Thrust per motor
S_factor = 2.2; % safety factor
%T_max = T_req*S_factor ; %Max thrust req
Vc = 11 ;%V cruise (m/s)
Propeller_size_distribution_box
Batt = importdata('Batteries_test.txt'); %import batteries data from text file
Motor = importdata('MotorSpec_test.txt'); %import Motors data from text file
Propsize = importdata('Propsize.txt'); %import a number of propeller's data ,diameter and picth

Bsize = size(Batt.data,2); % number of Batteries 
Msize = size(Motor.data,2); % number of motors
Psize = size(Propsize.data,1); %number of propellers
show = zeros(1,Psize);



for i= 1 :Bsize
    Cellcheck = Batt.data(1,i);
    for ii = 1:Msize
        Mcellmin = Motor.data(2,ii); %lowest cell for motor
        Mcellmax = Motor.data(3,ii);%highest cell for motor
        Mpowermax = Motor.data(6,ii); %motor's max power (W)
        if (Mcellmin<=Cellcheck)&&(Cellcheck<=Mcellmax)
        else
         continue   %skip motor that don't match with battery
        end
        fprintf('\nCalculating \nBatter :%s \nMotor :%s\n',Batt.textdata{i},Motor.textdata{ii});
        v = Cellcheck*3.7 ; %Voltage from battery
        kv = Motor.data(1,ii);% Motor's kv
        M_maxrpm = v*kv; %Motor's maximun RPM
        %clear data box for new loop
        show(:) = 0;
        size_box(:,:,3) = 0;
        
        for Ri = 1: size(size_box,1)
            for Ci = 1 : size(size_box,2)
                
                if (Ri == 10)&&(Ci == 10)
                    size_box(Ri,Ci,3) = max(max(size_box(:,:,3)));
                    continue 
                end
                
                Pdia = size_box(Ri,Ci,1); %Propeller's diameter
                Ppicth = size_box(Ri,Ci,2); %Propeller's picth
                [T2RPM_v0,RPM2P_v0,T2RPM_vc,RPM2P_vc,Max_Tv0,Max_Tvc,RPMlimit] = fitprop(Pdia,Ppicth,Vc);
                AUW = w + Batt.data(4,i)+ (4*Motor.data(9,ii));
                Tmax = (((AUW/N_m)/1000*9.81*0.224809)*S_factor);
                RPMmax = T2RPM_v0(Tmax); %find a rpm for generate Thrust max
                Pmax = (RPM2P_v0(RPMmax))*745.7; %find a Max power from rpm max (Hp to W)
                if RPMmax >= M_maxrpm %check motor max rpm
                  err = 1;
              elseif Tmax > Max_Tv0 %check Max Thrust
                  err = 1;
              elseif Pmax > Mpowermax %check that can motor use with propeller
                  err = 1;
              else
                  err = 0;
                end
                if err == 1 %Set high value
                    fprintf('\nErr Prop: %d * %d\n',Pdia,Ppicth);
                    EPower_hover = Pmax *2;
                else
                    Treq_hover = (Tmax)/S_factor ;
                    RPMhover = T2RPM_v0(Treq_hover);
                    Phover = (RPM2P_v0(RPMhover))*745.7;%(Hp to W)
                    Mi0 = Motor.data(5,ii); % motor's No-load current (A)
                    MR = Motor.data(4,ii); % motor's Internal Resistance (m-ohm)
                    Eff_motor = inline('(1-((Mi0*MR/1000)/(v-(x/kv))))*(x/(v*kv))'); %motor's eff function
                    x = RPMhover;
                    EPower_hover = Phover/(Eff_motor(MR,Mi0,kv,v,x));%find eletrict power for hover
               %    Cap = (Batt.data (2,i))*0.8; %battery's Capacity  (mAh) *0.8 mean that max dischart = 80% of total capacity
               %    Endurance = 60/1000*Cap*v/(N_m*EPower_hover);
                end
                
                size_box(Ri,Ci,3) = round(EPower_hover);   
                
            end
        end
        
        figure(1)
        clf %clear figure
        Max = max(max(size_box(:,:,3)))+1; %+1 for round down error
        Min = min(min(size_box(:,:,3)));
        cm = colormap(parula(Max-Min)); %make color map from min & max P
        hold on
        for ri = 1:size(size_box,1)
            for ci = 1:size(size_box,2)
                show(ri,ci) = stem3(ri,ci,size_box(ri,ci,3)); % plot 
                cm_idx = max( size_box(ri,ci,3)-Min, 1 );
                set(show(ri,ci), 'Color',cm(cm_idx,:), 'MarkerFaceColor',cm(cm_idx,:), 'MarkerEdgeColor',cm(cm_idx,:))
            end
        end
        hold off
        colorbar
        grid on
        xlim([0 11])%axes limite
        ylim([0 11])
        set(gca,'xtick',[0:1:11]) %grid spacing
        set(gca,'ytick',[0:1:11])
        view(30,30) %plot perspective
        title(['Battery : ' Batt.textdata{i} ' Motor : ' Motor.textdata{ii} ' -Prop size Dis'])
        xlabel('Propeller Diameter')
        ylabel('Propeller Picth')
        zlabel('Power')
        pause
        view(0,90)
        pause
       
    end
end