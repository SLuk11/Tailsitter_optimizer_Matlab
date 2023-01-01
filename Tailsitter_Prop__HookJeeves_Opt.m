clear all
clc
syms x ;

fprintf('Preliminary Design for VTOL Tailsitter UAV\n');
Wp = input('Your payload weight (g) = '); %input a payload's weight
Bmax = input('Your tailsitters span (m) = '); %input a uav's span
Bpc = 0;
Bcruise = Bmax-Bpc; %find int span for cruise (Seperate Span behind propellers out of Span)
Mission = 0;
while Mission == 0 %loop for setting parameiter Vc Hover ratio
    fprintf('\nSelect the following number that are a type of mission \n');
    fprintf('1: Mapping Mission\n2: Deliverly Mission\n3: Rescu Mission\n4: Custom your mission');%Define a mission for collect cruise speed and Hover time to cruise time ratio
    Mission = input('\nMission No. : ');
    if Mission == 1
        fprintf('\nMission No.1: Mapping Mission\nCruise speed 12 m/s\n0.1 Ratio of hover time to flight time\n');
        Vc = 12;
        hover_time = 0.1;
        cruise_time =  0.9;
    elseif Mission == 2
        fprintf('\nMission No.2: Deliverly Mission\nCruise speed 20 m/s\n0.3 Ratio of hover time to flight time\n');
        Vc = 20;
        hover_time = 0.3;
        cruise_time =  0.7;
    elseif Mission == 3
        fprintf('\nMission No.3: Rescu Mission\nCruise speed 15 m/s\n0.5 Ratio of hover time to flight time\n');
        Vc = 15;
        hover_time = 0.5;
        cruise_time =  0.5;
    elseif Mission == 4
        Vc = input('\nCruise speed (m/s) = ');
        Ratio = input('Ratio of hover time to flight time (0.0 - 1.0) = '); %Custom a ratio H/C in a mission
        hover_time = Ratio;
        cruise_time =  1-Ratio;
        fprintf('\nMission No.4: Custom Mission\nCruise speed %g m/s\n%g Ratio of hover time to flight time\n',Vc,Ratio);
    else
        fprintf('\n\nPlease select a Number of mission\n\n');
        Mission = 0;
    end
end
tic % start timer
N_m = 2; %number of motor
%T_req = w/N_m; % Thrust per motor
S_factor = 2.2; % safety factor
%T_max = T_req*S_factor ; %Max thrust req
V0 = 0;
Prop_distribution_box %Call Propeller size
Batt = importdata('Batteries.txt'); %import batteries data from text file
Motor = importdata('MotorSpec.txt'); %import Motors data from text file
Propsize = importdata('Propsize.txt'); %import a number of propeller's data ,diameter and picth

Bsize = size(Batt.data,2); % number of Batteries
Msize = size(Motor.data,2); % number of motors
Psize = size(Propsize.data,1); %number of propellers

%initial for Optimize
X0 = [ 3 3 ; 5 9 ; 11 11 ]; %coordinate for initial
step = 1; %step size
Flight_time = zeros(5,7); %endurance initial
Spec = zeros(5,7);

for i= 1:Bsize
    Cellcheck = Batt.data(1,i); %Call Battery cell
    w_batt = Batt.data(4,i); %Call Battery Max Watt
    Cap = (Batt.data (2,i))*0.8; %battery's Capacity  (mAh) *0.8 mean that max dischart = 80% of total capacity
    for ii = 1:Msize
        Mcellmin = Motor.data(2,ii); %lowest cell for motor
        Mcellmax = Motor.data(3,ii);%highest cell for motor
        Mpowermax = Motor.data(6,ii); %motor's max power (W)
        if (Mcellmin<=Cellcheck)&&(Cellcheck<=Mcellmax)
        else
            continue   %skip motor that don't match with battery
        end
        Mi0 = Motor.data(5,ii); % motor's No-load current (A)
        MR = Motor.data(4,ii)/1000; % motor's Internal Resistance (m-ohm to ohm)
        w_motor = Motor.data(9,ii); % call Moter max watt
        fprintf('\nCalculating \nBatter :%s \nMotor :%s\n',Batt.textdata{i},Motor.textdata{ii});
        v = Cellcheck*3.7 ; %Voltage from battery
        kv = Motor.data(1,ii);% Motor's kv
        M_maxrpm = v*kv; %Motor's maximun RPM
        Pmin_init = zeros(3,5);
        
        for initial = 1:3 % 3 Initial start point for optimization
            x0(1) = X0(initial,1);  
            x0(2) = X0(initial,2);
            
            Row_point =[];
            Column_point =[];
            count = 1;
            Pdia = size_box(x0(1),x0(2),1); %Propeller's diameter
            Ppicth = size_box(x0(1),x0(2),2); %Propeller's picth
            [EPower,new_auw,c] = Power_Calculation(Pdia,Ppicth,Wp,w_batt,w_motor,S_factor,Vc,N_m,Mi0,MR,M_maxrpm,Mpowermax,kv,v,Bmax,hover_time,cruise_time,V0,Bcruise); % Calculate Propeller Power thru Power_Calculation.m
            
            y0 = EPower;
            y0_old = 11;
            auw = 11;
            C = 11;
            D = 11;
            P = 11;
            %HJ optimization patten search
            while y0-y0_old ~= 0
                y0_old = y0;
                Row_point(count)= x0(1);
                Column_point(count) = x0(2);
                Perform = zeros(3,7);
                condition = 1;% left and right points create
                Perform(1,1) = y0;
                Perform(1,2) = x0(1);
                Perform(1,3) = x0(2);
                Perform(1,4) = auw;
                Perform(1,5) = C;
                Perform(1,6) = D;
                Perform(1,7) = P;
                [Xright(1),Xright(2),Xleft(1),Xleft(2)] = HJpoint(x0(1),x0(2),condition);
                false_alarm =0;
                false_step = [0;0];
                if Xright(1) ==0 ||Xright(2)==0
                    false_alarm = 1;
                    false_step(2) = 2;
                end
                if Xleft(1) ==0 ||Xleft(2)==0
                    false_alarm = 1;
                    false_step(1) = 3;
                end
                for hj1 = 1:2
                    Perform(2,hj1+1) = Xright(hj1);
                    Perform(3,hj1+1) = Xleft(hj1);
                end
                if false_alarm == 1
                    False_line = nonzeros(false_step);
                    for line = 1 : size(False_line,1)
                        Perform(False_line(line),:)=[];
                    end
                end
                
                for hj2 = 2:size(Perform,1)
                    
                    Pdia = size_box(Perform(hj2,2),Perform(hj2,3),1); %Propeller's diameter
                    Ppicth = size_box(Perform(hj2,2),Perform(hj2,3),2); %Propeller's picth
                    [EPower,new_auw,c] = Power_Calculation(Pdia,Ppicth,Wp,w_batt,w_motor,S_factor,Vc,N_m,Mi0,MR,M_maxrpm,Mpowermax,kv,v,Bmax,hover_time,cruise_time,V0,Bcruise);
                    
                    Perform(hj2,1) = EPower;
                    Perform(hj2,4) = new_auw;
                    Perform(hj2,5) = c;
                    Perform(hj2,6) = Pdia;
                    Perform(hj2,7) = Ppicth;
                    
                end
                x1 = sortrows(Perform);
                x1_min = x1(1,:);
                
                count = count+1;
                Row_point(count)= x1_min(2);
                Column_point(count) = x1_min(3);
                
                condition = 2;% up and down points create
                Perform2 = zeros(3,7);
                Perform2(1,:) = x1_min;
                [Xup(1),Xup(2),Xdown(1),Xdown(2)] = HJpoint(x1_min(2),x1_min(3),condition);
                false_alarm =0;
                false_step = [0;0];
                if Xup(1) ==0 ||Xup(2)==0
                    false_alarm = 1;
                    false_step(2) = 2;
                end
                if Xdown(1) ==0 ||Xdown(2)==0
                    false_alarm = 1;
                    false_step(1) = 3;
                end
                for hj3 = 1:2
                    Perform2(2,hj3+1) = Xup(hj3);
                    Perform2(3,hj3+1) = Xdown(hj3);
                end
                if false_alarm == 1
                    False_line = nonzeros(false_step);
                    for line1 = 1 : size(False_line,1)
                        Perform2(False_line(line1),:)=[];
                    end
                end
                
                for hj4 = 2:size(Perform2,1)
                    
                    Pdia = size_box(Perform2(hj4,2),Perform2(hj4,3),1); %Propeller's diameter
                    Ppicth = size_box(Perform2(hj4,2),Perform2(hj4,3),2); %Propeller's picth
                    [EPower,new_auw,c] = Power_Calculation(Pdia,Ppicth,Wp,w_batt,w_motor,S_factor,Vc,N_m,Mi0,MR,M_maxrpm,Mpowermax,kv,v,Bmax,hover_time,cruise_time,V0,Bcruise);

                    Perform2(hj4,1) = EPower;
                    Perform2(hj4,4) = new_auw;
                    Perform2(hj4,5) = c;
                    Perform2(hj4,6) = Pdia;
                    Perform2(hj4,7) = Ppicth;
                end
                x2 = sortrows(Perform2);
                x2_min = x2(1,:);
                count = count+1;
                Row_point(count)= x2_min(2);
                Column_point(count) = x2_min(3);
                
                if x2_min(2) == x0(1) && x2_min(3) == x0(2)
                    x0(1)= x2_min(2);
                    x0(2) = x2_min(3);
                    y0 = x2_min(1);
                    auw = x2_min(4);
                    D = x2_min(6);
                    P =x2_min(7);
                    C = x2_min(5);

                    continue
                end
                
                %hooke-Jeeves pattern move
                [Xmove,Ymove] = HJmove(x0(1),x0(2),x2_min(2),x2_min(3));
                if Xmove ==0 ||Ymove==0
                    x0(1)= x2_min(2);
                    x0(2) = x2_min(3);
                    y0 = x2_min(1);
                    auw = x2_min(4);
                    D = x2_min(6);
                    P =x2_min(7);
                    C = x2_min(5);

                    continue
                end
                Pdia = size_box(Xmove,Ymove,1); %Propeller's diameter
                Ppicth = size_box(Xmove,Ymove,2); %Propeller's picth
                [EPower,new_auw,c] = Power_Calculation(Pdia,Ppicth,Wp,w_batt,w_motor,S_factor,Vc,N_m,Mi0,MR,M_maxrpm,Mpowermax,kv,v,Bmax,hover_time,cruise_time,V0,Bcruise);
                Yp = EPower;
                
                if Yp > y0
                    x0(1)= x2_min(2);
                    x0(2) = x2_min(3);
                    y0 = x2_min(1);
                    auw = x2_min(4);
                    D = x2_min(6);
                    P =x2_min(7);
                    C = x2_min(5);

                elseif Yp<y0
                    x0(1)= Xmove;
                    x0(2) = Ymove;
                    y0 = Yp;
                    auw = new_auw;
                    D = Pdia;
                    P = Ppicth;
                    C = c;

                end
                count = count+1;
            end
            %collect the best answer for each initial start point
            if initial == 1
                Row_init1 = Row_point;
                Col_init1 = Column_point;
                Pmin_init(initial,1) = y0;
                Pmin_init(initial,2) = x2_min(4);
                Pmin_init(initial,3) = x2_min(6);
                Pmin_init(initial,4) = x2_min(7);
                Pmin_init(initial,5) = x2_min(5);

            elseif initial == 2
                Row_init2 = Row_point;
                Col_init2 = Column_point;
                Pmin_init(initial,1) = y0;
                Pmin_init(initial,2) = x2_min(4);
                Pmin_init(initial,3) = x2_min(6);
                Pmin_init(initial,4) = x2_min(7);
                Pmin_init(initial,5) = x2_min(5);

            elseif initial == 3
                Row_init3 = Row_point;
                Col_init3 = Column_point;
                Pmin_init(initial,1) = y0;
                Pmin_init(initial,2) = x2_min(4);
                Pmin_init(initial,3) = x2_min(6);
                Pmin_init(initial,4) = x2_min(7);
                Pmin_init(initial,5) = x2_min(5);

            end
            
        end
        Y_init = sortrows(Pmin_init);
        Power_min_init = Y_init(1,1);
        Time = (60/1000*Cap*v)/(2*Power_min_init);
        %%%%%%%%%%%%%%%%%%%%%%%%%%
        % Plot figure
        %         figure(1)
        %         clf %clear figure
        %         hold on
        %         plot(Col_init1,Row_init1,'-or')
        %         plot(Col_init2,Row_init2,'-xb')
        %         plot(Col_init3,Row_init3,'-sg')
        %         legend('Init(3,3)','Init(5,9)','Init(11,11)')
        %         hold off
        %         grid on
        %         xlim([0 16])%axes limite
        %         ylim([0 17])
        %         set(gca,'xtick',[0:1:16]) %grid spacing
        %         set(gca,'ytick',[0:1:17])
        %         title(['Battery : ' Batt.textdata{i} ' Motor : ' Motor.textdata{ii} ' -Prop size Dis'])
        %
        %         pause
        %%%%%%%%%%%%%%
        fprintf('Data collected.\n');
        if Time > Spec(5,1)
            Flight_time = Spec;
            Flight_time(5,1) = Time;%endurance (min)
            Flight_time(5,2) = Y_init(1,2); %AUW (g)
            Flight_time(5,3) = Y_init(1,3); %Prop's diameter (in)
            Flight_time(5,4) = Y_init(1,4); %Prop's picth (in)
            Flight_time(5,5) = Y_init(1,5) ; %MAC (m)
            Flight_time(5,6) = ii; %motor No.ii
            Flight_time(5,7) = i; %Batt No.i
            
            Spec = sortrows(Flight_time,-1);
        else
            continue
        end
    end
end
toc; % stop timer
for Name=1:5
    if Spec(Name,6) ==0 || Spec(Name,7) ==0
        M_name{Name,1} = '-';
        B_name{Name,1} = '-';
        continue
    end
    M_name{Name,1} = Motor.colheaders{1,Spec(Name,6)};
    B_name{Name,1} = Batt.colheaders{1,Spec(Name,7)};  
end
fprintf('\nHook-Jeeves Opt.\n');
fprintf('\nRequirement\n');
fprintf('\nPayload weight = %g g\nSpan = %g m\nCruise speed %g m/s\n%g Ratio of hover time to flight time\n',Wp,Bmax,Vc,Ratio);
fprintf('\nDesige result\n');
Calculate_time = toc/60;
fprintf('\nCalculate time = %g min\n',Calculate_time);
No = {'1st';'2nd';'3rd';'4th';'5th'};
VarNames = {'No', 'Endurance_min', 'AUW_g','Prop_dia','Prop_picth','Chord_m','Motor_No','Motor_Name','Battery_No','Battry_Name'};
Table = table(No,Spec(:,1),Spec(:,2),Spec(:,3),Spec(:,4),Spec(:,5),Spec(:,6),M_name,Spec(:,7),B_name, 'VariableNames',VarNames)
