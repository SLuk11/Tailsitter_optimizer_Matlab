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

X0 = [ 3 3 ; 5 5 ; 8 8 ]; %coordinate for initial
step = 1; %step size


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
        size_box(:,:,3) = 0;
        
        for initial = 1:3
            %while loop intial
            y0 = 1;
            min_EPower_hover = 0;
            x0(1) = X0(initial,1);
            x0(2) = X0(initial,2);
            Row_point =[];
            Column_point =[];
            count = 1;
             while y0 ~= min_EPower_hover
                 Row_point(count)= x0(1);
                 Column_point(count) = x0(2);
                 Ri = zeros(1,5);
                 Ci = zeros(1,5);
                 Power = zeros(5,2);
                 false_step = zeros(1,5);
                 false_alarm = 0;
                for point = 0:4 %5 points calculation
                    if point == 0
                        Ri(point+1) = x0(1);
                        Ci(point+1) = x0(2);
                    elseif point == 1
                        Ri(point+1) = x0(1)-step;
                        Ci(point+1) = x0(2)-step;
                    elseif point == 2
                        Ri(point+1) = x0(1)+step;
                        Ci(point+1) = x0(2)-step;
                    elseif point == 3
                        Ri(point+1) = x0(1)-step;
                        Ci(point+1) = x0(2)+step;
                    elseif point == 4
                        Ri(point+1) = x0(1)+step;
                        Ci(point+1) = x0(2)+step;
                    end
                    
                    if Ri(point+1) <1 ||Ri(point+1)>10 || Ci(point+1) <1 ||Ci(point+1) >10
                        false_alarm = 1;
                        false_step(point+1) = point+1;
                        Power(point+1,:) =0;
                        continue
                    elseif Ri(point+1) == 10 && Ci(point+1)==10
                        false_alarm = 1;
                        false_step(point+1) = point+1;
                        Power(point+1,:) =0;
                        continue
                    end
  
                Pdia = size_box(Ri(point+1),Ci(point+1),1); %Propeller's diameter
                Ppicth = size_box(Ri(point+1),Ci(point+1),2); %Propeller's picth
                Mi0 = Motor.data(5,ii); % motor's No-load current (A)
                MR = Motor.data(4,ii); % motor's Internal Resistance (m-ohm)
                w_batt = Batt.data(4,i);
                w_motor = Motor.data(9,ii);
                [EPower_hover] = Power_Calculation(Pdia,Ppicth,w,w_batt,w_motor,S_factor,Vc,N_m,Mi0,MR,M_maxrpm,Mpowermax,kv,v);
                
                Power(point+1,1) = EPower_hover;
                Power(point+1,2) = point+1;
                end
                
                if false_alarm ~= 0;
                    sort_step = sort(false_step,'descend');
                    cutline = nonzeros(sort_step);
                    for cut = 1:size(cutline)
                        Power(cutline(cut),:)=[];
                    end
                end
                y0 = Power(1,1);
                Yi = sortrows(Power);
                min_EPower_hover = Yi(1,1);
                x0(1) = Ri(Yi(1,2));
                x0(2) = Ci(Yi(1,2));
                
%                if y0 == min_EPower_hover
%                    Power_check = zeros(4,2);
%                    Ri_check = zeros(1,4);
%                    Ci_check = zeros(1,4);
%                    false_step_check = zeros(1,4);
%                    false_alarm_check = 0;
                    
                    
%                    for point_check = 1:4 %4 points check
%                    if point_check == 1
%                        Ri_check(point_check) = x0(1);
%                        Ci_check(point_check) = x0(2)+step;
%                    elseif point_check == 2
%                        Ri_check(point_check) = x0(1);
%                        Ci_check(point_check) = x0(2)-step;
%                    elseif point_check == 3
%                        Ri_check(point_check) = x0(1)-step;
%                        Ci_check(point_check) = x0(2);
%                    elseif point_check == 4
%                        Ri_check(point_check) = x0(1)+step;
%                        Ci_check(point_check) = x0(2);
%                    end
                    
%                    if Ri_check(point_check) <1 ||Ri_check(point_check)>10 || Ci_check(point_check) <1 ||Ci_check(point_check) >10
%                        false_alarm_check = 1;
%                        false_step_check(point_check) = point_check;
%                        Power_check(point_check,:) =0;
%                        continue
%                    elseif Ri_check(point_check) == 10 && Ci_check(point_check)==10
%                        false_alarm_check = 1;
%                        false_step_check(point_check) = point_check;
%                        Power_check(point_check,:) =0;
%                        continue
%                    end
  
%                    Pdia = size_box(Ri_check(point_check),Ci_check(point_check),1); %Propeller's diameter
%                    Ppicth = size_box(Ri_check(point_check),Ci_check(point_check),2); %Propeller's picth
%                    Mi0 = Motor.data(5,ii); % motor's No-load current (A)
%                    MR = Motor.data(4,ii); % motor's Internal Resistance (m-ohm)
%                    w_batt = Batt.data(4,i);
%                    w_motor = Motor.data(9,ii);
%                    [EPower_hover] =
%                    Power_Calculation(Pdia,Ppicth,w,w_batt,w_motor,S_factor,Vc,N_m,Mi0,MR,M_maxrpm,Mpowermax,kv,v);

%                    Power_check(point_check,1) = EPower_hover;
%                    Power_check(point_check,2) = point_check;

%                    end 
%                    if false_alarm_check ~= 0;
%                        sort_step_check = sort(false_step_check,'descend');
%                        cutline_check = nonzeros(sort_step_check);
%                        for cut_check = 1:size(cutline_check)
%                            Power_check(cutline_check(cut_check),:)=[];
%                        end
%                    end
%                    Yi_check = sortrows(Power_check);
%                    min_EPower_hover = Yi_check(1,1);
%                    x0(1) = Ri_check(Yi_check(1,2));
%                    x0(2) = Ci_check(Yi_check(1,2));    
%                end

                count = count+1;
             end 
             
             figure(1)
             clf %clear figure
             hold on
             plot(Row_point,Column_point,'-o')
             hold off
             grid on
             xlim([0 11])%axes limite
             ylim([0 11])
             set(gca,'xtick',[0:1:11]) %grid spacing
             set(gca,'ytick',[0:1:11])
             title(['Battery : ' Batt.textdata{i} ' Motor : ' Motor.textdata{ii} ' -Prop size Dis'])

             pause
        end
    end
  
end