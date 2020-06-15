clear all
close all
clc

%% The main string array will be parsed by the delimeter
delimiter = ';';

% %% exitexit

Path="/home/evgeny/ANPL/infrastructure/aevadim13/scenarios/orb_slam_active/results/Or_Salmon_2020-06-15_07-12-18/serialized_files/";

fn = "Factors_ind2rem_vec_02.txt";   % belief before update to 02
str = fileread(Path + fn);
ind2rem = strsplit(str, delimiter, 'CollapseDelimiters', false);
ind2rem = ind2rem';

fn = "belief_01_ser_factors.txt";   % belief before update to 02
str = fileread(Path + fn);
fg_bel = gtsam.NonlinearFactorGraph.string_deserialize(str);

fn = "belief_01_ser_values.txt";  %inc_belief_fg
str = fileread(Path + fn);
val_bel = gtsam.Values.string_deserialize(str);




fn = "FG_check_02.txt";         %part of the message from ORBSLAM
str = fileread(Path + fn);
fg_ORB = gtsam.NonlinearFactorGraph.string_deserialize(str);

% fn = "FG_rem_kv_first_02.txt";   %part of the message from ORBSLAM
% str = fileread(Path + fn);
% kv1 = gtsam.KeyVector.string_deserialize(str);
% 
% fn = "FG_rem_kv_second_02.txt";  %part of the message from ORBSLAM
% str = fileread(Path + fn);
% kv2 = gtsam.KeyVector.string_deserialize(str);


fn = "m_inc_graph_02.txt";  %inc_belief_fg
str = fileread(Path + fn);
inc_fg = gtsam.NonlinearFactorGraph.string_deserialize(str);



fn = "m_inc_values02.txt";  %inc_belief_fg
str = fileread(Path + fn);
inc_val02 = gtsam.Values.string_deserialize(str);


Tfg_ORB = fg_2_table(fg_ORB);
Tfg_bel = fg_2_table(fg_bel);
% Tk = kv1kv2_2_table(kv1,kv2);


% T.Properties.VariableNames(1) = {'First Letter'};
% T.Properties.VariableNames(2) = {'First Number'};
% T.Properties.VariableNames(3) = {'Second Letter'};
% T.Properties.VariableNames(4) = {'Second Number'};











isam2 = gtsam.ISAM2;








%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%% Functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function T = fg_2_table(f)
% ASSUMING BINARY FACTORS ONLY
% Display fg as matrix with:
%   1st colum pose symbol number (Ex: A0 --> 0)
%   2nd colum landmark symbol number (Ex: l721 --> 721)
%   3rd colum fg type (Ex: A0 - > 0)

flag=false;
for i=0:f.size-1
    keys = f.at(i).keys;
    if keys.size == 0
        error('Andrej was right')
    end
    if keys.size == 1
        flag=true;
        k1 = keys.at(0);
        FL(i+1) = char(gtsam.symbolChr(k1));
        FN(i+1) = gtsam.symbolIndex(k1);
        SL(i+1) = 'N';
        SN(i+1) = -1;
        continue
    end
    k1 = keys.at(0); k2 = keys.at(1);
    FL(i+1) = char(gtsam.symbolChr(k1));
    FN(i+1) = gtsam.symbolIndex(k1);
    SL(i+1) = char(gtsam.symbolChr(k2));
    SN(i+1) = gtsam.symbolIndex(k2);
    
    
end
if flag
    T = table(FL',FN',SL',SN');
else
    T = table(FL',FN',SL',SN');
end
end

function T = kv1kv2_2_table(kv1,kv2)

for i=0:kv1.size-1
    FL(i+1) = char(gtsam.symbolChr(kv1.at(i)));
    FN(i+1) = gtsam.symbolIndex(kv1.at(i));
    SL(i+1) = char(gtsam.symbolChr(kv2.at(i)));
    SN(i+1) = gtsam.symbolIndex(kv2.at(i));
end
T = table(FL',FN',SL',SN');
end