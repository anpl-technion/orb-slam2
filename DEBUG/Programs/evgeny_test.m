% % LOAD AND DESERIALIZE GTSAM OBJECTS

clear 
clc
close all
fg_ser = load_serialized_gtsam_objects_charArr('evaluateObjFn_CPP_session[2]graph[0].txt');
val_ser = load_serialized_gtsam_objects_charArr('evaluateObjFn_CPP_session[2]initialEstimate[0].txt');
fg_ser_k = load_serialized_gtsam_objects_charArr('evaluateObjFn_CPP_session[2]graph_k.txt');
val_ser_k = load_serialized_gtsam_objects_charArr('evaluateObjFn_CPP_session[2]initialEstimate_k.txt');


fg = gtsam.NonlinearFactorGraph.string_deserialize(fg_ser);
val = gtsam.Values.string_deserialize(val_ser);

fg_k = gtsam.NonlinearFactorGraph.string_deserialize(fg_ser_k);
val_k = gtsam.Values.string_deserialize(val_ser_k);


options.optimizer =  'GN'; % ['LM' or 'ISAM2' or 'all']


% figure, hold on, grid on
% [pose_keys, land_keys] = plot_scene(val_k, 'r*');
% axis([-3 3 -4 0])
% view(gca,[-46.5 51]);
% axis equal


figure, hold on, grid on
[pose_keys, land_keys] = plot_scene(val, 'r*');
axis([-3 3 -4 0])
view(gca,[-46.5 51]);
axis equal

switch options.optimizer
    case 'LM'
        
        optimizer = gtsam.LevenbergMarquardtOptimizer(fg, val);
        result = optimizer.optimize();
        plot_scene(result, 'b*', '*');
        
        case 'GN'
        
        optimizer = gtsam.GaussNewtonOptimizer(fg, val);
        result = optimizer.optimize();
        plot_scene(result, 'b*', '*');
        
    case 'ISAM2'
        
        isamParams = gtsam.ISAM2Params;
        isamParams.setFactorization('QR');
        isam = gtsam.ISAM2(isamParams);
        isam.update(fg, val)
        result = isam.calculateEstimate();
        plot_scene(result, 'b*', '*');
        
    otherwise
        
        
        optimizer = gtsam.LevenbergMarquardtOptimizer(fg, val);
        result1 = optimizer.optimize();
        
        subplot(1,2,1), hold on, grid on,
        [pose_keys, land_keys] = plot_scene(val, 'r*');
        axis([-3 3 -4 0])
        view(gca,[-46.5 51]);
        axis equal
        plot_scene(result1, 'b*', '*'); title('LM')
        
        isamParams = gtsam.ISAM2Params;
        isamParams.setFactorization('QR');
        isam = gtsam.ISAM2(isamParams);
        isam.update(fg, val)
        result2 = isam.calculateEstimate();
        
        subplot(1,2,2), hold on, grid on,
        [pose_keys, land_keys] = plot_scene(val, 'r*');
        axis([-3 3 -4 0])
        view(gca,[-46.5 51]);
        axis equal
        
        plot_scene(result2, 'b*', '*'); title('ISAM2')
        
end







function [pose_keys, land_keys] = plot_scene(val, linespec, label_suffix)

if ~exist('label_suffix','var'), label_suffix=''; end

k = gtsam.KeyVector(val.keys);
pose_keys = [];
land_keys = [];

for i = 0:k.size-1
    xi = val.at(k.at(i));
    if strcmp(class(xi), 'gtsam.Pose3')
        %H = xi.matrix
       
        pose_keys = [pose_keys k.at(i)];
        gtsam.plotPose3(xi, [], 0.5)
        text(xi.translation.x+0.1, xi.translation.y+0.1, xi.translation.z+0.1, [char(gtsam.symbolChr(pose_keys(end))) num2str(gtsam.symbolIndex(pose_keys(end))) label_suffix])
        i;
    elseif strcmp(class(xi), 'gtsam.Point3')
      
        land_keys = [land_keys k.at(i)];
        gtsam.plotPoint3(xi, linespec)
    end
end

end
