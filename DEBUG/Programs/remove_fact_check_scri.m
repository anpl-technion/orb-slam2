cd ~/Desktop/aevadim13_orb/DEBUG/Programs/

Test_Evgeny_DEBUG_ORB

isam2 = gtsam.ISAM2;

fg_post = fg_bel.clone;


for i = 0:inc_fg.size-1
    fg_post.add(inc_fg.at(i))
end
fg_bel.size
fg_post.size
val_post = gtsam.Values;
val_post.insert(val_bel);
val_post.insert(inc_val02);

ind2rem_ar = str2double(ind2rem);
ind2rem_ar(end)=[];

%%
% optimization with isam of graph constructed from scratch excluding
% removed factors works, obviously indexing is changed and sequential

% One by one
isam2 = gtsam.ISAM2;
isam2.update(fg_bel, val_bel);
    
for indx = 1 : length(ind2rem_ar)
    remFac_keys = gtsam.KeyVector;
    remFac_keys.push_back(ind2rem_ar(indx));
    indx
    fg_bel.at(ind2rem_ar(indx))
    
    
     if indx == 1 
        isam2.update(inc_fg, inc_val02, remFac_keys)
     else
        isam2.update(gtsam.NonlinearFactorGraph, gtsam.Values, remFac_keys)
     end
end

% % accumalative
% for num = 1:length(ind2rem_ar)
%     remFac_keys = gtsam.KeyVector;
%     for indx = 1 : num        
%         remFac_keys.push_back(ind2rem_ar(indx));
%     end
%     num
%     isam2 = gtsam.ISAM2;
%     isam2.update(fg_bel, val_bel);
%     isam2.update(inc_fg, inc_val02, remFac_keys);
% end
% fg_bel.at(ind2rem_ar(num))


% c++
remFac_keys = gtsam.KeyVector;

for indx = 1 : length(ind2rem_ar)-3
    
    remFac_keys.push_back(ind2rem_ar(indx));
    
end
isam2 = gtsam.ISAM2;
isam2.update(fg_bel, val_bel);
isam2.update(inc_fg, inc_val02, remFac_keys)
% % Error using gtsam_wrapper
% % Exception from gtsam:
% % Requested to eliminate a key that is not in the factors
% % 
% % 
% % Error in gtsam.ISAM2/update (line 169)
% %         varargout{1} = gtsam_wrapper(1409, this, varargin{:});



fg_post_rem = fg_post.clone;
graph = gtsam.NonlinearFactorGraph;
isam2 = gtsam.ISAM2;

j=1;
for i=0:fg_post.size-1
    if j <= length(ind2rem_ar) && ind2rem_ar(j) == i
        j = j +1;
    else
        graph.add(fg_post.at(i));
    end
    
end
isam2.update(graph, val_post)
%%

% every index in removed indices exist in prior and the size of the graph
% after factor removal is not changed, but the removed factors do not exist
% and cannot be accessed. Indexing is kept so there exist jumps (intervals)
% at places where the removed index is

fg_prior_rem = fg_bel.clone;
for i=1:length(ind2rem_ar)
    fg_bel.at(ind2rem_ar(i));
   
    fg_prior_rem.remove(ind2rem_ar(i))
end

fg_prior_rem.size
fg_bel.size
%fg_post.size - fg_post_rem.size == length(ind2rem_ar)
%%