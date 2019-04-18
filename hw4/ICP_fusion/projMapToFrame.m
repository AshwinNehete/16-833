function [proj_map, proj_flag] = projMapToFrame(fusion_map, h, w, tform, cam_param)
    
    %==== Set parameters ====
    fx = cam_param(1);
    fy = cam_param(2);
    cx = cam_param(3);
    cy = cam_param(4);

    %==== TODO: Project all terms in the fusion map based on other input parameters ====
    %==== (Hint 1: Only project the points in front of the camera) ====
    %==== (Hint 2: Calculate all the projected indices together and use them in vectorization) ====
    %==== (Hint 3: Discard the indices that exceed the frame boundaries) ====

    % Write your code here...
    fx = cam_param(1);
    fy = cam_param(2);
    cx = cam_param(3);
    cy = cam_param(4);
    
    tform = inv(tform.T');
    tform = tform(1:3, :);
    K = [fx 0 cx; 0 fy cy; 0 0 1];
    
    locs = fusion_map.pointcloud.Location;
    locs_t = vertcat(locs', ones(1, size(locs,1)));
    
    proj_points_full = K*tform*locs_t;
    proj_points_full = proj_points_full';
    positive_depth_idx = find(proj_points_full(:,3)>0);
    
    proj_points_full = proj_points_full ./ proj_points_full(:,3);
    proj_points_full = round(proj_points_full);
    
    [~, unique_idx, ~] = unique(proj_points_full, 'rows');
    x_bounds = and(proj_points_full(:,1)>0, proj_points_full(:,1)<=w);
    y_bounds = and(proj_points_full(:,2)>0, proj_points_full(:,2)<=h);
    bounds_idx = find(and(x_bounds, y_bounds));
    valid_idx = intersect(bounds_idx, intersect(unique_idx, positive_depth_idx));
    
    x = proj_points_full(valid_idx, 1);
    y = proj_points_full(valid_idx, 2);
    out_idx = sub2ind([h w], y, x);
    
    proj_points = zeros(h*w, 3);
    proj_normals = zeros(h*w, 3);
    proj_colors = zeros(h*w, 3);
    proj_ccounts = zeros(h*w, 1);
    proj_times = zeros(h*w, 1);
    
    proj_points(out_idx, :) = locs(valid_idx, 1:3);
    proj_normals(out_idx, :) = fusion_map.normals(valid_idx, 1:3);
    proj_colors(out_idx, :) = fusion_map.pointcloud.Color(valid_idx, 1:3);
    proj_ccounts(out_idx, :) = fusion_map.ccounts(valid_idx);
    proj_times(out_idx, :) = fusion_map.times(valid_idx);
    
    proj_points = reshape(proj_points, h, w, 3);
    proj_normals = reshape(proj_normals, h, w, 3);
    proj_colors = reshape(proj_colors, h, w, 3);
    proj_ccounts = reshape(proj_ccounts, h, w);
    proj_times = reshape(proj_times, h, w);

    %==== Output the projected map in a struct ====
    %==== (Notice: proj_points[], proj_colors[], and proj_normals[] are all 3D matrices with size h*w*3) ====
    %==== (Notice: proj_ccounts[] and proj_times[] are both 2D matrices with size h*w) ====
    proj_flag = valid_idx;
    proj_map = struct('points', proj_points, 'colors', proj_colors, 'normals', proj_normals, 'ccounts', proj_ccounts, 'times', proj_times);
        
end
