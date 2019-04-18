function updated_map = avgProjMapWithInputData(proj_map, input_data, alpha, h, w, is_use, t)

    %==== Set variables ====
    input_points = input_data.pointcloud.Location;
    input_colors = input_data.pointcloud.Color;
    input_normals = input_data.normals;
    proj_points = proj_map.points;
    proj_colors = proj_map.colors;
    proj_normals = proj_map.normals;
    proj_ccounts = proj_map.ccounts;
    proj_times = proj_map.times;

    %==== TODO: Update all the terms in the projected map using the input data ====
    %==== (Hint: apply is_use[] as a mask in vectorization) ====

    % Write your code here...
    input_colors = double(input_colors);
    
    is_use = reshape(is_use, [h*w, 1]);
    updated_colors = reshape(proj_colors, [h*w, 3]);
    updated_normals = reshape(proj_normals, [h*w, 3]);
    updated_ccounts = reshape(proj_ccounts, [h*w, 1]);
    updated_times = reshape(proj_times, [h*w, 1]);
    
    
    proj_ccounts = reshape(proj_ccounts,[],1);
    proj_times = reshape(proj_times,[],1);
    alpha = reshape(alpha,[],1);
    is_use = reshape(is_use,[],1);
    proj_points = reshape(proj_points,[],3);
    proj_colors = reshape(proj_colors,[],3);
    proj_normals = reshape(proj_normals,[],3);
    input_points = reshape(input_points,[],3);
    input_colors = reshape(input_colors,[],3);
    input_normals = reshape(input_normals,[],3);
    
    proj_ccounts_use = proj_ccounts(is_use,:);
    proj_times_use = proj_times(is_use,:);
    alpha_use = alpha(is_use,:);
    proj_points_use = proj_points(is_use,:);
    proj_colors_use = proj_colors(is_use,:);
    proj_normals_use = proj_normals(is_use,:);
    input_points_use = input_points(is_use,:);
    input_normals_use = input_normals(is_use,:);
    input_colors_use = input_colors(is_use,:);

    updated_points_v = (proj_ccounts_use.*proj_points_use+alpha_use.*input_points_use)./(proj_ccounts_use+alpha_use);
    updated_normals_v = (proj_ccounts_use.*proj_normals_use+alpha_use.*input_normals_use)./(proj_ccounts_use+alpha_use);
    updated_colors_v = input_colors_use;
    updated_ccounts_v = proj_ccounts_use+alpha_use;

    updated_points = proj_points;
    updated_colors = proj_colors;
    updated_normals = proj_normals;
    updated_ccounts = proj_ccounts;
    updated_times = proj_times;

    updated_points(is_use,:) = updated_points_v;
    updated_points = reshape(updated_points,h,w,3);

    updated_colors(is_use,:) = updated_colors_v;
    updated_colors = reshape(updated_colors,h,w,3);

    updated_normals(is_use,:) = updated_normals_v;
    updated_normals = reshape(updated_normals,h,w,3);

    updated_ccounts(is_use) = updated_ccounts_v;
    updated_ccounts = reshape(updated_ccounts,h,w,1);

    updated_times(is_use) = t;
    updated_times = reshape(updated_times,h,w,1);
    
    %==== Output the updated projected map in a struct ====
    updated_map = struct('points', updated_points, 'colors', updated_colors, 'normals', updated_normals, 'ccounts', updated_ccounts, 'times', updated_times);
end