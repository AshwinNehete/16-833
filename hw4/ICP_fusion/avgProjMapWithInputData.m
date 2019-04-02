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
    is_use = reshape(is_use, [h*w, 1]);
    updated_points = reshape(proj_points, [h*w, 3]);
    updated_colors = reshape(proj_colors, [h*w, 3]);
    updated_normals = reshape(proj_normals, [h*w, 3]);
    updated_ccounts = reshape(proj_ccounts, [h*w, 1]);
    updated_times = reshape(proj_times, [h*w, 1]);
    input_alpha = reshape(alpha, [h*w, 1]);
    
    % Update
    usable_points = reshape(updated_points(repmat(is_use, 1, 3)), [], 3);
    usable_input_pts = reshape(input_points(repmat(is_use, 1, 3)), [], 3);
    usable_normals = reshape(updated_normals(repmat(is_use, 1, 3)), [], 3);
    usable_input_normals = reshape(input_normals(repmat(is_use, 1, 3)), [], 3);
    usable_ccounts = updated_ccounts(is_use);
    usable_alpha = input_alpha(is_use);
    
    updated_points(repmat(is_use, 1, 3)) = reshape((usable_ccounts .* usable_points + usable_alpha .* usable_input_pts) ./ (usable_ccounts + usable_alpha), [] ,1);
    updated_colors(repmat(is_use, 1, 3)) = input_colors(repmat(is_use, 1, 3));
    updated_normals(repmat(is_use, 1, 3)) = reshape((usable_ccounts .* usable_normals + usable_alpha .* usable_input_normals) ./ (usable_ccounts + usable_alpha), [] ,1);
    updated_ccounts(is_use) = usable_ccounts + usable_alpha;
    updated_times(is_use) = t;
    
    updated_points = reshape(updated_points, [h, w, 3]);
    updated_colors = reshape(updated_colors, [h, w, 3]);
    updated_normals = reshape(updated_normals, [h, w, 3]);
    updated_ccounts = reshape(updated_ccounts, [h, w, 1]);
    updated_times = reshape(updated_times, [h, w, 1]);
    
    %==== Output the updated projected map in a struct ====
    updated_map = struct('points', updated_points, 'colors', updated_colors, 'normals', updated_normals, 'ccounts', updated_ccounts, 'times', updated_times);
        
end