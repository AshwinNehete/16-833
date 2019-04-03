function fusion_map = updateFusionMapWithProjMap(fusion_map, updated_map, h, w, proj_flag)

    %==== TODO: Merge the updated map with the remaining part of the old fusion map ====

    % Write your code here...
    not_proj_flag = not(proj_flag);

    % point cloud manipulation
    not_proj_points = reshape(fusion_map.pointcloud.Location(repmat(not_proj_flag, 1, 3)), [], 3);
    reshaped_points = reshape(updated_map.points, [h * w, 3]);
    map_points = cat(1, not_proj_points, reshaped_points);

    % colors manipulation
    not_proj_colors = reshape(fusion_map.pointcloud.Color(repmat(not_proj_flag, 1, 3)), [], 3);
    reshaped_colors = reshape(updated_map.colors, [h * w, 3]);
    map_colors = cat(1, not_proj_colors, reshaped_colors);

    % normals manipulation
    not_proj_normals = reshape(fusion_map.normals(repmat(not_proj_flag, 1, 3)), [], 3);
    reshaped_normals = reshape(updated_map.normals, [h * w,3]);
    map_normals = cat(1, not_proj_normals, reshaped_normals);

    % ccounts manipulation
    not_proj_ccounts = fusion_map.ccounts(not_proj_flag);
    reshaped_ccounts = reshape(updated_map.ccounts, [h * w, 1]);
    map_ccounts = cat(1, not_proj_ccounts, reshaped_ccounts);

    % time manipulation
    not_proj_times = fusion_map.times(not_proj_flag);
    reshaped_times = reshape(updated_map.times, [h * w, 1]);
    map_times = cat(1, not_proj_times, reshaped_times);
    
    %==== Output the final point-based fusion map in a struct ====
    map_pointcloud = pointCloud(map_points, 'Color', map_colors);
    fusion_map = struct('pointcloud', map_pointcloud, 'normals', map_normals, 'ccounts', map_ccounts, 'times', map_times);
      
end
   