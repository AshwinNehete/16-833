function fusion_map = updateFusionMapWithProjMap(fusion_map, updated_map, h, w, proj_flag)

    %==== TODO: Merge the updated map with the remaining part of the old fusion map ====

    % Write your code here...
    unchanged_flag = not(proj_flag);

    % old map information
    unchanged_points = reshape(fusion_map.pointcloud.Location(repmat(unchanged_flag, 1, 3)), [], 3);
    unchanged_colors = reshape(fusion_map.pointcloud.Color(repmat(unchanged_flag, 1, 3)), [], 3);
    unchanged_normals = reshape(fusion_map.normals(repmat(unchanged_flag, 1, 3)), [], 3);
    unchanged_ccounts = fusion_map.ccounts(unchanged_flag);
    unchanged_times = fusion_map.times(unchanged_flag);

    % new map information
    new_points = reshape(updated_map.points, [h * w, 3]);
    new_colors = reshape(updated_map.colors, [h * w, 3]);
    new_normals = reshape(updated_map.normals, [h * w,3]);
    new_ccounts = reshape(updated_map.ccounts, [h * w, 1]);
    new_times = reshape(updated_map.times, [h * w, 1]);

    % concatenate together
    map_points = cat(1, unchanged_points, new_points);
    map_colors = cat(1, unchanged_colors, new_colors);
    map_normals = cat(1, unchanged_normals, new_normals);
    map_ccounts = cat(1, unchanged_ccounts, new_ccounts);
    map_times = cat(1, unchanged_times, new_times);
    
    %==== Output the final point-based fusion map in a struct ====
    map_pointcloud = pointCloud(map_points, 'Color', map_colors);
    fusion_map = struct('pointcloud', map_pointcloud, 'normals', map_normals, 'ccounts', map_ccounts, 'times', map_times);
      
end
   