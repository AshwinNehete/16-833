function fusion_map = updateFusionMapWithProjMap(fusion_map, updated_map, h, w, proj_flag)

    %==== TODO: Merge the updated map with the remaining part of the old fusion map ====

    % Write your code here...
    map_points = fusion_map.pointcloud.Location;
    map_colors = fusion_map.pointcloud.Color;
    map_normals = fusion_map.normals;
    map_ccounts = fusion_map.ccounts;
    map_times = fusion_map.times;

    proj_flag_b = zeros(length(proj_flag), 1);
    proj_flag_b(proj_flag) = 1;
    map_points  = map_points(~proj_flag_b, :);
    map_colors  = map_colors(~proj_flag_b, :);
    map_normals = map_normals(~proj_flag_b, :);
    map_ccounts = map_ccounts(~proj_flag_b, :);
    map_times = map_times(~proj_flag_b, :);
    
    updated_map_points  = reshape(updated_map.points, [ ], 3);
    updated_map_colors  = reshape(updated_map.colors, [ ], 3);
    updated_map_normals = reshape(updated_map.normals, [ ], 3);
    updated_map_ccounts = reshape(updated_map.ccounts, [ ], 1);
    updated_map_times = reshape(updated_map.times, [ ], 1);

    valid_updates = ~all(updated_map_points == 0, 2);
    updated_map_points  = updated_map_points(valid_updates, :);
    updated_map_colors  = updated_map_colors(valid_updates, :);
    updated_map_normals = updated_map_normals(valid_updates, :);
    updated_map_ccounts = updated_map_ccounts(valid_updates, :);
    updated_map_times   = updated_map_times(valid_updates, :);
    
    map_points  = [map_points; updated_map_points];
    map_colors  = [map_colors; updated_map_colors];
    map_normals = [map_normals; updated_map_normals];
    map_ccounts = [map_ccounts; updated_map_ccounts];
    map_times = [ map_times; updated_map_times];

    %==== Output the final point-based fusion map in a struct ====
    map_pointcloud = pointCloud(map_points, 'Color', map_colors);
    fusion_map = struct('pointcloud', map_pointcloud, 'normals', map_normals, 'ccounts', map_ccounts, 'times', map_times);
      
end
   