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

    K = [fx 0 cx; 0 fy cy; 0 0 1];
    MapPointsCameraFrame = pctransform(fusion_map.pointcloud, tform.invert);

    proj_flag = true([size(fusion_map.pointcloud.Location, 1), 1]);
    in_front_mask = (MapPointsCameraFrame.Location(:, 3) > 0);

    proj_flag = and(proj_flag, in_front_mask);
    in_front_mask = repmat(in_front_mask, 1, 3);
    valid_points = MapPointsCameraFrame.Location .* in_front_mask;

    pts_proj = (K*reshape(valid_points(in_front_mask), [], 3)')';
    pts_proj = pts_proj ./ pts_proj(:, 3);
    valid_points(in_front_mask) = reshape(pts_proj, [], 1);

    in_boundary_mask = valid_points(:,1) > 0 & valid_points(:,1) < h & valid_points(:,2) > 0 & valid_points(:,2) < w;
    valid_points = ceil(valid_points);
    proj_flag = and(proj_flag, in_boundary_mask);

    proj_points = zeros(h * w, 3);
    proj_colors = zeros(h * w, 3);
    proj_normals = zeros(h * w, 3);
    proj_ccounts = zeros(h * w, 1);
    proj_times = zeros(h* w, 1);

    pixel_points = reshape(fusion_map.pointcloud.Location(repmat(proj_flag, 1, 3)), [], 3);
    pixel_colors = reshape(fusion_map.pointcloud.Color(repmat(proj_flag, 1, 3)), [], 3);
    pixel_normals = reshape(fusion_map.normals(repmat(proj_flag, 1, 3)), [], 3);
    pixel_ccounts = fusion_map.ccounts(proj_flag);
    pixel_times = fusion_map.times(proj_flag);
    pixel_locations = reshape(valid_points(repmat(proj_flag, 1, 3)), [], 3);

    proj_points(pixel_locations(:, 1) * h + pixel_locations(:, 2) + 1, :) = pixel_points;
    proj_colors(pixel_locations(:, 1) * h + pixel_locations(:, 2) + 1, :) = pixel_colors;
    proj_normals(pixel_locations(:, 1) * h + pixel_locations(:, 2) + 1, :) = pixel_normals;
    proj_ccounts(pixel_locations(:, 1) * h + pixel_locations(:, 2) + 1) = pixel_ccounts;
    proj_times(pixel_locations(:, 1) * h + pixel_locations(:, 2) + 1) = pixel_times;

    proj_points = reshape(proj_points, [h, w, 3]);
    proj_colors = reshape(proj_colors, [h, w, 3]);
    proj_normals = reshape(proj_normals, [h, w, 3]);
    proj_ccounts = reshape(proj_ccounts, [h, w, 1]);
    proj_times = reshape(proj_times, [h, w, 1]);

    %==== Output the projected map in a struct ====
    %==== (Notice: proj_points[], proj_colors[], and proj_normals[] are all 3D matrices with size h*w*3) ====
    %==== (Notice: proj_ccounts[] and proj_times[] are both 2D matrices with size h*w) ====
    proj_map = struct('points', proj_points, 'colors', proj_colors, 'normals', proj_normals, 'ccounts', proj_ccounts, 'times', proj_times);
        
end
