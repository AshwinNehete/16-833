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


    %==== Output the projected map in a struct ====
    %==== (Notice: proj_points[], proj_colors[], and proj_normals[] are all 3D matrices with size h*w*3) ====
    %==== (Notice: proj_ccounts[] and proj_times[] are both 2D matrices with size h*w) ====
    proj_map = struct('points', proj_points, 'colors', proj_colors, 'normals', proj_normals, 'ccounts', proj_ccounts, 'times', proj_times);
        
end
