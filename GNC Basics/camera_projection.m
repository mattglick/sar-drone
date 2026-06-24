function [u,v] = project_point_to_pixel(point_3d, fx, fy, cx, cy)

    % point_3d is [X; Y; Z] in camera frame
    % fx, fy are focal lengths in pixels
    % cx, cy is the principal point (image center)
    % u, v are the output pixel coordinates
    
    % YOUR CODE HERE

    u = point_3d(1) / point_3d(3) * fx + cx;

    v = point_3d(2) / point_3d(3) * fy + cy;


end

    point_3d = [1;2;5];

    img_w = 1280;
    img_h = 720;
    fov_h = 82;
    fx = (img_w/2) / tan(deg2rad(fov_h/2));
    fy = fx;
    cx = img_w/2;
    cy = img_h/2;
    
    [u, v] = project_point_to_pixel(point_3d, fx, fy, cx, cy);
    fprintf('Pixel: u=%.1f, v=%.1f\n', u, v);

 function ray = pixel_to_ray(u, v, fx, fy, cx, cy)


    % u, v are pixel coordinates
    % returns a unit vector ray in camera frame
    
    % YOUR CODE HERE
    
   ray = [(u - cx)/fx ; (v-cy)/fy;1];

   ray = ray / norm(ray);  % Normalize the ray to make it a unit vector

end

% Calculate the ray from pixel coordinates
ray = pixel_to_ray(u, v, fx, fy, cx, cy);

disp(ray)