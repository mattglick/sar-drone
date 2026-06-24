% Matthew Glick
% 3d Monocular Geolocation Simulation
% 23 June 2026

%% Flowchart

% 1. Create drone and person position
% 2. Project target into camera frame



%% Initializations

drone_lat = 47.6062;
drone_lon = -122.3321;
drone_alt =19;        % meters
roll      = 0;
pitch = deg2rad(-45);  % steeper downward look % nose tilted down 15 degrees
yaw       = deg2rad(30);  % facing 30 degrees from north

% Section 2: True target position (you make this up too)
target_lat = 47.60625;
target_lon = -122.33205;
target_alt = 0;

% Section 3: Convert target GPS to local NED coordinates relative to drone
R_earth = 111320;

%% Calculations

% How far north is the target from the drone in metres?
north = (target_lat-drone_lat) * R_earth;

% How far east?
east = (target_lon - drone_lon) * R_earth * cos(deg2rad(drone_lat));

% How far down? (target is on ground, drone is at drone_alt)
down = drone_alt - target_alt;

target_NED = [north; east; down];

target_XYZ = [north; east; down];

R = Rz(yaw) * Ry(pitch) * Rx(roll);

fprintf('Rotation matrix R:\n');
disp(R)

% Convert NED to camera frame
% Camera X = East (right)
% Camera Y = Down
% Camera Z = North (forward)
target_ned_vec = [north; east; down];

% First rotate by drone attitude
target_body = R' * target_ned_vec;

% Then reorder body frame to camera frame
% Body: X=forward, Y=right, Z=down
% Camera: X=right, Y=down, Z=forward
target_cam = [target_body(2); target_body(3); target_body(1)];

fprintf('Target NED: north=%.1f, east=%.1f, down=%.1f\n', north, east, down);
fprintf('Target in camera frame: X=%.1f, Y=%.1f, Z=%.1f\n', target_cam(1), target_cam(2), target_cam(3));

img_w = 1280;
img_h = 720;
fov_h = 82;
fx = (img_w/2) / tan(deg2rad(fov_h/2));
fy = fx;
cx = img_w/2;
cy = img_h/2;

[u,v] = project_point_to_pixel( target_cam, cx, cy, fx, fy);

fprintf('Target appears at pixel: u=%.1f, v=%.1f\n', u, v);

pixel_to_latlon(u, v, fx, fy, cx, cy, R, drone_lat, drone_lon, drone_alt, target_lat, target_lon);





%% Simulation & Visualization





%% Plotting



%% Functions

function [u, v] = project_point_to_pixel( target_cam, cx, cy, fx, fy )

    u = target_cam(1) / target_cam(3) * fx + cx;

    v = target_cam(2) / target_cam(3) * fy + cy;

end

function R = Rx(angle)
    R = [1 0 0; 0 cos(angle) -sin(angle); 0 sin(angle) cos(angle)];
end

function R = Ry(angle)
    R = [cos(angle) 0 sin(angle); 0 1 0; -sin(angle) 0 cos(angle)];
end

function R = Rz(angle)
    R = [cos(angle) -sin(angle) 0; sin(angle) cos(angle) 0; 0 0 1];
end

function pixel_to_latlon(u, v, fx, fy, cx, cy, R, drone_lat, drone_lon, drone_alt, target_lat, target_lon)

    ray_cam = [(u - cx)/fx; (v - cy)/fy; 1];
    ray_cam = ray_cam / norm(ray_cam);
    
    ray_body = [ray_cam(3); ray_cam(1); ray_cam(2)];
    ray_NED = R * ray_body;
    
    t = drone_alt / ray_NED(3);
    north_est = ray_NED(1) * t;
    east_est  = ray_NED(2) * t;
    
    est_lat = drone_lat + north_est / 111320;
    est_lon = drone_lon + east_est / (111320 * cos(deg2rad(drone_lat)));

    error_north = (est_lat - target_lat) * 111320;
    error_east  = (est_lon - target_lon) * 111320 * cos(deg2rad(drone_lat));
    error_m = sqrt(error_north^2 + error_east^2);
    fprintf('Estimated: %.6f, %.6f\n', est_lat, est_lon);
    fprintf('True:      %.6f, %.6f\n', target_lat, target_lon);
    fprintf('Error: %.2f metres\n', error_m);
    


end

