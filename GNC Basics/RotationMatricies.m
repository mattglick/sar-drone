function R = Rx(angle)
    R = [1 0 0; 0 cos(angle) -sin(angle); 0 sin(angle) cos(angle)];
end

function R = Ry(angle)
    R = [cos(angle) 0 sin(angle); 0 1 0; -sin(angle) 0 cos(angle)];
end

function R = Rz(angle)
    R = [cos(angle) -sin(angle) 0; sin(angle) cos(angle) 0; 0 0 1];
end

v_body = [0;0;1];

yaw = deg2rad(45);

pitch = deg2rad(30);

roll = deg2rad(0);

R = Rx(roll) * Rz(yaw) * Ry(pitch);
v_world = R * v_body;

disp('World Frame Vector')

disp(v_world)

R2 = R';

v_revert = R2 * v_world;

disp('World Frame Reverted Vector')

disp(v_revert)
