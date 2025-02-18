% MIT License
% 
% Copyright (c) 2023 Roman Adámek
% Copyright (c) 2025 Georg Nestlinger
% 
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is
% furnished to do so, subject to the following conditions:
% 
% The above copyright notice and this permission notice shall be included in all
% copies or substantial portions of the Software.
% 
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
% SOFTWARE.

clc, clear
rng(1)

%% Param defintion
init_state = [0, 0, 0, 0, 0]; % x, y, phi, v, steer
goal_state = [6, 6, 0];


%% Define environment - obstacles [x, y, radius]
n_obstacles = 40;
obstacles = [rand(n_obstacles, 2)*4+1, 0.2*ones(n_obstacles, 1)];


%% Init
car_real = VehicleModel();
car = VehicleModel();
controller = MPPIController(car);
controller.Visualize = false;

car_state = init_state;
dt = controller.dt;
for i = 1:100
    action = controller.step(car_state, goal_state, obstacles);
    
    car_state = car_real.step(action, dt, car_state);
    
    % Exporting to animated GIF requires at least R2022a
    %exportgraphics(gcf,'animation.gif','Append',true);
end

