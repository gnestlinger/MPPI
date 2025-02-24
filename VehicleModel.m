% MIT License
% 
% Copyright (c) 2023 Roman Adámek
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

classdef VehicleModel < handle  
    
    properties
        m = 2000.0, 			% Mass (kg)
        tau_steering = 2;       % steering time constant
        tau_velocity = 3;       % velocity time constant
        
        max_vel = 5;
    end
    
    methods
        function self = VehicleModel()
        end

        function state = step(self, action, dt, state)
            x = state(1);
            y = state(2);
            phi = state(3);
            prev_vel = state(4);
            prev_steer = state(5);

            vel = action(1);
            steer = action(2);

            vel = prev_vel + dt*(vel-prev_vel)/self.tau_velocity;
            steer = prev_steer + dt*(steer-prev_steer)/self.tau_steering;

            if vel > self.max_vel
                vel = self.max_vel;
            end

            x = x + vel*dt*cos(steer + phi);
            y = y + vel*dt*sin(steer + phi);
            phi = phi + steer;    

            state = [x, y, phi, vel, steer];
        end
    end
end

