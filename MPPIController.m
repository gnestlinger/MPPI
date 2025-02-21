classdef MPPIController < matlab.System ...
        & matlab.system.mixin.Propagates ...
        & matlab.system.mixin.CustomIcon
% MPPIController    Model Predictive Path Integral controller.
%
%   MPPIController properties (visualization):
%   Visualize - Enable/disable visualization.
%   TraceRobotPath - Trace the evolution of the robot path.
%   FigureNbr - Print visualization to specific figure.
% 
%   MPPIController properties:
%   NbrTrajs - Number of rollout trajectories.
%   NbrPredSteps - Number of prediction steps.
%   Lambda - Temperature.
%   nu - Exploration variance.
%   R - Control weighting matrix.
%   cov - Variance of control inputs disturbance.
%   TimeStep - Prediction time step.
%   max_vel - Max. veloctiy.
%   max_steer - Max. steering angle.
% 
%   MPPIController methods:
%   step - Compute control action for current state.
% 
%   See also matlab.System.

% MIT License
% 
% Copyright (c) 2023 Roman Adámek
% Copyright (c) 2025 Georg Nestlinger
% 
% Permission is hereby granted, free of charge, to any person obtaining a
% copy of this software and associated documentation files (the
% "Software"), to deal in the Software without restriction, including
% without limitation the rights to use, copy, modify, merge, publish,
% distribute, sublicense, and/or sell copies of the Software, and to permit
% persons to whom the Software is furnished to do so, subject to the
% following conditions:
% 
% The above copyright notice and this permission notice shall be included
% in all copies or substantial portions of the Software.
% 
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
% OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
% MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN
% NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
% DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
% OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
% USE OR OTHER DEALINGS IN THE SOFTWARE.
    
    
    properties (Nontunable, Logical)
        % Visualize - Enable/disable visualization
        %   When used in Simulink in a MATLAB System block, "Simulate
        %   using" must be set to "Interpreted execution".
        Visualize = false
        
        % TraceRobotPath - Trace the robot path
        TraceRobotPath = true
    end
    
    properties (Nontunable, PositiveInteger) % Public, non-tunable properties
        % FigureNbr - Print visualization to specific figure.
        FigureNbr = 99
        
        % NbrTrajs - Number of rollout trajectories
        NbrTrajs = 400
        
        % NbrPredSteps - Number of prediction steps
        NbrPredSteps = 25
    end
    
    properties % Public, tunable properties
        % Lambda - Temperature selectiveness of trajectories by their costs 
        %   The closer this value to 0, the "more" we take in considiration
        %   controls with less cost, 0 mean use control with best cost,
        %   huge value will lead to just taking mean of all trajectories
        %   without cost consideration.
        Lambda = 10
        
        % nu - Exploration variance
        nu = 500
        
        % R - Control weighting matrix
        R = diag([1 5])
        
        % cov - Variance of control inputs disturbance
        cov = [1 0.4]
        
        % TimeStep - Prediction time step (sec)
        TimeStep = 0.1
        
        max_vel = 5;
        max_steer = 0.5;
    end

    properties (SetAccess = immutable)
        Model% = VehicleModel()
    end

    properties (Access = private) % Pre-computed constants
        control_sequence
        
        % Graphic handles
        AxesHandle = gobjects(1)
        PlotHandleTrajsRollout = gobjects(0)
        PlotHandleTrajOpt = gobjects(1)
        PlotHandleRobotTrace = gobjects(1)
        PlotHandleTitle = gobjects(1)
        NSTepCalls = 0
    end
    
    properties (Access = private, Nontunable)
        NbrStates = 5
    end
    
    
    
    methods
        function self = MPPIController(varargin)
        % Support name-value pair arguments when constructing object
            setProperties(self, nargin-1, varargin{2:end})
            if nargin > 0
                self.Model = varargin{1};
            end
        end%Constructor
    end
    
    methods(Access = protected)
        %% Common functions
        function setupImpl(self, ~, ~, obstacles)
        % Perform one-time calculations, such as computing constants
            
            self.control_sequence = zeros(numel(self.cov), self.NbrPredSteps);
            
            if self.Visualize
                self.setupPlot(obstacles);
            end
        end%fcn
        
        function y = stepImpl(self, state, goal, obstacles)
        % Implement algorithm. Calculate y as a function of input u and
        % discrete states.
            
            % Init variables
            states = coder.nullcopy(zeros(self.NbrStates, self.NbrPredSteps + 1));
            states(:,1) = state;
            S = zeros(self.NbrTrajs, 1);
            rollouts_states = zeros(self.NbrTrajs, self.NbrPredSteps + 1, self.NbrStates);
            
            % Generate random control input disturbances
            du = normrnd(0, self.cov, [numel(self.cov) self.NbrTrajs self.NbrPredSteps]);
            du(du > 0.5) = 0.5;
            du(du < -0.5) = -0.5;
            
            mdl = self.Model;
            dt = self.TimeStep;
            self.control_sequence(:,1) = 0;
            U = circshift(self.control_sequence, -1, 2);
            for k = 1:self.NbrTrajs
                duk = permute(du(:,k,:), [1 3 2]);
                
                u = U + duk;
                for i = 1:self.NbrPredSteps
                    % Single trajectory step
                    tmp = mdl.step(u(:,i), dt, states(:,i));
                    states(:,i+1) = tmp;
                end
                
                % Compute cost of the state
                S(k) = sum(self.cost_function(states(:,2:end), U, duk, goal, obstacles));
                
                rollouts_states(k,:,:) = states';
            end%for
            
            % Update the control input according to the expectation over K
            % sample trajectories
            entropy = self.total_entropy(du, S - min(S));
            U = U + permute(entropy, [2 3 1]); 
            
            % Output saturation
            vMax = self.max_vel;
            sMax = self.max_steer;
            U(1, U(1,:) > +vMax) = +vMax;
            U(1, U(1,:) < -vMax) = -vMax;
            U(2, U(2,:) > +sMax) = +sMax;
            U(2, U(2,:) < -sMax) = -sMax;
            self.control_sequence = U;
            
            % Select control action
            y = U(:,1);
            
            % Predicted states w.r.t. optimal control sequence
            xp = coder.nullcopy(zeros(self.NbrStates, self.NbrPredSteps + 1));
            xp(:,1) = state(:);
            for i = 1:self.NbrPredSteps
                % Single trajectory step
                xp(:,i+1) = mdl.step(U(:,i), self.TimeStep, xp(:,i));
            end
            
            if self.Visualize
                self.NSTepCalls = self.NSTepCalls + 1;
                self.stepPlot(state, rollouts_states, S, xp);
            end
        end%fcn
        
        function resetImpl(self)
        % Initialize / reset discrete-state properties
        end
    end
    
    methods (Access = private)
        function setupPlot(self, obstacles)
        %SETUPPLOT  Setup axes and plot handles.
        
            xMax = ceil(max(obstacles(:,1))/10)*10;
            yMax = ceil(max(obstacles(:,2))/10)*10;
            
            fig = figure(self.FigureNbr);
            axh = gca(fig);
            cla(axh, 'reset')
            self.AxesHandle = axh;
            set(axh, 'Box','on', ...
                'NextPlot','add', ...
                'DataAspectRatio',[1 1 1], ...
                'XGrid','on', ...
                'YGrid','on', ...
                'XLim',[-1 xMax], ...
                'YLim',[-1 yMax]);
            
            % Plot obstacles
            for i = 1:size(obstacles,1)
                r = obstacles(i,3);
                pos = [obstacles(i,[1 2]) - r, 2*r, 2*r];
                rectangle(axh, 'Position',pos, 'Curvature',[1 1], ...
                    'FaceColor', 'k', 'Edgecolor','none');
            end
            
            % Initialize graphic handles for rollout trajectories
            nans = nan(self.NbrPredSteps + 1, self.NbrTrajs);
            legend(axh, 'AutoUpdate','off') % No legend for rollout trajectories
            self.PlotHandleTrajsRollout = plot(axh, nans, nans, 'k-', ...
                'LineWidth',0.5);
            legend(axh, 'AutoUpdate','on')
            
            % Initialize graphic handle for robot trace/prediction
            self.PlotHandleTrajOpt = plot(axh, nans(:,1), nans(:,1), 'g--', ...
                'LineWidth',1, 'DisplayName','Optimal trajectory');
            self.PlotHandleRobotTrace = plot(axh, nan, nan, 'g.-', ...
                'MarkerSize',10, 'DisplayName','Robot trajectory');
            set(self.PlotHandleRobotTrace, 'XData',[], 'YData',[])
            
            % Title
            self.PlotHandleTitle = title(axh, sprintf('t = %.2f', 0));
        end%fcn
        
        function stepPlot(self, state, rollouts_states, rollouts_costs, xp)
        %STEPPLOT   Plot current iteration.
        
            t = self.NSTepCalls*self.TimeStep;
            set(self.PlotHandleTitle, 'String',sprintf('t = %.2f sec', t));
            
            minRC = min(rollouts_costs);
            costs = (rollouts_costs - minRC)/(max(rollouts_costs) - minRC);
            one = ones(self.NbrTrajs, 1);
            set(self.PlotHandleTrajsRollout, ...
                {'XData'},num2cell(rollouts_states(:,:,1), 2), ...
                {'YData'},num2cell(rollouts_states(:,:,2), 2), ...
                {'Color'},num2cell([1 - costs 0*one 0.2*one], 2));
            set(self.PlotHandleTrajOpt, 'XData',xp(1,:), 'YData',xp(2,:))
            
            if self.TraceRobotPath
                h = self.PlotHandleRobotTrace;
                x = get(h, 'XData');
                y = get(h, 'YData');
                set(h, 'XData', [x state(1)], 'YData',[y state(2)])
            end
            
            drawnow('limitrate')
        end%fcn
        
        function cost = cost_function(self, state, u, du, goal, obstacles)
           state_cost = self.state_cost_function(state, goal, obstacles);
           control_cost = self.control_cost_function(u, du);
           
           cost = state_cost + control_cost;
        end

        function cost = state_cost_function(self, state, goal, obstacles)
            obstacle_cost = self.obstacle_cost_function(state(1,:), state(2,:), obstacles);
            heading_cost = self.heading_cost_function(state(3,:), goal(3));
            distance_cost = self.distance_cost_function(state(1,:), state(2,:), goal);%norm(self.goal(1:2) - state(1:2))^2*5;

            cost = distance_cost + heading_cost + obstacle_cost;
        end

        function cost = distance_cost_function(self, x, y, goal)
            weight = 100;
%             cost = 100*norm(self.goal(1:2) - state(1:2))^2;
            cost = weight*hypot(goal(1) - x, goal(2) - y).^2;
        end%fcn

        function cost = heading_cost_function(self, yaw, yawGoal)
            weight = 50;
            pow = 2;
            cost = weight*abs(get_angle_diff(yawGoal, yaw)).^pow;
        end%fcn
        
        function cost = control_cost_function(self, u, du)
%             cost = (1-1/self.nu)/2 * du'*self.R*du + u'*self.R*du + 1/2*u'*self.R*u;
            
            RR = self.R;
            cost = 0.5*(1-1/self.nu)*sum((du'*RR.*du'), 2)' ...
                + sum((u'*RR.*du'), 2)' ...
                +0.5*sum((u'*RR.*u'), 2)';
        end
        
        function obstacle_cost = obstacle_cost_function(self, x, y, obstacles)
                        
            if isempty(obstacles)
                obstacle_cost = 0;
                return
            end
            distance_to_obstacle = hypot(obstacles(:,1) - x, obstacles(:,2) - y);
            [min_dist,min_dist_idx] = min(distance_to_obstacle, [], 1);
            
            hit = double(min_dist < obstacles(min_dist_idx,3)');
            obstacle_cost = 550*exp(-min_dist/5) + 1e6*hit;
        end
        
        function value = total_entropy(self, du, trajectory_cost)
            exponents = exp(-1/self.Lambda * trajectory_cost);
            
            value = sum(exponents.*permute(du, [2 1 3]) ./ sum(exponents), 1);
        end
    end
    
end%class


function angle = get_angle_diff(angle1, angle2)
angle = mod(angle1 - angle2 + pi, 2*pi) - pi;
end%fcn

function r = normrnd(mu, sigma, sz)
r = randn(sz).*sigma(:) + mu;
end%fcn
