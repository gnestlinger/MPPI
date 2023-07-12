classdef MPPIController < handle
    %MPPICONTROLLER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        lambda
        horizon
        n_samples
        cov
        cov_mat

        model
        dt

        control_sequence
        optimized_control_sequence

        goal
        state
        n_states = 5;

        rollouts_states
        rollouts_costs
        rollouts_plot_handle = [];
    end
    
    methods
        function self = MPPIController(lambda, cov, horizon, n_samples, model, dt, goal)
            self.lambda = lambda;
            self.horizon = horizon;
            self.n_samples = n_samples;
            self.cov_mat = cov*eye(2);
            self.cov = cov;
            self.model = model;
            self.dt = dt;
            self.goal = goal;

            self.control_sequence = zeros(2,self.horizon);            
        end
        
        function action = get_action(self, state)
            % Init variables
            self.state = state;
            init_state = state;
            states = zeros(self.n_states, self.horizon+1);
            S = zeros(self.n_samples, 1);

            self.rollouts_states = zeros(self.n_samples, self.horizon+1, self.n_states);
            self.rollouts_costs = zeros(self.n_samples,1);

            % Generate random control input disturbances
            delta_u_vel = normrnd(0, 1, [self.n_samples, self.horizon]);
            delta_u_steer = normrnd(0, 0.4, [self.n_samples, self.horizon]);

            delta_u_steer(delta_u_steer > 0.5) = 0.5;
            delta_u_steer(delta_u_steer < -0.5) = -0.5;

            delta_u(1,:,:) = delta_u_vel;
            delta_u(2,:,:) = delta_u_steer;

            

            for k = 1:self.n_samples
%                 figure
                states(:,1) = init_state;
                for i = 1:self.horizon                    
                    % Single trajectory step
                    states(:,i+1) = self.model.step(self.control_sequence(:,i) + delta_u(:, k, i), self.dt, states(:,i));

                    % Compute cost of the state
                    S(k) = S(k) + self.cost_function(states(:,i+1));
%                     fprintf("Cost: %f\n", self.cost_function(states(:,i+1)));

                end               
%                 states
%                 self.control_sequence
%                 delta_u
%                 fprintf("Actions for %d rollout: \n", k);
%                 self.control_sequence() + reshape(delta_u(:, k, :), 2,self.horizon)

%                 fprintf("States for %d trajectory: \n", k);
%                 states
%                 plot(states(1,:), states(2,:))
                self.rollouts_states(k,:,:) = states';
                self.rollouts_costs(k) = S(k,end);
            end     

            % Update the control input according to the expectation over K sample trajectories
            S_normalized = S - min(S);
            for i = 1:self.horizon
                self.control_sequence(:,i) = self.control_sequence(:,i) + self.total_entropy(delta_u(:,:,i)', S_normalized(:))';                
            end
            self.control_sequence
            self.optimized_control_sequence = self.control_sequence;

            action = self.control_sequence(:,1);
            self.control_sequence = [self.control_sequence(:,2:end), [0; 0]];
        end

        function cost = cost_function(self, state)     
            cost = norm(self.goal(1:2) - state(1:2))^2*10 + (self.goal(3)-state(3))^2;            
        end

        function value = total_entropy(self, du, trajectory_cost)
            exponents = exp(-1/self.lambda * trajectory_cost);

            value = sum(exponents.*du ./ sum(exponents),1);
        end

        function plot_rollouts(self, fig)
            if ~isempty(self.rollouts_plot_handle)
                for i = 1:length(self.rollouts_plot_handle)
                    delete(self.rollouts_plot_handle(i));
                end
                self.rollouts_plot_handle = [];
            end
            figure(fig)
            costs = (self.rollouts_costs - min(self.rollouts_costs))/(max(self.rollouts_costs) - min(self.rollouts_costs));
            [~, min_idx] = min(costs);
            for i = 1:self.n_samples
                if i == min_idx
                    color = [0, 1, 1];
                else
                    color = [1-costs(i), 0, 0.2];
                end
                self.rollouts_plot_handle(end+1) = plot(self.rollouts_states(i,:,1), self.rollouts_states(i,:,2), 'Color', color);
%                 text(self.rollouts_states(i,end,1), self.rollouts_states(i,end,2), string(self.rollouts_costs(i)))
            end

            % Rollout of selected trajectory
            states = zeros(self.n_states, self.horizon+1);
            states(:,1) = self.state;

            for i = 1:self.horizon                    
                % Single trajectory step
                states(:,i+1) = self.model.step(self.optimized_control_sequence(:,i), self.dt, states(:,i));
            end

            self.rollouts_plot_handle(end+1) = plot(states(1,:), states(2,:), '--', 'Color', [0,1,0]);

        end     
    end
end

