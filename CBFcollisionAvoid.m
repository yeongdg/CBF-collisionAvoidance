=2=classdef CBFcollisionAvoid < CBFcontroller
    methods
        function obj = CBFcollisionAvoid(params)
            obj@CBFcontroller(params)
        end

        function B = safetySet(obj, x)
            B(1,:) = ((x(1)-obj.params.obs.x)^2 + (x(2)-obj.params.obs.x)^2) - (obj.params.obs.Radius)^2;
        end

        function Bjacob = safetySetJacob(obj, x)
            % [ dB1(x)/x1 dB1(x)/x2 ]
            % [ dB2(x)/x1 dB2(x)/x2 ]
            % [         ...         ]
            Bjacob(1,:) = 2*[(x(1) - obj.params.obs.x) (x(2)-obj.params.obs.y)];
        end

        function dxdt = dynamics(obj, x, t)
            % define the state equations
            f = [0; 0];
            g = [1 0; 0 1];
            u_des = obj.params.K*[(obj.params.target.x - x(1)); (obj.params.target.y - x(2))];
            u = CBFqpSolver(obj, x, f, g, u_des);
            dxdt = f + g*u;
        end
    end
end

