classdef CBFcontroller
    % CBFCONTROLLER Class
    
    properties
        % This section stores basic information about various parameters.
        % Each parameter includes details such as its operational range and units.
        params
    end
    
    methods
        function obj = CBFcontroller(params)
            % Initialize the CBFcontroller Class parameters
            obj.params = params;
        end

        function B = safetySet(obj, x)
            % Content to be defined in the child class
            % Define a control barrier function to ensure operation within safe ranges
            B = [];
        end

        function Bjacob = safetySetJacob(obj, x)
            % Content to be defined in the child class
            % Define the CBF jacobian function, which is the derivative of the CBF with respect to the state space x
            Bjacob = [];
        end
        
        % dx = f(x) + g(x)*u
        function u_act = CBFqpSolver(obj, x, f, g, u_des)
            % u(x) = argmin 1/2*||u - u_des||.^2
            %      = argmin 1/2*(uT*u - 2*uT*u_des + u_desT*u_des)
            % s.t. Lf*B(x) + Lg*B(x)*u >= -gamma(B(x))
            %
            % Explanation about the quadprog in matlab
            % [x,fval,exitflag,output,lambda] = quadprog(H,f,A,b,Aeq,beq,lb,ub,x0,options);
            % qp function: 1/2*x'*H*x + f'*x
            % s.t. A*x <= b
            % s.t. Aeq*x = beq
            % s.t. x(i) >= lb(i)
            % s.t. x(i) <= ub(i)

            B = safetySet(obj, x);
            if isempty(B)
                error('Safety set is empty. Ensure safetySet function is correctly implemented.');
            end
            
            [~, uColumns] = size(g);
            quadH = eye(uColumns);      % quadH must be a square matrix
            quadF = -u_des;

            disp('quadH: ');
            disp(quadH);
            disp('quadF: ');
            disp(quadF);
            % dL/dx
            Bjacob = safetySetJacob(obj, x);
    
            % dL/dt = dL/dx(->Bjacob) * dx/dt = Bjacob * (f + g*u)
            LfB = Bjacob * f;
            LgB = Bjacob * g;           % u is implicitly included in the inequality constraints, so it is omitted

            A_inequal = -LgB;
            b_inequal = LfB + obj.params.gamma*B;

            [u_act, ~, exitflag] = quadprog(quadH, quadF, A_inequal, b_inequal);

            if exitflag ~= 1
                disp(['QP solver failed: ',num2str(exitflag)]);
                u_act = u_des;          % Default to 0 if solver fails
            end
        end
        
    end
end

