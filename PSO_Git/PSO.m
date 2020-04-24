function out = PSO(problem,params)
    
    %% Problem definition

CostFunction = problem.CostFunction; % can be a general cost function

nVar = problem.nVar; %No. of unknown (Decision) variables % in this 5 dimensional space

VarSize = [1 nVar]; %Matrix Size of decision variable

VarMin = problem.VarMin; % Lower Bound of Decision Variables
VarMax = problem.VarMax; % Upper Bound of Decision Variables

%% Parameters of PSO

MaxIt = params.MaxIt; % Maximum Number of Iterations

nPop = params.nPop; % Population size (Swarm Size)

w = params.w; % Inertia co-efficient
wdamp = params.wdamp; % damping factor for better convergence
c1 = params.c1; % Personal Acceleration Coefficient
c2 = params.c2; % Social Acceleration Coefficient

ShowIterInfo = params.ShowIterInfo;

MaxVelocity = (VarMax - VarMin)*0.2;
MinVelocity = -(VarMax - VarMin)*0.2;

%% Initialization

empty_particle.Position = [];
empty_particle.Velocity = [];
empty_particle.Cost = [];
empty_particle.Best.Position = [];
empty_particle.Best.Cost = [];

particle = repmat(empty_particle,nPop,1);

% Initialize Global Best
GlobalBest.Cost = inf; % we need to find minima of Cost function
GlobalBest.Position = [];

% Intialize Population Members
for i=1:nPop
    
    % Generate Random Solution
    particle(i).Position = unifrnd(VarMin,VarMax,VarSize);
    
    % Evaluation
    particle(i).Cost = CostFunction(particle(i).Position);
    
    % Initialize Velocity
    particle(i).Velocity = zeros(VarSize);
    
    % update the personal best to its current location
    particle(i).Best.Cost = particle(i).Cost;
    particle(i).Best.Position = particle(i).Position;
    
    %finding Global Best
    if(particle(i).Cost < GlobalBest.Cost)
        GlobalBest.Cost = particle(i).Cost;
        GlobalBest.Position = particle(i).Position;
    end
    
end

% Array to hold best cost value in each iteration
BestCosts = zeros(MaxIt, 1);

%% Main Loop of PSO

for it=1:MaxIt 
    
    for i=1:nPop
        
        % Update Velocity
        particle(i).Velocity = w*particle(i).Velocity ...
            + c1*rand(VarSize).*(particle(i).Best.Position - particle(i).Position) ...
            + c2*rand(VarSize).*(GlobalBest.Position - particle(i).Position);
        
        % Apply Lower and Upper Bound Limits
        particle(i).Velocity = max(particle(i).Velocity,MinVelocity); % Lower Limit
        particle(i).Velocity = min(particle(i).Velocity,MaxVelocity); % Upper limit
        
        % Update Postion
        particle(i).Position = particle(i).Position + particle(i).Velocity;
        
        % Apply Lower and Upper Bound Limits
        particle(i).Position = max(particle(i).Position,VarMin);
        particle(i).Position = min(particle(i).Position,VarMax);
        
        % Update Evaluation
        particle(i).Cost = CostFunction(particle(i).Position);
        % Update Personal Best
        if particle(i).Cost < particle(i).Best.Cost
            particle(i).Best.Cost = particle(i).Cost;
            particle(i).Best.Position = particle(i).Position;
            % update Global Best
            if particle(i).Cost < GlobalBest.Cost
                GlobalBest.Cost = particle(i).Cost;
                GlobalBest.Position = particle(i).Position;
            end
            
        end
        
    end
    
    % Store the Best Cost Value of each Iteration
    BestCosts(it) = GlobalBest.Cost;
    
    % Display Iteration Information
    if (ShowIterInfo)
    disp([ ' Iteration ' num2str(it) ' : Best Cost = ' num2str(BestCosts(it))])
    end
    
    % Damping Inertia Coefficienct
    w = w * wdamp;
end
    out.pop = particle;
    out.BestSol = GlobalBest;
    out.BestCosts = BestCosts;
    
end