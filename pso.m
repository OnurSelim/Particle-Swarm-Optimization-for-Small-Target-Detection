clc;
clear;
close all;

%% Problem Definition

I = imread("Images/img_002.bmp");
sortedImage = reshape(I,1,[]);

nVar=6;            % Number of Decision Variables

VarSize=[1 nVar];   % Size of Decision Variables Matrix

VarMin= 0;         % Lower Bound of Variables
VarMax= 256;         % Upper Bound of Variables

VarMinX = 0;
VarMaxX = 5;

VarMinY = 0;
VarMaxY = 5;

VarMinWp = 20;
VarMaxWp = size(I,2);

VarMinHp = 20;
VarMaxHp = size(I,1);

VarMinWHt = 3;
VarMaxWHt = 15;

%% PSO Parameters

MaxIt=100;      % Maximum Number of Iterations

nPop=400;        % Population Size (Swarm Size)

% PSO Parameters
w=1;            % Inertia Weight
wdamp=0.99;     % Inertia Weight Damping Ratio
c1=1.5;         % Personal Learning Coefficient
c2=2.0;         % Global Learning Coefficient

% If you would like to use Constriction Coefficients for PSO,
% uncomment the following block and comment the above set of parameters.

% % Constriction Coefficients
% phi1=2.05;
% phi2=2.05;
% phi=phi1+phi2;
% chi=2/(phi-2+sqrt(phi^2-4*phi));
% w=chi;          % Inertia Weight
% wdamp=1;        % Inertia Weight Damping Ratio
% c1=chi*phi1;    % Personal Learning Coefficient
% c2=chi*phi2;    % Global Learning Coefficient

% Velocity Limits
VelMax=0.5*(VarMax-VarMin);
VelMin=-VelMax;

%% Initialization

empty_particle.Position=[];
empty_particle.Cost=[];
empty_particle.Velocity=[];
empty_particle.Best.Position=[];
empty_particle.Best.Cost=[];

particle=repmat(empty_particle,nPop,1);

GlobalBest.Cost=inf;
% [~,tempI] = maxk(sortedImage,nPop);
% initXs = floor(tempI/200);
% initYs = tempI-floor(tempI/200)*200;


for i=1:nPop    
    % Initialize Position
%     tempX = initXs(i)+randi([VarMinX VarMaxX]);
%     tempY = initYs(i)+randi([VarMinY VarMaxY]);

    particle(i).Position(1) = randi([VarMinWp,VarMaxWp]);
    particle(i).Position(2) = randi([VarMinHp,VarMaxHp]);
    particle(i).Position(3) = randi([VarMinWp,floor(VarMaxWp/4)]);
    particle(i).Position(4) = randi([VarMinHp,floor(VarMaxHp/4)]);
    particle(i).Position(5) = randi([VarMinWHt,VarMaxWHt]);
    particle(i).Position(6) = randi([VarMinWHt,VarMaxWHt]);
    % Initialize Velocity
    particle(i).Velocity=zeros(VarSize);
    
    % Evaluation
    particle(i).Cost=CostFunction(I,particle(i).Position,0);
    
    % Update Personal Best
    particle(i).Best.Position=particle(i).Position;
    particle(i).Best.Cost=particle(i).Cost;
    
    % Update Global Best
    if particle(i).Best.Cost<GlobalBest.Cost
        
        GlobalBest=particle(i).Best;
        
    end
    
end

BestCost=zeros(MaxIt,1);

%% PSO Main Loop

for it=1:MaxIt
    
    for i=1:nPop
        
        % Update Velocity
        particle(i).Velocity = w*particle(i).Velocity ...
            +c1*rand(VarSize).*(particle(i).Best.Position-particle(i).Position) ...
            +c2*rand(VarSize).*(GlobalBest.Position-particle(i).Position);
        
        % Apply Velocity Limits
        particle(i).Velocity = max(particle(i).Velocity,VelMin);
        particle(i).Velocity = min(particle(i).Velocity,VelMax);
        
        % Update Position
        particle(i).Position = round(particle(i).Position + particle(i).Velocity);
        
        % Velocity Mirror Effect
        IsOutside=[(particle(i).Position(1)<VarMin | particle(i).Position(1)>VarMax)...
            (particle(i).Position(2)<VarMin | particle(i).Position(2)>VarMax)...
            (particle(i).Position(3)<VarMinWp | particle(i).Position(3)>VarMaxWp)...
            (particle(i).Position(4)<VarMinHp | particle(i).Position(4)>VarMaxHp)...
            (particle(i).Position(5)<VarMinWHt | particle(i).Position(5)>VarMaxWHt)...
            (particle(i).Position(6)<VarMinWHt | particle(i).Position(6)>VarMaxWHt)];
        particle(i).Velocity(IsOutside)=-particle(i).Velocity(IsOutside);
        
        % Apply Position Limits
        particle(i).Position(1) = max(particle(i).Position(1),1);
        particle(i).Position(1) = min(particle(i).Position(1),VarMaxWp);
        
        particle(i).Position(2) = max(particle(i).Position(2),1);
        particle(i).Position(2) = min(particle(i).Position(2),VarMaxHp);

        particle(i).Position(3) = max(particle(i).Position(3),VarMinWp);
        particle(i).Position(3) = min(particle(i).Position(3),VarMaxWp/4);

        particle(i).Position(4) = max(particle(i).Position(4),VarMinHp);
        particle(i).Position(4) = min(particle(i).Position(4),VarMaxHp/4);

        particle(i).Position(5) = max(particle(i).Position(5),VarMinWHt);
        particle(i).Position(5) = min(particle(i).Position(5),VarMaxWHt);

        particle(i).Position(6) = max(particle(i).Position(6),VarMinWHt);
        particle(i).Position(6) = min(particle(i).Position(6),VarMaxWHt);

        if(particle(i).Position(1)+particle(i).Position(3)>VarMaxWp ...
                &&particle(i).Position(2)+particle(i).Position(4)>VarMaxHp)
            particle(i).Position(1) = VarMaxWp-particle(i).Position(3)-1;
            particle(i).Position(2) = VarMaxHp-particle(i).Position(4)-1;
        elseif(particle(i).Position(1)+particle(i).Position(3)>VarMaxWp)
            particle(i).Position(1) = VarMaxWp-particle(i).Position(3)-1;
        elseif(particle(i).Position(2)+particle(i).Position(4)>VarMaxHp)
            particle(i).Position(2) = VarMaxHp-particle(i).Position(4)-1;
        end
        
        % Evaluation
        particle(i).Cost = CostFunction(I,particle(i).Position,0);
        
        % Update Personal Best
        if particle(i).Cost<particle(i).Best.Cost
            
            particle(i).Best.Position=particle(i).Position;
            particle(i).Best.Cost=particle(i).Cost;
            
            % Update Global Best
            if particle(i).Best.Cost<GlobalBest.Cost
                
                GlobalBest=particle(i).Best;
                
            end
            
        end
        
    end
    
    BestCost(it)=GlobalBest.Cost;
    
    disp(['Iteration ' num2str(it) ': Best Cost = ' num2str(BestCost(it))]);
    
    w=w*wdamp;
    
end

BestSol = GlobalBest;

% for i=1:100
%     Temp = CostFunction(I,particle(i).Position,1);
% end

Fitness = CostFunction(I,GlobalBest.Position,1);
%% Results

figure;
%plot(BestCost,'LineWidth',2);
semilogy(BestCost,'LineWidth',2);
xlabel('Iteration');
ylabel('Best Cost');
grid on;
