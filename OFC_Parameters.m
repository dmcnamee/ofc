function params = OFC_Parameters(varargin)
%% FUNCTION: Globalizes and returns INDEPENDENT parameters for OFC.
% INPUTS:   varargin  = {pass parameters explicitly}
% OUTPUTS:  optThresh = diff threshold for iterative L/K optimization
%           initDiff  = initial difference in L
%           maxIter   = maximum number of iterations
%           xinit     = initial position-velocity-actuator state of the system and goal-target positions, default = (0,...,0) column
%           tinit     = initial time of system, default = 0
%           mdim      = dimensionality of plane of movement, default = 2
%           plim      = [-xlim +xlim;-ylim +ylim], default = [-15 15; 0 30]
%           vlim      = [-xlim +xlim;-ylim +ylim], default = [0 80; 0 80]
%           ulim      = [-xlim +xlim;-ylim +ylim], default = [0 1500; 0 1500] cm/s^2
%           alim      = [-xlim +xlim;-ylim +ylim], default = [0 100; 0 100] cm/s^2
%           pgoal     = [ngoals x mdim] goal-target positions, default = [0 10; -5 20]
%           wgoal     = weights of goal-targets, default = ones(size(pgoal,1),1)
%           goalsize  = size of goal-targets, default = 1cm
%           b         = viscous constant, default = 10 N.s/m
%           tau       = time constant of linear filter, default = 50ms
%           m         = mass of hand, default = 1kg
%           tf        = time constant for actuator, default = 40ms
%           smdelay   = sensorimotor delay, default = 50ms
%           tres      = time resolution (step size) for discretization, default = 0.02s
%           pres      = position resolution (step size) for discretization, default = 0.2cm
%           vres      = velocity resolution (step size) for discretization, default = 3.33cm/s
%           ures      = acceleration resolution (step size) for discretization, default = 166.67cm/s^2
%           c1        = signal-dependent noise parallel to u, default = 0.15
%           c2        = signal-dependent noise perp to u, default = 0.05
%           usigma    = std of signal-dependent noise, default = 0.015
%           psigma    = std of position noise, default = 1cm
%           vsigma    = std of velocity noise, default = 10cm/s
%           asigma    = std of actuator noise, default = 1N
%           ssigma    = std of overall sensory noise (scales other noise terms), default = 0.5
%           Wenergy   = weight of energy in cost function, default = 0.00005
%           Wtime     = weight of time taken in cost function, default = 20
%           Wstop     = weight of stopping in cost function, default = 1
%           Wactuator = weight of actuation in cost function, default = 0.1
%           Wgoal     = weight of goal-target distance in cost function, default = 100
%           Wtimeout  = weight of exceeding time limit in cost function, default = 100
%           tmax      = maximum allowed time for movement, tmax = 1s
%           vthresh   = maximum allowed velocity at final target, vthresh = 5 cm/s
%           xdim      = dimensionality of state variable X
%           Tgoal     = discrete times at which via-points-goals are traversed (assumed regular intervals)
% NOTES:    N/A
% ISSUES:   alim vs ulim?
% REFS:     Todorov2002 / Liu2007
% AUTHOR:   Daniel McNamee, daniel.c.mcnamee@gmail.com

%% set parameters as global variables
% clearvars -global;
global optThresh initDiff maxIter;
global xinit tinit mdim;
global plim vlim ulim alim;
global tres pres vres ures;
global pgoal wgoal goalsize ngoal;
global b tau m tf smdelay;
global tsteps psteps vsteps usteps;
global c1 c2 C usigma psigma vsigma asigma ssigma;
global Wenergy Wtime Wstop Wactuator Wgoal Wtimeout;
global tmax vthresh;
global xdim;
global T Tgoal;

%% setup
while ~isempty(varargin)
    switch varargin{1}
        case 'params'
            v2struct(varargin{2});
        case 'optThresh'
            optThresh = varargin{2};
        case 'initDiff'
            initDiff = varargin{2};
        case 'maxIter'
            maxIter = varargin{2};
        case 'xinit'
            xinit = varargin{2};
        case 'tinit'
            tinit = varargin{2};
        case 'mdim'
            mdim = varargin{2};
        case 'plim'
            plim = varargin{2};
        case 'vlim'
            vlim = varargin{2};
        case 'ulim'
            ulim = varargin{2};
        case 'pgoal'
            pgoal = varargin{2};
        case 'wgoal'
            wgoal = varargin{2};
        case 'Tgoal'
            Tgoal = varargin{2};
        case 'goalsize'
            goalsize = varargin{2};
        case 'b'
            b = varargin{2};
        case 'tau'
            tau = varargin{2};
        case 'm'
            m = varargin{2};
        case 'tf'
            tf = varargin{2};
        case 'smdelay'
            smdelay = varargin{2};
        case 'tres'
            tres = varargin{2};
        case 'pres'
            pres = varargin{2};
        case 'vres'
            vres = varargin{2};
        case 'ures'
            ures = varargin{2};
        case 'c1'
            c1 = varargin{2};
        case 'c2'
            c2 = varargin{2};
        case 'usigma'
            usigma = varargin{2};
        case 'psigma'
            psigma = varargin{2};
        case 'vsigma'
            vsigma = varargin{2};
        case 'asigma'
            asigma = varargin{2};
        case 'ssigma'
            ssigma = varargin{2};
        case 'Wenergy'
            Wenergy = varargin{2};
        case 'Wtime'
            Wtime = varargin{2};
        case 'Wstop'
            Wstop = varargin{2};
        case 'Wactuator'
            Wactuator = varargin{2};
        case 'Wgoal'
            Wgoal = varargin{2};
        case 'Wtimeout'
            Wtimeout = varargin{2};
        case 'tmax'
            tmax = varargin{2};
        case 'vthresh'
            vthresh = varargin{2};
        otherwise
            error(['Unexpected option: ' varargin{1}])
    end
      varargin(1:2) = [];
end

%% defaults
if isempty(optThresh)
    optThresh = 0.0001;
end
if isempty(initDiff)
    initDiff = 1;
end
if isempty(maxIter)
    maxIter = 100;
end
if isempty(mdim)
    mdim = 2;
end
if isempty(plim)
    % plim = [-30 30; -30 30];
    plim = [-10 10; -5 25];
end
if isempty(vlim)
    vlim = [-40 40; -40 40];
end
if isempty(ulim)
    ulim = [-700 700; -700 700];
end
if isempty(alim)
    alim = [-100 100; -100 100];
end
if isempty(pgoal)
    pgoal = [0 10; -5 20];
end
if isempty(goalsize)
    goalsize = 1;
end
if isempty(b)
    b = 10;
end
if isempty(tau)
    tau = 0.05;
end
if isempty(m)
    m = 1;
end
if isempty(tf)
    tf = 10;
end
if isempty(smdelay)
    smdelay = 0.05;
end
if isempty(tres)
    tres = 0.02;
end
if isempty(pres)
    pres = 0.2;
end
if isempty(vres)
    vres = 10/3;
end
if isempty(ures)
    ures = 1000/6;
end
if isempty(c1)
    c1 = 0.15;
end
if isempty(c2)
    c2 = 0.05;
end
if isempty(usigma)
    usigma = 0.015;
end
if isempty(psigma)
    psigma = 1;
end
if isempty(vsigma)
    vsigma = 10;
end
if isempty(asigma)
    asigma = 1;
end
if isempty(ssigma)
    ssigma = 0.5;
end
if isempty(Wenergy)
    Wenergy = 0.00005;
end
if isempty(Wtime)
    Wtime = 0.01;
end
if isempty(Wstop)
    Wstop = 100;
end
if isempty(Wactuator)
    Wactuator = 0.01;
end
if isempty(Wgoal)
    Wgoal = 100;
end
if isempty(Wtimeout)
    Wtimeout = 0;
end
if isempty(tmax)
    tmax = 1;
end
if isempty(vthresh)
    vthresh = 5;
end
if isempty(tinit)
    tinit = 0;
end

%% dependent variables
if isempty(xinit)
    xinit   = [zeros(1,3*mdim) reshape(pgoal',1,ngoal*mdim)]';  % initial state
end
OFC_GlobalVars();

%% dependent defaults
if isempty(Tgoal)
    Tgoal   = T(ceil((1:ngoal)*(tsteps/ngoal)));                % regular interval goal-times
end
if isempty(wgoal)
    wgoal   = ones(1,ngoal);                                    % weights for goal-targets
end

%% save variables to struct
params = ws2struct();
try
    params = rmfield(params,'varargin');
end
 

end
