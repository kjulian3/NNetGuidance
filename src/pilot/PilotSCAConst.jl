module PilotSCAConst

export DT, DTI, G, Bearingmin, Bearingmax
export Rangemin, Rangemax, Thetamin, Thetamax

export  Bearingdim, Speeddim, NStates, NActions
export Rangedim, Thetadim

export Bearings, Speeds, Actions, ACTIONS
export Ranges, Thetas, Banks, Bankdim, Bankmin

export SigmaSpeed, SigmaBank, SigmaBankCOC
export SigmaDim, SigmaWeightNominal, SigmaWeightOffNominal
export RangeTol, BankTol, BearingTol, BankGoal, Bankmax, NominalSpeed
export Speeds, Speedmin, Speedmax,Speeddim

const DTI = 0.1 # [s]

const Rangemin = 0.0 # [m]
const Rangemax = 300.0 #[m]
const Thetamin = -pi #[rad]
const Thetamax = pi #[rad]

const Bearingmin = -pi  # [rad]
const Bearingmax = pi  # [rad]
const Bankmin    = -pi/180.0*72.0
const Bankmax    = pi/180.0*72.0

const NActions = 2
const STATE_DIM = 4
const ACTION_DIM = 2

const Ranges = [0.,  2.,  4.,  5.,  6.,  8.,  10., 12.,
                14., 16., 18., 20., 23., 26., 29., 32., 35., 38.,
                41., 44., 47., 50., 55., 60., 65., 70., 75., 80., 
                85., 90., 95., 100.,105.,110.,120.,130.,140.,150.,
                160.,170.,180.,190.,200.,230.,260.,300.,350.]
const Thetas = edges_theta = [-180.0,-175.0,-162.0,-150.0, -135.0,-120.0,-105.0,-95.0,-85.0,-75.0,-60.0,-50.0,-40.0,-35.0,-30.0,-26.0,-22.0,
    -18.0,-15.0,-12.0,-9.0,-6.0,-3.0,-1.0,0.0,
    1.0,3.0,6.0,9.0,12.0,15.0,18.0,22.0,26.0,30.0,35.0,40.0,50.0,60.0,75.0,85.0,95.0,105.0,120.0,135.0,150.0,162.0,175.0,180.0] *pi/180.0

Speedmin = 18.0
Speedmax = 24.0
Speeddim = 4
const Speeds = collect(linspace(Speedmin,Speedmax,Speeddim))
const Banks = [-72.0,-69.0,-66.0,-63.0,-60.0,-57.0,-54.0,-48.0,-42.0,-36.0,-30.0,-21.0,-12.0,-6.0,-3.0,0.0,3.0,6.0,12.0,21.0,30.0,36.0,42.0,48.0,54.0,57.0,60.0,63.0,66.0,69.0,72.0]*pi/180.0
const Actions = [:right,:left]
const ACTIONS = deg2rad([-5, 5])

const Rangedim = length(Ranges)
const Thetadim = length(Thetas)
const Bearingdim = 49#length(Bearings)
const Bearings = collect(linspace(Bearingmin, Bearingmax, Bearingdim))
const Bankdim = length(Banks)
const NStates = Rangedim*Thetadim*Bearingdim*Bankdim*Speeddim+1

const RangeTol   = 6.0000000000001
const BearingTol = 8.0000000000001*pi/180.0
const BankTol    = 6.0000000000001*pi/180.0
const BankGoal   =-72.0pi/180.0


const NominalSpeed = 22.0
const SigmaSpeed = 0.0  # [m/s]
const SigmaBank = deg2rad(0.0)  # [rad]

const SigmaDim = 2  # number of dimensions for sigma-point sampling
const SigmaWeightNominal = 1 / 2
const SigmaWeightOffNominal = (1 - SigmaWeightNominal) / (2 * SigmaDim)

end # module PilotSCAConst
