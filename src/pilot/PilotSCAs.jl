# TODO: consider linked list to avoid concatenation costs
module PilotSCAs

export SCA
export numStates, numActions
export reward, nextStates

export ind2a, state2gridState, gridState2state, index2state, ind2x

export State, Action

using GridInterpolations, DiscreteMDPs

import DiscreteMDPs.DiscreteMDP
import DiscreteMDPs.reward
import DiscreteMDPs.nextStates
import DiscreteMDPs.numStates
import DiscreteMDPs.numActions

using PilotSCAConst

type SCA <: DiscreteMDP
    nStates::Int64
    nActions::Int64
    actions::Vector{Symbol}
    grid::RectangleGrid
    function SCA()
        grid = RectangleGrid(Ranges, Thetas, Bearings, Banks,Speeds)
        return new(NStates, NActions, Actions, grid)
    end # function SCA
end # type SCA

type State
    r::Float64
    th::Float64
    bearing::Float64
    bank::Float64
    speed::Float64
end # type State

type Action
    ownship::Symbol
end # type Action

function numStates(mdp::SCA)    
    return mdp.nStates
end # function numStates

function numActions(mdp::SCA)
    return mdp.nActions
end # function numActions

function reward(mdp::SCA, istate::Int64, iaction::Int64)
    state = State(
        0.0,  # r
        0.0,  # th
        0.0,  # bearing
        BankGoal,  # bank
        Speedmin,
        )

    if istate < mdp.nStates
        state = gridState2state(ind2x(mdp.grid, istate))
    end # if

    action = ind2a(mdp.actions, iaction)
    return reward(mdp, state, action)
end # function reward


function reward(mdp::SCA, state::State, action::Action)
    reward = 0.0
    if !checkGoalState(state)
        reward -= 1.0
    end
    
    return reward
end # function reward

function checkGoalState(state::State)
    if (state.r > RangeTol) || (abs(state.bearing) > BearingTol) || ((abs(state.bank-BankGoal))> BankTol)
        return false
    end
    return true
end

function ind2a(actions::Vector{Symbol}, iaction::Int64)
    iOwnship = iaction % length(actions)
    if iOwnship == 0
        iOwnship = length(actions)
    end # if
    return Action(actions[round(Int,iOwnship)])
end # function ind2a

# Returns turn angle corresponding to action in degrees.
function getTurnAngle(action::Symbol)    
    if action == :right
        return -5.0/180.0*pi
    elseif action == :left
        return 5.0/180.0*pi
    else
        throw(ArgumentError("illegal action symbol"))
    end # if    
end # function getTurnAngle

function getNextState(
        state::State,
        action::Action,
        sigmaTurnOwnship::Float64 = 0.0,
        sigmaSpeedOwnship::Float64 = 0.0,
        dt::Float64 = DTI)
            
    newState = State(
        state.r,
        state.th,
        state.bearing,
        state.bank,
        state.speed
        )
    
    if !checkGoalState(state)
        bank = newState.bank #+ getTurnAngle(action.ownship)
        turnOwnship = bank + sigmaTurnOwnship


        p2 = newState.bearing
        v1 = state.speed

        r = newState.r 
        th = newState.th 
        
        psi1 = 9.80*tan(turnOwnship)/v1*dt
        
        x2 = r*cos(th)
        y2 = r*sin(th) 
        x1 = v1*dt
        y1 = 0
        if (abs(turnOwnship)>0.000001)
            radius = abs(v1^2/9.80/tan(turnOwnship))
            x1 = radius*abs(sin(psi1))
            y1 = radius*(1-cos(psi1))*sign(psi1)
        end
            
        
        pr = p2-psi1
        xabs = x2-x1
        yabs = y2-y1

        r= norm([yabs,xabs])
        th = atan2(yabs,xabs) - psi1

        if pr>pi
            pr= pr -2*pi
        elseif pr<-pi
            pr = pr+2*pi
        end
        if th>pi
            th = th -2*pi
        elseif th<-pi
            th = th+2*pi
        end

        bearing = pr
        
        
        bank += getTurnAngle(action.ownship)
        if bank>Bankmax
            bank = Bankmax
        elseif bank<Bankmin
            bank = Bankmin
        end
        
        newState.bearing = bearing
        newState.r = r
        newState.th = th
        newState.bank = bank
        newState.speed = v1

        if checkGoalState(newState)
            newState.bearing = 0.0
            newState.r = 0.0
            newState.th = 0.0
            newState.bank = BankGoal
        end 
    end
    return newState    
end # function getNextState



# Returns next states and associated transition probabilities.
function nextStates(mdp::SCA, istate::Int64, iaction::Int64)
    if istate == mdp.nStates
        return [mdp.nStates], [1.0]
    end # if

    state = gridState2state(ind2x(mdp.grid, istate))
    action = ind2a(mdp.actions, iaction)
    return sigmaSample(mdp, state, action)
end # function nextStates

function sigmaSample(mdp::SCA, state::State, action::Action)
    nominalIndices, nominalProbs = nextStatesSigma(mdp, state, action)
    speedIndices, speedProbs = sigmaSpeed(mdp, state, action)
    bankIndices, bankProbs = sigmaBank(mdp, state, action)
    return [
            nominalIndices;
            speedIndices;
            bankIndices], 
        [
            nominalProbs * SigmaWeightNominal;
            speedProbs * SigmaWeightOffNominal;
            bankProbs * SigmaWeightOffNominal]
end # function sigmaSample

function sigmaSpeed(mdp::SCA, state::State, action::Action)
    # negative sigma
    negIndices, negProbs = nextStatesSigma(mdp, state, action,0.0,-SigmaSpeed)

    #positive sigma
    posIndices, posProbs = nextStatesSigma(mdp, state, action,0.0, SigmaSpeed)

    return [negIndices; posIndices],
           [negProbs;   posProbs]
end # function sigmaSpeed

function sigmaBank(mdp::SCA, state::State, action::Action)

    # negative sigma
    negIndices, negProbs = nextStatesSigma(mdp, state, action, -SigmaBank,0.0)

    # positive sigma
    posIndices, posProbs = nextStatesSigma(mdp, state, action, SigmaBank,0.0)

    return [negIndices; posIndices],
           [negProbs;   posProbs]
end # function sigmaBank

function nextStatesSigma(
        mdp::SCA,
        state::State,
        action::Action,
        sigmaTurn::Float64 = 0.0,
        sigmaSpeed::Float64 = 0.0
        )
    
    trueNextState = getNextState(state, action, sigmaTurn,sigmaSpeed)
    gridNextState = state2gridState(trueNextState)
    
    if checkGoalState(trueNextState)
        return [mdp.nStates], [1.0]
    else
        return interpolants(mdp.grid, gridNextState)
    end # if
end # function nextStatesSigma

function state2gridState(state::State)
   return [
        state.r,
        state.th,
        state.bearing,
        state.bank,
        state.speed,
    ]
end # function state2gridState

function index2state(mdp::SCA, stateIndices::Vector{Int64})
    states = Array(State, length(stateIndices))
    
    for index = 1:length(stateIndices)
        states[index] = gridState2state(ind2x(mdp.grid, index))
    end # for index
    
    return states   
end # function index2state

function gridState2state(gridState::Vector{Float64})    
    

    return State(
        gridState[1],  # r
        gridState[2],  # th
        gridState[3],  # bearing
        gridState[4],  # bank
        gridState[5],  #speed
        )
end # function gridState2state

end # module PilotSCAs