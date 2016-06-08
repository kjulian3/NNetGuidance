module PilotSCAViz

export viz_pairwise_policy, Policy, read_policy, evaluate, get_belief, get_qval
export NNet, evaluate_network, evaluate_network_multiple

using GridInterpolations, Interact, PGFPlots
using PilotSCAConst

type Policy
    alpha       :: Matrix{Float64}
    actions     :: Matrix{Float64}
    nactions    :: Int64
    qvals       :: Vector{Float64}

    function Policy(alpha::Matrix{Float64}, actions::Matrix{Float64})
        return new(alpha, actions, length(actions), zeros(length(actions)))
    end # function Policy
end

function read_policy(actions::Matrix{Float64}, alpha::Matrix{Float64})
    return Policy(alpha, actions)
end # function read_policy

function evaluate(policy::Policy, belief::SparseMatrixCSC{Float64,Int64})
    fill!(policy.qvals, 0.0)
    get_qval!(policy, belief)
    ibest = indmax(policy.qvals)
    return policy.actions[:, ibest], ibest
end # function evaluate

function get_qval!(policy::Policy, belief::SparseMatrixCSC{Float64, Int64})
    fill!(policy.qvals, 0.0)
    for iaction in 1:policy.nactions
        for ib in 1:length(belief.rowval)
            policy.qvals[iaction] += belief.nzval[ib] * policy.alpha[belief.rowval[ib], iaction]
        end # for b
    end # for iaction
end # function get_qval!

function get_belief(pstate::Vector{Float64}, grid::RectangleGrid,interp::Bool=true)
    belief = spzeros(NStates, 1)
    indices, weights = interpolants(grid, pstate)
    if !interp
        largestWeight = 0;
        largestIndex = 0;
        for i = 1:length(weights)
            if weights[i]>largestWeight
                largestWeight = weights[i]
                largestIndex = indices[i]
            end
        end
        indices = largestIndex
        weights = 1.0
    end
    for i = 1:length(indices)
        belief[indices[i]] = weights[i]
    end # for i
    return belief
end # function get_belief

type NNet
    file::AbstractString
    weights::Array{Any,1}
    biases::Array{Any,1}
    symmetric::Int32
    numLayers::Int32
    inputSize::Int32
    outputSize::Int32
    maxLayerSize::Int32
    
    layerSizes::Array{Int32,1}
    mins::Array{Float64,1}
    maxes::Array{Float64,1}
    means::Array{Float64,1}
    ranges::Array{Float64,1}
    
    function NNet(file::AbstractString)
        this  = new()
        this.file = file
        f = open(this.file)
        line = readline(f)
        line = readline(f)
        record = split(line,[',','\n'])
        this.numLayers = parse(Int32,record[1])
        this.inputSize = parse(Int32,record[2])
        this.outputSize = parse(Int32,record[3])
        this.maxLayerSize=parse(Int32,record[4])
        
        line = readline(f)
        record = split(line,[',','\n'])
        this.layerSizes = zeros(this.numLayers+1)
        for i=1:(this.numLayers+1)
            this.layerSizes[i]=parse(Int32,record[i])
        end
        
        line = readline(f)
        record = split(line,[',','\n'])
        this.symmetric = parse(Int32,record[1])
        
        line = readline(f)
        record = split(line,[',','\n'])
        this.mins = zeros(this.inputSize)
        for i=1:(this.inputSize)
            this.mins[i]=parse(Float64,record[i])
        end
        
        line = readline(f)
        record = split(line,[',','\n'])
        this.maxes = zeros(this.inputSize)
        for i=1:(this.inputSize)
            this.maxes[i]=parse(Float64,record[i])
        end
        
        
        line = readline(f)
        record = split(line,[',','\n'])
        this.means = zeros(this.inputSize+1)
        for i=1:(this.inputSize+1)
            this.means[i]=parse(Float64,record[i])
        end
        
        line = readline(f)
        record = split(line,[',','\n'])
        this.ranges = zeros(this.inputSize+1)
        for i=1:(this.inputSize+1)
            this.ranges[i]=parse(Float64,record[i])
        end
        
        
        this.weights = Any[zeros(this.layerSizes[2],this.layerSizes[1])]
        this.biases  = Any[zeros(this.layerSizes[2])]
        for i=2:this.numLayers
            this.weights = [this.weights;Any[zeros(this.layerSizes[i+1],this.layerSizes[i])]]
            this.biases  = [this.biases;Any[zeros(this.layerSizes[i+1])]]
        end
        
        layer=1
        i=1
        j=1
        line = readline(f)
        record = split(line,[',','\n'])
        while !eof(f)
            while i<=this.layerSizes[layer+1]
                while record[j]!=""
                    this.weights[layer][i,j] = parse(Float64,record[j])
                    j=j+1
                end
                j=1
                i=i+1
                line = readline(f)
                record = split(line,[',','\n'])
            end
            i=1
            while i<=this.layerSizes[layer+1]
                this.biases[layer][i] = parse(Float64,record[1])
                i=i+1
                line = readline(f)
                record = split(line,[',','\n'])
            end
            layer=layer+1
            i=1
            j=1
        end
        close(f)
        
        return this
    end
end

#Evaluates one set of inputs
function evaluate_network(nnet::NNet,input::Array{Float64,1})
    numLayers = nnet.numLayers
    inputSize = nnet.inputSize
    outputSize = nnet.outputSize
    symmetric = nnet.symmetric
    biases = nnet.biases
    weights = nnet.weights
    
    inputs = zeros(inputSize)
    for i = 1:inputSize
        if input[i]<nnet.mins[i]
            inputs[i] = (nnet.mins[i]-nnet.means[i])/nnet.ranges[i]
        elseif input[i] > nnet.maxes[i]
            inputs[i] = (nnet.maxes[i]-nnet.means[i])/nnet.ranges[i] 
        else
            inputs[i] = (input[i]-nnet.means[i])/nnet.ranges[i] 
        end
    end
    if symmetric ==1 && inputs[2]<0
        inputs[2] = -inputs[2]
        inputs[1] = -inputs[1]
    else
        symmetric = 0
    end
    for layer = 1:numLayers-1
        temp = max(*(weights[layer],inputs[1:nnet.layerSizes[layer]])+biases[layer],0)
        inputs = temp
    end
    outputs = *(weights[end],inputs[1:nnet.layerSizes[end-1]])+biases[end]
    for i=1:outputSize
        outputs[i] = outputs[i]*nnet.ranges[end]+nnet.means[end]
    end
    return outputs
end

#Evaluates multiple inputs at once. Each set of inputs should be a column in the input array
#Returns a column of output Q values for each input set
function evaluate_network_multiple(nnet::NNet,input::Array{Float64,2})
    numLayers = nnet.numLayers
    inputSize = nnet.inputSize
    outputSize = nnet.outputSize
    symmetric = nnet.symmetric
    biases = nnet.biases
    weights = nnet.weights
        
    _,numInputs = size(input)
    symmetryVec = zeros(numInputs)
    
    inputs = zeros(inputSize,numInputs)
    for i = 1:inputSize
        for j = 1:numInputs
            if input[i,j]<nnet.mins[i]
                inputs[i,j] = (nnet.mins[i]-nnet.means[i])/nnet.ranges[i]
            elseif input[i,j] > nnet.maxes[i]
                inputs[i,j] = (nnet.maxes[i]-nnet.means[i])/nnet.ranges[i] 
            else
                inputs[i,j] = (input[i,j]-nnet.means[i])/nnet.ranges[i] 
            end
        end
    end
    for i=1:numInputs
        if symmetric ==1 && inputs[2,i]<0
            inputs[2,i] = -inputs[2,i]
            inputs[1,i] = -inputs[1,i]
            symmetryVec[i] = 1
        else
            symmetryVec[i] = 0
        end
    end
    
    for layer = 1:numLayers-1
        inputs = max(*(weights[layer],inputs[1:nnet.layerSizes[layer],:])+*(biases[layer],ones(1,numInputs)),0)
    end
    outputs = *(weights[end],inputs[1:nnet.layerSizes[end-1],:])+*(biases[end],ones(1,numInputs))
    for i=1:outputSize
        for j=1:numInputs
            outputs[i,j] = outputs[i,j]*nnet.ranges[end]+nnet.means[end]
        end
    end
    return outputs
end



function viz_pairwise_policy(
        alpha::Matrix{Float64},
        neuralNetworkPath)
    
    nnet = []
    batch_size = 500
    if neuralNetworkPath!=""
        nnet = NNet(neuralNetworkPath);
    end
    grid = RectangleGrid(Ranges, Thetas, Bearings, Banks,Speeds)
    policy = read_policy(ACTIONS', alpha)

    @manipulate for bearing in round(Int,Bearings*180/pi),
        bank in round(Int,Banks*180/pi),
        speed in round(Int,[Speeds;14.0]),
        nbin = [250,200,150,100,50],
        scale = [4.0, 3.0, 2.0, 1.5, 1.0],
        interp = [true,false],
        savePlot = [false,true]
        
        
        #Load table with the inputs needed to plot the heat map
        if nnet!=[]
            inputsNet= zeros(nbin*nbin,5)    
            ind = 1
            for i=linspace(round(Int,-1*Rangemax/scale),round(Int,Rangemax/scale),nbin)
                for j=linspace(round(Int,-1*Rangemax/scale),round(Int,Rangemax/scale),nbin)
                    r = sqrt(i^2+j^2)
                    th = atan2(j,i)
                    inputsNet[ind,:] = [r,th,deg2rad(bearing),deg2rad(bank),speed];
                    ind = ind+1
                end
            end

            #Calculate all of the Q values from the input array
            q_nnet = zeros(nbin*nbin,2);
            ind = 1

            while ind+batch_size<nbin*nbin
                input = inputsNet[ind:(ind+batch_size-1),:]'
                output = evaluate_network_multiple(nnet,input) 
                q_nnet = [q_nnet[1:(ind-1),:];output';q_nnet[ind+batch_size:end,:]]
                ind=ind+batch_size
            end
            input = inputsNet[ind:end,:]'
            output = evaluate_network_multiple(nnet,input)
            q_nnet = [q_nnet[1:(ind-1),:];output']
        end
        
        # ownship uav
        function get_heat1(x::Float64, y::Float64) 
            r = norm([x,y])
            th = atan2(y,x)
            action, _ = evaluate(policy, get_belief(
            [r, th, deg2rad(bearing), deg2rad(bank),speed], grid,interp)) 
            return rad2deg(action[1])
        end # function get_heat1
        
        ind = 1
        #Neural Net Heat Map
       function get_heat2(x::Float64, y::Float64)                          
           qvals = q_nnet[ind,:]
           ind +=1
           return rad2deg(ACTIONS[indmax(qvals)])
       end # function get_heat2
        
        
        g = GroupPlot(2, 1, groupStyle = "horizontal sep=3cm")
        push!(g, Axis([
            Plots.Image(get_heat1, (round(Int,-Rangemax/scale), round(Int,Rangemax/scale)), 
            (round(Int,-Rangemax/scale), round(Int,Rangemax/scale)), 
                        zmin = -5, zmax = 5,
                        xbins = nbin, ybins = nbin,
            colormap = ColorMaps.Named("Blues"), colorbar = true),
            Plots.Node(L">", 0, 0, style="rotate=0,font=\\Huge"),
            Plots.Node(L">", Rangemax/300.0*260.0/scale, Rangemax/300.0*260.0/scale, style=string("rotate=", bearing, ",font=\\Huge"))
            ], width="10cm", height="10cm", xlabel="x (m)", ylabel="y (m)", title="Table Policy"))

        if nnet!=[]
            push!(g, Axis([
                Plots.Image(get_heat2, (round(Int,-Rangemax/scale), round(Int,Rangemax/scale)), 
                (round(Int,-Rangemax/scale), round(Int,Rangemax/scale)), 
                            zmin = -5, zmax = 5,
                            xbins = nbin, ybins = nbin,
                colormap = ColorMaps.Named("Blues"), colorbar = true),
                Plots.Node(L">", 0, 0, style="rotate=0,font=\\Huge"),
                Plots.Node(L">", Rangemax/300.0*260.0/scale, Rangemax/300.0*260.0/scale, style=string("rotate=", bearing, ",font=\\Huge"))
                ], width="10cm", height="10cm", xlabel="x (m)", ylabel="y (m)", title="Neural Network Policy"))
        end
        if savePlot
            save("UAV_PolicyPlot.tex",g,include_preamble=false)
        else
            g
        end
        
    end 
end # function viz_pairwise_policy

end # module PilotSCAViz