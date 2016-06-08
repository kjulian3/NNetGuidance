
push!(LOAD_PATH, "./dvi")
push!(LOAD_PATH, "./pilot")

addprocs(round(Int,(CPU_CORES / 2)))

using DiscreteValueIteration2, JLD, PilotSCAs

mdp = SCA()

# check size of mdp
function getBytes(x)
   total = 0;
   fieldNames = fieldnames(typeof(x));
   if fieldNames == ()
      return sizeof(x);
   else
     for fieldName in fieldNames
        total += getBytes(getfield(x,fieldName));
     end
     return total;
   end
end

println("mdp of type ", typeof(mdp), " takes up ", getBytes(mdp) / 1000.0, " kB")

# informal validation of transition function
nextStateIndices, probs = nextStates(mdp, 1, 1)
println("next state indices:\n", nextStateIndices, "\n")
println("probabilities:\n", probs, "\n")
println("probabilities sum to ", sum(probs))



# parallel solution
numProcs = round(Int,(CPU_CORES / 2))
numProcs = 6
solver = ParallelSolver(
    numProcs,
    maxIterations = 600,
    tolerance = 1e-5,
    gaussSiedel = false,  # true
    includeV = true,
    includeQ = true,
    includeA = true)

println("\nStarting parallel solver...")
policy = solve(solver, mdp, verbose = true)
println("\nParallel solution generated!")