# implements value iteration solver for MDPs

module DiscreteValueIteration2

export ParallelSolver
export SerialSolver
export solve
export DiscretePolicy
export action, value

using DiscreteMDPs

import DiscreteMDPs.Solver
import DiscreteMDPs.solve
import DiscreteMDPs.Policy
import DiscreteMDPs.action
import DiscreteMDPs.value

include("helpers.jl")
include("policy.jl")
include("serial.jl")
include("parallel.jl")

end # module
