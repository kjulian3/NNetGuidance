# NNetGuidance

This repository supplies the code used to create a neural network guidance system for the Fregata team in the Spring 2016 class of AA241X at Stanford. There are four main components in this code: MDP table generation, neural network regression, neural network text file generation, and Pixhawk software. Here is how to use each part:

#### MDP table generation
The parameters of the MDP are defiend in src/pilot/*jl. These files define the state and action spaces, rewards, and transitions. These functions get called by the solveRegular function in sr/dvi/parallel.jl. The bottom part of this function defines how frequently and where the tables are written as JLD files, so make sure to edit that section before use! The solveRegular function gets called by src/RPilotSCA.jl. Therefore, to run the code and and generate your own MDP solutions, you need run the julia script by calling "julia RPilotSCA.jl", given that you have installed julia and it is on your path. This will produce large JLD files. They are too large to have an example file in this repository.


#### Neural Network Regression
First, the MDP solution must be converted from JLD table form to input and output training data for a network. The file src/Fregata_genData.ipynb contains the code to do just that. It creates normalized input and output data to train the network. The most critical data points, those with steep banks and close ranges, are written multiple times in the training data. Make sure to save the mean and range values, because those will be used in the src/data/convertHDF5.py later.

The result is an h5 data file with all of the training data for the neural network. The file /src/Fregata_crossEnt4.py uses the python library Keras along with the training data to train a neural network. The file names defined in the bottom section of this file define where you will save the parameters of the neural network in h5 format.

#### Neural Network File Generation
After training a neural network, the h5 output file is converted to a custom text file so that the C++ software flown on the pixhawk can easliy read the nueral network parameters. The file src/data/conertHDF5.py contains the function to write imprortant network information and the parameters of the network in a custom text file format. Note that you must specify the means, ranges, mins, and maxes of the inputs here. Calling "python convertHDF5.py *.h5" will convert all of the h5 network files into text files and place them in the nnet folder. 

#### Pixhawk Software
The folder src/Pixhawk contains the high and low modules used with the class version of the firmware. In the low module, there are two files, nnet.cpp and nnet.h, that define how the neural network is loaded from memory to create a neural network object. Those files also contain functions to evaluate a neural network object given a set of inputs and space in memory in which to write the outputs. The file Race72.nnet must be loaded onto the microSD card so that the software can read in the network parameters.
