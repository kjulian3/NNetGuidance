import math
import h5py
import numpy as np
import os
import sys
files = []
if len(sys.argv) > 1:
    files = sys.argv[1:]
else:
    files = os.listdir("./")
for ii in range(len(files)):
    fileName = files[ii]
    if fileName[-2:] == 'h5':
        print "File: " + fileName[:-2]+'nnet'
        f       = h5py.File("./"+fileName,'r')
        with open("./nnet/"+fileName[:-2]+'nnet','w') as f2:
            f2.write("Neural Network File Format by Kyle Julian, Stanford 2016\n")
            keys1 = f.keys();
            keys2 = f[keys1[0]].keys();
            numLayers = len(keys1);
            inputSize = len(f[keys1[0]][keys2[0]]);
            outputSize = len(f[keys1[len(keys1)-1]][keys2[1]])
            maxLayerSize = inputSize;
            for key in keys1:
                if len(f[key][keys2[1]])>maxLayerSize :
                    maxLayerSize = len(f[key][keys2[1]])

            str = "%d,%d,%d,%d,\n" % (numLayers,inputSize,outputSize,maxLayerSize)
            f2.write(str)
            str = "%d," % inputSize
            f2.write(str)
            for key in keys1:
                str = "%d," % len(f[key][keys2[1]])
                f2.write(str)

            f2.write("\n")
            if "DRL" not in fileName:
                if "sym.h5" not in fileName:
                    if "run2a.h5" in fileName:
                        print "RUN_2A!"
                        f2.write("0,\n") #Symmetric Boolean
                        f2.write("0.0,-3.141593,-3.141593,100.0,0.0,0.0,-3.0,\n") #Mininmum Input Values
                        f2.write("60760.0,3.141593,3.141593,1200.0,1200.0,100.0,3.0,\n") #Maximum Input Values
                        f2.write("1.9791091e+04,0.0,0.0,650.0,600.0,35.1111111,0.0,7.5188840201005975,\n") #Means of inputs for normalizations
                        f2.write("60261.0,6.28318530718,6.28318530718,1100.0,1200.0,100.0,6.0,373.94992,\n") #Ranges of inputs for
                    elif "FREGATA" in fileName:
                        print "FREGATA!"
                        f2.write("0\n") #Symmetric Boolean
                        f2.write("14.0, -3.141592653589793, -3.141592653589793,-1.3089969389957472,\n") #Mininmum Input Values
                        f2.write("350.0, 3.141592653589793, 3.141592653589793,1.3089969389957472,\n") #Maximum Input Values
                        f2.write("66.813286341843906, 0.0, 0.0, 0.0, 0.0,\n") #Means
                        f2.write("336.0, 6.2831853071795862, 6.2831853071795862, 2.6179938779914944, 1.0,\n")
                    elif "FREGAT2" in fileName:
                        if "Speed" in fileName:
                            print "FREGAT2! 72B! Speed!"
                            f2.write("0\n") #Symmetric Boolean
                            f2.write("12.0, -3.141592653589793, -3.141592653589793,-1.2566370614359172,18.0\n") #Mininmum Input Values
                            f2.write("350.0, 3.141592653589793, 3.141592653589793,1.2566370614359172,24.0\n") #Maximum Input Values
                            f2.write("58.91827212994, 0.0, 0.0, -0.15824155773913484, 21.0, 0.0,\n") #Means
                            f2.write("338.0, 6.2831853071795862, 6.2831853071795862, 2.5132741228718345, 6.0, 1.0,\n")
                        elif "72B" in fileName:
                            print "FREGAT2! 72B!"
                            f2.write("0\n") #Symmetric Boolean
                            f2.write("12.0, -3.141592653589793, -3.141592653589793,-1.2566370614359172,\n") #Mininmum Input Values
                            f2.write("350.0, 3.141592653589793, 3.141592653589793,1.2566370614359172,\n") #Maximum Input Values
                            f2.write("72.293753090471071, 0.0, 0.0, -0.27361306883859055, 0.0,\n") #Means
                            f2.write("338.0, 6.2831853071795862, 6.2831853071795862, 2.5132741228718345, 1.0,\n") 
                        else:
                            print "FREGAT2!"
                            f2.write("0\n") #Symmetric Boolean
                            f2.write("14.0, -3.141592653589793, -3.141592653589793,-1.3089969389957472,\n") #Mininmum Input Values
                            f2.write("350.0, 3.141592653589793, 3.141592653589793,1.3089969389957472,\n") #Maximum Input Values
                            f2.write("76.644651573436946, 0.0, 0.0, -0.33764531409579845, 0.0,\n") #Means
                            f2.write("336.0, 6.2831853071795862, 6.2831853071795862, 2.6179938779914944, 1.0,\n")
                    else:
                        print "Not Symmetric"
                        f2.write("0,\n") #Symmetric Boolean
                        f2.write("0.0,-3.141593,-3.141593,100.0,0.0,0.0,-3.0,\n") #Mininmum Input Values
                        f2.write("60760.0,3.141593,3.141593,800.0,800.0,100.0,3.0,\n") #Maximum Input Values
                        f2.write("1.9791091e+04,0.0,0.0,450.0,400.0,35.1111111,0.0,3.58839022945,\n") #Means of inputs for normalizations
                        f2.write("60261.0,6.28318530718,6.28318530718,700.0,800.0,100.0,6.0,387.158972402,\n") #Ranges of inputs for normalizations
                else:
                    print "Symmetric Neural Network!"
                    f2.write("1,\n") #Symmetric Boolean
                    f2.write("0.0,-3.141593,0.0,100.0,0.0,0.0,-3.0,\n")
                    f2.write("60760.0,3.141593,3.141593,800.0,800.0,100.0,3.0,\n") #Maximum Input Values
                    f2.write("1.9791091e+04,0.0,1.570796326623,450.0,400.0,35.1111111,0.0,3.58839022945,\n") #Means of inputs for normalizations
                    f2.write("60261.0,6.28318530718,3.141592653589,700.0,800.0,100.0,6.0,387.158972402,\n") #Ranges of inputs for normalizations
            else:
                print "DRL!"
                f2.write("0\n") #Symmetric Boolean
                f2.write("0.0, -3.141593, -3.141593, 10.0, 10.0,\n") #Mininmum Input Values
                f2.write("3000.0, 3.141593, 3.141593, 20.0, 20.0,\n") #Maximum Input Values
                f2.write("1500.0,0.0,0.0,15.0,15.0,0.0\n") #Means of inputs for normalizations
                f2.write("3000.0,6.283185307,6.283185307,10.0,10.0,1.0\n") #Ranges of inputs for normalizations

            #Write weights and biases of neural network
            for key in f.keys():
                for key2 in f[key].keys():
                    data = np.array(f[key][key2]).T
                    for i in range(len(data)):
                        for j in range(int(np.size(data)/len(data))):
                            str = ""
                            if int(np.size(data)/len(data))==1:
                                str = "%.6e," % data[i] #Four digits written. More can be used, but that requires more more space.
                            else:
                                str = "%.6e," % data[i][j]
                            f2.write(str)
                        f2.write("\n")
