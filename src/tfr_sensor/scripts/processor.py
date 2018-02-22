#! /usr/bin/python3
"""utility script for parsing data from ros msgs

Takes in a file on stdin, proccesses yaml and throws numeric data into numpy arrays
for statistics processing
Expects data from rostopic echo. E.G. msgs seperated by ---"""

import sys
import numpy

def grabData(tokens, targetString, numDimensions):
    """Finds a target string in a list, and returns N data points following that string,
    discarding labels.
    
    E.X.
    list= [orientation:, x:, 5,y:, 6, z:, 7]
    result = findAndGrabData(list, "orientation", 3)
    result => [5,6,7] """
    idx = tokens.index(targetString)
    #grab all of the data elements discarding labels, from desired index
    return [float(elem) for elem in tokens[ idx+2 : idx+2+2*numDimensions : 2 ]]


    
if __name__ == "__main__":
    
    #temporary storage
    orientation = []
    angular_velocity = []
    linear_acceleration = []

    #process the data one message at at time
    for line in sys.stdin.read().split("---"):

        #break the message into lines
        tokens = line.strip().split()
        if len(tokens) == 0:
            continue

        #parse some YAML
        orientation.append(grabData(tokens, "orientation:", 4))
        angular_velocity.append(grabData(tokens, "angular_velocity:", 3))
        linear_acceleration.append(grabData(tokens, "linear_acceleration:", 3))

    #throw it into num py
    orientation = numpy.array(orientation)
    angular_velocity = numpy.array(angular_velocity)
    linear_acceleration = numpy.array(linear_acceleration)

    #calculating statistics is easy see examples for variance, averages, and stddev
    print(  "Orientation", 
            orientation, 
            "Orientation Variances", 
            orientation.var(axis=0), sep="\n")
    print(  "Angular Velociy", 
            angular_velocity, 
            "Angular Velocity Means", 
            angular_velocity.mean(axis=0), sep="\n")
    print(  "Linear_acceleration",
            linear_acceleration, 
            "Linear Acceleration Std Deviations",
            linear_acceleration.std(axis=0), sep="\n")

