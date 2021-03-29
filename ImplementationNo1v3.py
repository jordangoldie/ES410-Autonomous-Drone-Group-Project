# BRUTE FORCE/TRIAL METHOD
import math
from itertools import permutations
from itertools import combinations
from sys import maxsize

def TSP(distances,startingnodeindex):
    print("Entered TSP")
    # store all nodes apart from starting node
    nodeindex = []
    for i in range(NumberOfLocations):
        if i != startingnodeindex:
            nodeindex.append(i)
    # store minimum cost Hamiltonian Cycle
    minpath = maxsize # sets minimum path distancce to max value computer can hold
    finalpermutation = None
    nextpermutation = permutations(nodeindex) # finds all possible permutations
    for i in nextpermutation:
        print(i)
        currentpathcost = 0 # store current path cost
        # compute current path total cost
        k = startingnodeindex
        for j in i:
            currentpathcost += distances[k][j]
            k = j
        currentpathcost += distances[k][startingnodeindex]
        print(currentpathcost)
        # update minimum cost and path
        if currentpathcost < minpath:
            minpath = currentpathcost
            finalpermutation = i
    print("Exited TSP")
    return [finalpermutation,minpath]


def calculatedistance(a,b):
    c = ((a[0]-b[0])**2) + ((a[1]-b[1])**2)
    c = math.sqrt(c)
    return c




locations = [(-98.7412086605092,258.161897831941),
(-107.757411144327,278.900541727472),
(-78.5553131438378,279.786961388307),
(-71.5087690801748,250.468222327575),


(-80.3069604108349,295.681309756639),
(-106.185794649048,261.098374131895),
(-84.5005630565392,258.862124401948),
(-99.9520301303023,282.700612795295)]

'''
(-71.0479155873529,283.648610686413),
(-63.6515031253764,255.706399625704),
(-79.7629036144391,244.965465976725),
(-87.900829533046,246.849980061202),
(-78.4370975914964,271.605611765227),
(-99.3763062393439,266.33607489905),
(-60.9313983065162,280.145084253592),
(-70.9540395959419,267.404695854302),
(-77.7069455289304,263.086667085904),
(-106.789506530978,286.853794283047),
(-63.7606334175286,263.903994845594),
(-79.1208274072562,252.941107230563),
(-57.0841381719665,268.603303872547),
(-56.2473518223636,260.642500489602),
(-88.2388408834472,293.037349489681),
(-97.9532741844152,250.200794939667),
(-72.2871724277928,291.671450251201),
(-113.094145817983,265.132563533333),
(-91.2555359780359,254.576060843174),
(-106.690955717116,253.113355446182),
(-100.104833148197,274.702072226059),
(-79.7039942822176,287.704065129091),
(-113.735637842255,273.584452417822),
(-71.3977814177339,275.65626472914),
(-106.620439705541,269.927410515505),
(-64.0710676238549,287.563397856467),
(-91.8190782623457,262.680075798255),
(-96.4905104764811,290.286792962767),
(-85.5101976068615,275.813630950923),
(-92.5248658366133,271.037099739658),
(-71.1750897677379,258.467699634733),
(-85.1499274366892,267.253840232907),
(-92.7428120692244,279.232591032412),
(-63.9369731123897,272.731094235358),
(-87.2510191500269,285.049824956938)]
'''

NumberOfLocations = len(locations)

# create distances graph [(node 0's distance to every other node), (node 1's distance to every other node), ...]
distances = []
for LocationIndex in range(NumberOfLocations):
    #print(LocationIndex)
    nodedistances = []
    #distances.append(locations[LocationIndex])
    for j in range(NumberOfLocations): # for every location
        nodedistance = calculatedistance(locations[LocationIndex],locations[j])
        nodedistances.append(nodedistance)
    #print(nodedistances)
    distances.append(nodedistances)
print(distances)


# program
print("Starting Program")
StartingNodeIndex = 0
[a1,a2] = TSP(distances,StartingNodeIndex)
print("The inbetween node indexes: ")
print(a1)
print("The total cost of above path: ")
print(a2)
print("Exited Program")

# need to add
# 1) distance and node from final node in list back to start node.? not essential for our case