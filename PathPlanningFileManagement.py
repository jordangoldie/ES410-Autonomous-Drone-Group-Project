

def StoreLocations(nameoffile,finalcords):
    # save to csv file
    # "OrderOfLocations.csv"
    latitudes = []
    longitudes = []
    plantornot = []
    data_file = open(nameoffile,"w+")
    for i in range(len(finalcords)):
        latitudes.append(finalcords[i][0])
        longitudes.append(finalcords[i][1])
        plantornot.append(finalcords[i][2])
    data_file.write(str(latitudes) + "\n")
    data_file.write(str(longitudes) + "\n")
    data_file.write(str(plantornot) + "\n")
    data_file.close()
    print("Locations saved in CSV file saved as " + nameoffile)



def ExtractLocations(nameoffile):
    # reading from csv file
    # "OrderOfLocations.csv"
    datafile = open(nameoffile,"r")
    latitudes1 = datafile.readline()
    longitudes1 = datafile.readline()
    plantornot1 = datafile.readline()


    latitudes11 =[]
    for i in latitudes1:
        latitudes11.append(i)
    for i in latitudes11:
        if (i == "[") | (i == "]") | (i == " "):
            latitudes11.remove(i)
    latitudes11.remove("\n")
    aaa = ''.join(latitudes11)
    bbb = str(aaa)
    ccc = bbb.split(",")
    latitudes = []
    for i in ccc:
        ddd = float(i)
        latitudes.append(ddd)
    #print(latitudes)


    longitude11 =[]
    for i in longitudes1:
        longitude11.append(i)
    for i in longitude11:
        if (i == "[") | (i == "]") | (i == " "):
            longitude11.remove(i)
    longitude11.remove("\n")
    aaa = ''.join(longitude11)
    bbb = str(aaa)
    ccc = bbb.split(",")
    longitudes = []
    for i in ccc:
        ddd = float(i)
        longitudes.append(ddd)
    #print(longitudes)


    plantornot11 =[]
    for i in plantornot1:
        plantornot11.append(i)
    for i in plantornot11:
        if (i == "[") | (i == "]") | (i == " "):
            plantornot11.remove(i)
    plantornot11.remove("\n")
    aaa = ''.join(plantornot11)
    bbb = str(aaa)
    ccc = bbb.split(",")
    plantornot = []
    for i in ccc:
        ddd = int(i)
        plantornot.append(ddd)
    #print(plantornot)

    return (latitudes,longitudes,plantornot)