import json, sys

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

#Load a json config into a dictionary
def load_json(string):

    try:
        fileread = open(string, "r")
    except:
        print(bcolors.FAIL + "--- Could not open file")
        return -1

    text = fileread.read()

    try:
        data = json.loads(text)
    except:
        print(bcolors.FAIL + "--- ERROR: Could not load data")
        return -2

    return data

json_file = load_json(sys.argv[1])

print("Moving objects in definition")

if bool(sys.argv[5]) == True:
    if "ab" in json_file["paddock"]:
        json_file["paddock"]["ab"]["start"]["lat"] = json_file["paddock"]["ab"]["start"]["lat"]+float(sys.argv[3])
        json_file["paddock"]["ab"]["start"]["lon"] = json_file["paddock"]["ab"]["start"]["lon"]+float(sys.argv[4])
        json_file["paddock"]["ab"]["end"]["lat"] = json_file["paddock"]["ab"]["end"]["lat"]+float(sys.argv[3])
        json_file["paddock"]["ab"]["end"]["lon"] = json_file["paddock"]["ab"]["end"]["lon"]+float(sys.argv[4])
        print(bcolors.OKGREEN + "Moved the AB line")
    else:
        print(bcolors.WARNING + "--- WARNING: No AB line found in definition")
else:
    print(bcolors.OKGREEN + "Did not move the AB line")

if "obstacles" in json_file["paddock"]:
    found_obstacles = 0
    for elem in json_file["paddock"]["obstacles"]:
        for point in elem["boundaries"]:
            point["lat"] = point["lat"]+float(sys.argv[3])
            point["lon"] = point["lon"]+float(sys.argv[4])
        found_obstacles = found_obstacles + 1
    if found_obstacles != 0:
        print(bcolors.OKGREEN + "Moved " + str(found_obstacles) + " obstacles")
    else:
        print(bcolors.WARNING + "--- WARNING: No obstacles found in definition")

if "boundaries" in json_file["paddock"]:
    for point in json_file["paddock"]["boundaries"]:
        point["lat"] = point["lat"]+float(sys.argv[3])
        point["lon"] = point["lon"]+float(sys.argv[4])
    print(bcolors.OKGREEN + "Moved the boundary (geofence)")
else:
    print(bcolors.WARNING + "--- WARNING: No geofence found in definition")

if "green_area" in json_file["paddock"]:
    for point in json_file["paddock"]["green_area"]:
        point["lat"] = point["lat"]+float(sys.argv[3])
        point["lon"] = point["lon"]+float(sys.argv[4])
    print(bcolors.OKGREEN + "Moved the green_area")
else:
    print(bcolors.WARNING + "--- WARNING: No green_area found in definition")

if "supplementary_data" in json_file:
    found_sup = 0
    for element in json_file["supplementary_data"]:
        if element["type"] == "Group":
            if element["name"] == "supplied" or element["name"] == "Supplied":
                print(bcolors.WARNING + "--- WARNING: Skipping supplied data")
                continue;
            for sub_element in element["children"]:
                if sub_element["type"] == "Group":
                    for sub_sub_element in sub_element["children"]:
                        if sub_sub_element["type"] == "Group":
                            print(bcolors.FAIL + "--- ERROR: Go deeper")
                        else:
                            for point in sub_sub_element["points"]:
                                point["lat"] = point["lat"]+float(sys.argv[3])
                                point["lon"] = point["lon"]+float(sys.argv[4])
                            found_sup = found_sup + 1
                else:
                    for point in sub_element["points"]:
                        point["lat"] = point["lat"]+float(sys.argv[3])
                        point["lon"] = point["lon"]+float(sys.argv[4])
                    found_sup = found_sup + 1
        else:
            for point in element["points"]:
                point["lat"] = point["lat"]+float(sys.argv[3])
                point["lon"] = point["lon"]+float(sys.argv[4])
            found_sup = found_sup + 1
    print(bcolors.OKGREEN + "Moved " + str(found_sup) + " supplementary items")
else:
    print(bcolors.WARNING + "--- WARNING: No supplementary data found in definition")

if "roads" in json_file["paddock"]:
    print(bcolors.FAIL + "--- ERROR: Unable to shift roads contained in the definition")

with open(sys.argv[2], "w") as write_file:
    json.dump(json_file, write_file)
