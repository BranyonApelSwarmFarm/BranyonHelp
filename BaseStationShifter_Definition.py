import json, sys

#Load a json config into a dictionary
def load_json(string):

	try:
		fileread = open(string, "r")
	except:
		return -1

	text = fileread.read()

	try:
		data = json.loads(text)
	except:
		return -2

	return data

json_file = load_json(sys.argv[1])

#buffer
#type
#ab
#obstacles
#green_area
#boundaries

#print(json_file)

for elem in json_file["paddock"]["obstacles"]:
    for point in elem["boundaries"]:
        point["lat"] = point["lat"]+float(sys.argv[3])
        point["lat"] = point["lat"]+float(sys.argv[4])

for point in json_file["paddock"]["boundaries"]:
    point["lat"] = point["lat"]+float(sys.argv[3])
    point["lat"] = point["lat"]+float(sys.argv[4])

for point in json_file["paddock"]["green_area"]:
    point["lat"] = point["lat"]+float(sys.argv[3])
    point["lat"] = point["lat"]+float(sys.argv[4])



for element in json_file["supplementary_data"]:
    if element["type"] == "Group":
        for sub_element in element["children"]:
            if sub_element["type"] == "Group":
                for sub_sub_element in sub_element["children"]:
                    if sub_sub_element["type"] == "Group":
                        print("Go deeper")
                    else:
                        for point in sub_sub_element["points"]:
                            point["lat"] = point["lat"]+float(sys.argv[3])
                            point["lat"] = point["lat"]+float(sys.argv[4])
            else:
                for point in sub_element["points"]:
                    point["lat"] = point["lat"]+float(sys.argv[3])
                    point["lat"] = point["lat"]+float(sys.argv[4])
    else:
        for point in element["points"]:
            point["lat"] = point["lat"]+float(sys.argv[3])
            point["lat"] = point["lat"]+float(sys.argv[4])
print()

with open(sys.argv[2], "w") as write_file:
    json.dump(json_file, write_file)