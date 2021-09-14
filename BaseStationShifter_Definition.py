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

for element in json_file["paddock"]:
    print(element)

#print()
#print(json_file["paddock"]["buffer"])
#print()
#print(json_file["paddock"]["type"])
#print()
#print(json_file["paddock"]["ab"])
#print()
#print(json_file["paddock"]["obstacles"])
#print()
#print(json_file["paddock"]["green_area"])
#print()
#print(json_file["paddock"]["boundaries"])

for elem in json_file["paddock"]["obstacles"]:
    for point in elem["boundaries"]:
        point["lat"] = point["lat"]+float(sys.argv[3])
        print(point["lat"])
        print(point["lon"])
        print()
    print()


with open(sys.argv[2], "w") as write_file:
    json.dump(json_file, write_file)