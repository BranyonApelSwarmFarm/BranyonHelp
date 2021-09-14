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

print(json_file)

with open(sys.argv[2], "w") as write_file:
    json.dump(json_file, write_file)