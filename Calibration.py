import yaml
import json
import sys

# use like:
# python3 Calibration.py IN.yaml IN.json OUT.yaml
# python3 Calibration.py Puppet/data/robots/yankee_slave.yaml perception_data_sanitation_tf_frame_mods.json Puppet/data/robots/yankee_slave.yaml 

#Load a yaml config into a dictionary
def load_yaml(string):

	try:
		fileread = open(string, "r")
	except:
		return -1

	text = fileread.read()

	try:
		data = yaml.load(text, Loader=yaml.FullLoader)
	except:
		return -2

	return data



#Writes a dictionary into a nicely formatted yaml
def write_yaml(dict, filepath):
    with open(filepath, 'w') as outfile:
        yaml.dump(dict, outfile, default_flow_style=False, explicit_start=True)



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

# Load current config
yaml_file = load_yaml(sys.argv[1])

# Load cam cal json
json_file = load_json(sys.argv[2])

# Set / Change cal data in YAML
print(yaml_file)
print(json_file)
yaml_file['swarmfarm::swarmbot::tf_frame_mods'] = json_file

# Save changes
write_yaml(yaml_file, sys.argv[3])

