import json
import sys
import csv

# Save changes
write_yaml(yaml_file, sys.argv[3])

with open('/home/branyon.apel/DUMMY.txt', 'w') as f:
    json.dump(data, f)