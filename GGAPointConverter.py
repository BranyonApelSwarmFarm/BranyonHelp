import csv, sys

# use like python3 BaseStationAverager.py IN

reader = csv.reader(open(sys.argv[1]))

total_lat = 0.0
total_lon = 0.0
total_alt = 0.0
points = 0

for row in reader:
    if row[0] == "$GPGGA":
        print(row[2][2:] + '\t' + str(float(row[2][2:])/60.0))
        print(float(row[4][2:])/60.0)
        points = points + 1;

print(total_lat / float(points))
print(total_lon / float(points))
print(total_alt / float(points))
print("Averaged over " + str(points) + " points")
