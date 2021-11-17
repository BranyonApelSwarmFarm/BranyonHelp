import csv, sys

# use like python3 BaseStationAverager.py IN

reader = csv.reader(open(sys.argv[1]))

total_lat = 0.0
total_lon = 0.0
total_alt = 0.0
points = 0

for row in reader:
    if row[0] == "$GPGGA":
        total_lat = total_lat + float(row[2])
        total_lon = total_lon + float(row[4])
        total_alt = total_alt + float(row[9])
        points = points + 1;

print((((total_lat / float(points)) % 100) / 60) + float(int((total_lat / float(points)) / 100)))
print((((total_lon / float(points)) % 100) / 60) + float(int((total_lon / float(points)) / 100)))
print(total_alt / float(points))
print("Averaged over " + str(points) + " points")
