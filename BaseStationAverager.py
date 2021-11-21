import csv, sys

# Use like python3 BaseStationAverager.py IN
# Takes a document of serial output from a GPS, extracts the GPGGA strings and averages them

reader = csv.reader(open(sys.argv[1]))

total_lat = 0.0
total_lon = 0.0
total_alt = 0.0
points = 0.0

for row in reader:
    if row[0] == "$GPGGA":
        total_lat = total_lat + float(row[2])
        total_lon = total_lon + float(row[4])
        total_alt = total_alt + float(row[9])
        points = points + 1.0;

# This is just beautiful maths, all because GGA strings encode their data as dddmm.mmmmmm which is just stupid
print((((total_lat / points) % 100) / 60) + int(total_lat / points / 100))
print((((total_lon / points) % 100) / 60) + int(total_lon / points / 100))
print(total_alt / float(points))
print("Averaged over " + str(points) + " points")
