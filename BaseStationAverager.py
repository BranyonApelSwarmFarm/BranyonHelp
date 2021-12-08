import csv, sys, statistics

# Use like python3 BaseStationAverager.py IN
# Takes a document of serial output from a GPS, extracts the GPGGA strings and averages them

reader = csv.reader(open(sys.argv[1]))

lats = []
lons = []
alts = []

for row in reader:
    if row[0] == "$GPGGA":
        lats.append(float(row[2]))
        lons.append(float(row[4]))
        alts.append(float(row[9]))

# This is just beautiful maths, all because GGA strings encode their data as dddmm.mmmmmm which is just stupid
print(str(((statistics.mean(lats) % 100) / 60) + int(statistics.mean(lats) / 100)) + "\t" + str(statistics.stdev(lats)))
print(str(((statistics.mean(lons) % 100) / 60) + int(statistics.mean(lons) / 100)) + "\t" + str(statistics.stdev(lons)))
print(str(statistics.mean(alts)) + "\t" + str(statistics.stdev(alts)))
print("Averaged over " + str(len(alts)) + " points")
