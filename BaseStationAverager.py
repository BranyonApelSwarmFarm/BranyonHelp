import csv, sys, statistics

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
       
print()

# This is just beautiful maths, all because GGA strings encode their data as dddmm.mmmmmm which is just stupid
print(str(((statistics.mean(lats) % 100) / 60) + int(statistics.mean(lats) / 100)) + "\t" + str(statistics.stdev(lats)))
print(str(((statistics.mean(lons) % 100) / 60) + int(statistics.mean(lons) / 100)) + "\t" + str(statistics.stdev(lons)))
print(str(statistics.mean(alts)) + "\t" + str(statistics.stdev(alts)))
print("Averaged over " + str(len(alts)) + " points")

print()

if (statistics.stdev(lons) < 0.000001) and (statistics.stdev(lats) < 0.000001) and (statistics.stdev(alts) < 0.1):
	print(bcolors.OKGREEN +"Stdevs look good")
elif (statistics.stdev(lons) < 0.000005) and (statistics.stdev(lats) < 0.000005) and (statistics.stdev(alts) < 0.5):
	print(bcolors.WARNING + "Stdevs are large")
else:
	print(bcolors.FAIL + "Stdevs are excessive")

if (len(alts) > 25):
	print(bcolors.OKGREEN + "Number of points is sufficient")
elif (len(alts) > 10):
	print(bcolors.WARNING + "Number of points is low")
else:
	print(bcolors.FAIL + "Not enough points")
	
print()
