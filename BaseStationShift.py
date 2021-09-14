import csv, sys
reader = csv.reader(open(sys.argv[1]), delimiter="\t")
for row in reader:
    row[0]=str(float(row[0])+0.1)
    row[1]=str(float(row[1])+0.1)
    print(row[0] + ',' + row[1])