import csv, sys
reader = csv.reader(open(sys.argv[1]), delimiter="\t")
writer = csv.writer(open(sys.argv[2], 'w'), delimiter="\t")
for row in reader:
    row[0]=str(float(row[0])+float(sys.argv[3]))
    row[1]=str(float(row[1])+float(sys.argv[4]))

    print(row[0] + ',' + row[1])

    writer.writerow(row)