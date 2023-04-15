import csv

with open('battery_output.csv', 'r') as infile, open('output_batteryUse.csv', 'w', newline='') as outfile:
    reader = csv.reader(infile)
    writer = csv.writer(outfile, delimiter=',')
    
    for row in reader:
        new_row = []
        for item in row:
            if isinstance(item, str) and item != '':
                if item[0] in [':', '-'] and len(item) > 1 and item[1].isalpha():
                    if item[0] == ':':
                        item = item.replace(':', "'", 1)
                    elif item[0] == '-':
                        item = "'" + item[0] + item[1:]
                new_row.extend(item.split(';'))
        writer.writerow(new_row)

    
