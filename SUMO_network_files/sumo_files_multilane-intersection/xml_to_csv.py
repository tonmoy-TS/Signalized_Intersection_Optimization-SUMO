import argparse
import xml.etree.ElementTree as ET
import csv
import os

def xml_to_csv(input_file, output_file):
    tree = ET.parse(input_file)
    root = tree.getroot()

    with open(output_file, 'w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)

        # Write the header
        header = ['begin', 'end', 'id', 'sampledSeconds', 'numEdges', 'CO_abs', 'CO2_abs', 'HC_abs', 'PMx_abs', 'NOx_abs', 'fuel_abs', 'electricity_abs',
                  'CO_normed', 'CO2_normed', 'HC_normed', 'PMx_normed', 'NOx_normed', 'fuel_normed', 'electricity_normed',
                  'traveltime', 'CO_perVeh', 'CO2_perVeh', 'HC_perVeh', 'PMx_perVeh', 'NOx_perVeh', 'fuel_perVeh', 'electricity_perVeh']
        csv_writer.writerow(header)

        for interval in root.findall('interval'):
            row = [interval.get(attr) for attr in header]
            csv_writer.writerow(row)

def main():
    parser = argparse.ArgumentParser(description='Convert XML data to CSV format.')
    parser.add_argument('input_file', type=str, help='Path to the input XML file.')
    parser.add_argument('-o', '--output_file', type=str, default=None, help='Optional path to the output CSV file.')

    args = parser.parse_args()

    input_file = os.path.abspath(args.input_file)
    if args.output_file is None:
        output_file = os.path.splitext(input_file)[0] + '.csv'
    else:
        output_file = os.path.abspath(args.output_file)

    xml_to_csv(input_file, output_file)

if __name__ == '__main__':
    main()
