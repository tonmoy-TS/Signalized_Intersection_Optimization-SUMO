import csv
import xml.etree.ElementTree as ET
import optparse

def xml_to_csv(xml_file, csv_file):
    tree = ET.parse(xml_file)
    root = tree.getroot()

    # Open the CSV file for writing
    with open(csv_file, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)

        # Write the header row
        header = ['interval_begin', 'interval_end', 'interval_id', 'edge_id', 'sampledSeconds', 'numEdges',
                  'CO_abs', 'CO2_abs', 'HC_abs', 'PMx_abs', 'NOx_abs', 'fuel_abs', 'electricity_abs',
                  'CO_normed', 'CO2_normed', 'HC_normed', 'PMx_normed', 'NOx_normed', 'fuel_normed', 'electricity_normed',
                  'traveltime', 'CO_perVeh', 'CO2_perVeh', 'HC_perVeh', 'PMx_perVeh', 'NOx_perVeh', 'fuel_perVeh', 'electricity_perVeh']
        writer.writerow(header)

        # Iterate through the XML tree and extract the relevant data
        for interval in root.findall('./interval'):
            interval_begin = interval.get('begin')
            interval_end = interval.get('end')
            interval_id = interval.get('id')

            for edge in interval.findall('./edge'):
                edge_id = edge.get('id')
                data = [interval_begin, interval_end, interval_id, edge_id]

                # Extract the remaining attributes and add them to the data list
                for attr in header[4:]:
                    data.append(edge.get(attr))

                # Write the data to the CSV file
                writer.writerow(data)

if __name__ == '__main__':
    parser = optparse.OptionParser(usage="Usage: %prog [options] input_file_name")
    parser.add_option('-o', '--output', dest='output_file_name', help='Output file name (optional)', default=None)

    (options, args) = parser.parse_args()

    if len(args) != 1:
        parser.error("Incorrect number of arguments. Please provide input_file_name.")

    input_file_name = args[0]
    output_file_name = options.output_file_name

    xml_to_csv(input_file_name, output_file_name)