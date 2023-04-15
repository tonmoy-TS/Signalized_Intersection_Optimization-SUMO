import xml.etree.ElementTree as ET
import csv
import argparse

def process_element(element):
    attributes = element.attrib
    return attributes

def xml_to_csv(xml_file, csv_file):
    tree = ET.parse(xml_file)
    root = tree.getroot()

    header = set()
    rows = []
    for element in root.iter():
        attributes = process_element(element)
        header.update(attributes.keys())
        rows.append(attributes)

    with open(csv_file, 'w', newline='', encoding='utf-8') as csvfile:
        csvwriter = csv.DictWriter(csvfile, fieldnames=header)
        csvwriter.writeheader()
        for row in rows:
            csvwriter.writerow(row)

def main():
    parser = argparse.ArgumentParser(description='Convert XML file to CSV file')
    parser.add_argument('input_xml_file', help='Input XML file')
    parser.add_argument('-o', '--output_csv_file', required=False, help='Output CSV file')
    args = parser.parse_args()

    input_xml_file = args.input_xml_file
    output_csv_file = args.output_csv_file
    xml_to_csv(input_xml_file, output_csv_file)

if __name__ == '__main__':
    main()
