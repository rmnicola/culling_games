import csv

def load_from_csv(csv_file_path):
    with open(csv_file_path, 'r') as file:
        csv_reader = csv.reader(file)
        data_list = []
        for row in csv_reader:
            data_list.append(row)
    return data_list
