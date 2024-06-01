import argparse
import re
import matplotlib.pyplot as plt
import numpy as np

def search_log_file(log_file_path, keyword):
    lines = [] 

    try:
        with open(log_file_path, 'r') as file:
            for line_number, line in enumerate(file, start=1):
                if keyword in line:
                    print(f"Line {line_number}: {line.strip()}")
                    lines.append(line_number)
    except FileNotFoundError:
        print(f"Error: The file '{log_file_path}' does not exist.")
    except Exception as e:
        print(f"An error occurred: {e}")

    return lines

def search_log_file_rows(log_file_path, row_start, row_end):
    data = []
    try:
        with open(log_file_path, 'r') as file:
            for line_number, line in enumerate(file, start=1):
                if line_number >= row_start and line_number<row_end:
                    match = re.search(r'Publishing VelY to Ctrl:\s*([\d\.]+)', line)
                    if match:
                        print(f"Line {line_number}: {match.group(1)}")
                        data.append(float(match.group(1)))
    except FileNotFoundError:
        print(f"Error: The file '{log_file_path}' does not exist.")
    except Exception as e:
        print(f"An error occurred: {e}")

    return data 

def main():
    parser = argparse.ArgumentParser(description="Search for a keyword in a log file.")
    parser.add_argument('--log_file', default='logfile_deploy.log', help="The path to the log file.")
    parser.add_argument('--keyword', default="Experiment", help="The keyword to search for. Default is 'Experimen'.")
    
    args = parser.parse_args()

    lines = search_log_file(args.log_file, args.keyword)
    print(lines)

    row_start = 2
    row_end = 71
    data = search_log_file_rows(args.log_file, row_start, row_end)
    print(data)
    print(type(data))


    # Sample data
    t = np.linspace(0,10,len(data))

    # Create a new figure
    plt.figure()

    # Plot data
    plt.plot(t, data, label='y = x^2', color='b', marker='o')

    # Add a title
    plt.title('Simple 2D Line Plot')

    # Add x and y labels
    plt.xlabel('x-axis')
    plt.ylabel('y-axis')

    # Add a legend
    plt.legend()

    # Show the plot
    plt.show()

    




if __name__ == "__main__":
    main()
