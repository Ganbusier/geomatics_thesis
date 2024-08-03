import sys

def read_ply_header(file_path):
    with open(file_path, 'rb') as f:
        lines = []
        while True:
            line = f.readline().decode('utf-8').strip()
            lines.append(line)
            if line == 'end_header':
                break
        return lines

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python read_ply_header.py <input.ply>")
        sys.exit(1)

    ply_file = sys.argv[1]
    header = read_ply_header(ply_file)

    for line in header:
        print(line)
