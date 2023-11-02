import argparse
import os
import base64

def compress_data(input_filename, chunk_size, output_dir):
    with open(input_filename, 'rb') as input_file:
        data = input_file.read()

    index = []

    chunk_index = 0
    for i in range(0, len(data), chunk_size):
        chunk = data[i:i + chunk_size]
        compressed_chunk = base64.b64encode(chunk).decode('utf-8')
        index.append((chunk_index, len(compressed_chunk)))

        compressed_filename = os.path.join(output_dir, f"{os.path.basename(input_filename)}.{chunk_index}.bin")
        with open(compressed_filename, 'w') as compressed_file:
            compressed_file.write(compressed_chunk)
        chunk_index += 1

    index_filename = os.path.join(output_dir, f"{os.path.basename(input_filename)}.bin")
    with open(index_filename, 'w') as index_file:
        index_file.write(f"{os.path.basename(input_filename)}.bin\n")
        for start, length in index:
            index_file.write(f"{start},{length}\n")

def main():
    parser = argparse.ArgumentParser(description="Compress data and create an index")
    parser.add_argument("input_file", help="Input file to compress")
    parser.add_argument("--chunk_size", type=int, default=1024 * 1000 * 50, help="Chunk size for compression")
    args = parser.parse_args()

    output_dir = os.path.dirname(args.input_file)
    compress_data(args.input_file, args.chunk_size, output_dir)

if __name__ == '__main__':
    main()
