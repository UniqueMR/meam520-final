import sys
import pdb

def main():
    # Check if command line arguments were provided
    print(sys.argv)
    pdb.set_trace()

    if len(sys.argv) > 1:
        for index, arg in enumerate(sys.argv[1:], 1):  # sys.argv[1:] to skip the script name
            print(f"Argument {index}: {arg}")
    else:
        print("No arguments were provided.")

if __name__ == "__main__":
    main()