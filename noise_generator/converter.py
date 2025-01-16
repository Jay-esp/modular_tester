import os
from tkinter import Tk, filedialog

def convert_bits_to_bytes(input_file):
    # Determine the output file name in the same directory
    output_file = os.path.join(os.path.dirname(input_file), "slg47004_config.h")

    # Read the file and extract bit values
    with open(input_file, "r") as file:
        lines = file.readlines()
    
    # Collect bit values as integers
    bit_values = []
    for line in lines:
        parts = line.split()
        if len(parts) >= 2 and parts[1].isdigit():
            bit_values.append(int(parts[1]))

    # Group bits into 8-bit chunks and convert to bytes
    byte_values = []
    for i in range(0, len(bit_values), 8):
        chunk = bit_values[i:i + 8]
        # Pad with 0s if the last chunk is less than 8 bits
        if len(chunk) < 8:
            chunk += [0] * (8 - len(chunk))
        # Convert to a byte (integer), fix bit order here
        byte = sum(bit << idx for idx, bit in enumerate(chunk))  # Corrected bit order
        byte_values.append(byte)

    # Write the C header file
    with open(output_file, "w") as file:
        file.write("#ifndef SLG47004_CONFIG_H\n")
        file.write("#define SLG47004_CONFIG_H\n\n")
        file.write("const uint8_t slg47004_config[] = {\n")

        # Write bytes in rows of 16 for readability
        for i in range(0, len(byte_values), 16):
            row = ", ".join(f"0x{val:02X}" for val in byte_values[i:i + 16])
            file.write(f"    {row},\n")

        file.write("};\n\n")
        file.write("#endif // SLG47004_CONFIG_H\n")

    print(f"Header file created at: {output_file}")


# Main entry point with file selector
if __name__ == "__main__":
    # Open file dialog
    Tk().withdraw()  # Hide the main Tkinter window
    input_file = filedialog.askopenfilename(
        title="Select the SLG47004 Bit Configuration File",
        filetypes=[("Text Files", "*.txt"), ("All Files", "*.*")]
    )
    
    if input_file:
        convert_bits_to_bytes(input_file)
    else:
        print("No file selected. Exiting.")
