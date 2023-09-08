import sys

if __name__ == "__main__":
    # print(f"Arguments count: {len(sys.argv)}")
    if len(sys.argv) != 4:
        print("Invalid Arguments. Usage: preprocess.py path/to/amc/file joint axis")
        exit()
    amcFilePath = sys.argv[1]
    jointName = sys.argv[2]
    axis = sys.argv[3]

    extractedRotation = []
    offset = 0
    if jointName == "root":
        offset = 3
    f = open(amcFilePath, "r")
    for line in f:
        seperated = line.split(" ")
        # print(seperated)
        if seperated[0] == jointName:
            if axis == "X":
                extractedRotation.append(float(seperated[1 + offset]))
            elif axis == "Y":
                extractedRotation.append(float(seperated[2 + offset]))
            elif axis == "Z":
                extractedRotation.append(float(seperated[3 + offset]))
    f.close()

    outputPathName = amcFilePath.split(".")
    outputPath = outputPathName[0] + "-extracted-" + jointName + "-" + axis + ".txt"
    f = open(outputPath, "w")
    for data in extractedRotation:
        f.write(str(data))
        f.write("\n")
    f.close()

    exit()
